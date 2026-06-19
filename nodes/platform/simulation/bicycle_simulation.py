#!/usr/bin/env python3

import math
import traceback
from copy import deepcopy

import rospy
import tf2_ros

from geometry_msgs.msg import PoseStamped, TwistStamped, PoseWithCovarianceStamped, Quaternion, Point, Pose
from autoware_mini.msg import VehicleCommand, VehicleStatus

from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
from carla_msgs.msg import CarlaControl, CarlaStatus

from autoware_mini.geometry import get_orientation_from_heading, get_heading_from_orientation
from autoware_mini.lanelet2 import load_lanelet2_map, get_height_at_position
from autoware_mini.transform import transform_pose, transform_to_pose, pose_to_transform

class BicycleSimulation:

    def __init__(self):
        # get parameters
        self.publish_rate = rospy.get_param("~publish_rate")
        self.broadcast_tf = rospy.get_param("~broadcast_tf")
        self.wheel_base = rospy.get_param("wheel_base")
        self.acceleration_limit = rospy.get_param("acceleration_limit")
        self.deceleration_limit = rospy.get_param("deceleration_limit")
        self.default_acceleration = rospy.get_param("/planning/default_acceleration")
        self.default_deceleration = rospy.get_param("/planning/default_deceleration")
        self.lanelet2_map = load_lanelet2_map(rospy.get_param("~lanelet2_map_path"))

        # internal state of bicycle model
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.acceleration = 0.0
        self.speed = 0.0
        self.heading_angle = 0.0
        self.target_speed = 0.0
        self.steering_angle = 0.0
        self.turn_signal = VehicleStatus.TURN_STRAIGHT

        # optional initial pose from map config (loaded into /localization by localization.launch)
        initial_position = rospy.get_param('/localization/initial_position', None)
        if initial_position is not None:
            position = initial_position['position']
            orientation = initial_position['orientation']
            self.x = position['x']
            self.y = position['y']
            self.z = get_height_at_position(self.lanelet2_map, self.x, self.y, position.get('z', 0.0))
            quaternion = Quaternion(x=orientation['x'], y=orientation['y'], z=orientation['z'], w=orientation['w'])
            self.heading_angle = get_heading_from_orientation(quaternion)
            rospy.loginfo("%s - initial pose from map config: (%f, %f, %f) heading %f rad",
                          rospy.get_name(), self.x, self.y, self.z, self.heading_angle)

        # localization publishers
        self.current_pose_pub = rospy.Publisher('/localization/current_pose', PoseStamped, queue_size=1, tcp_nodelay=True)
        self.current_velocity_pub = rospy.Publisher('/localization/current_velocity', TwistStamped, queue_size=1, tcp_nodelay=True)
        self.vehicle_status_pub = rospy.Publisher('vehicle_status', VehicleStatus, queue_size=1, tcp_nodelay=True)
        self.carla_status_pub = rospy.Publisher('/carla/status', CarlaStatus, queue_size=10, latch=True)

        # publish initial status as running
        self.paused = False
        self.publish_carla_status()

        self.tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.br = tf2_ros.TransformBroadcaster()

        # visualization of the bicycle model
        self.bicycle_markers_pub = rospy.Publisher('bicycle_markers', MarkerArray, queue_size=1, tcp_nodelay=True)

        # initial position and vehicle command from outside
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.initialpose_callback, queue_size=None, tcp_nodelay=True)
        rospy.Subscriber('/initialvelocity', TwistStamped, self.initialvelocity_callback, queue_size=None, tcp_nodelay=True)
        rospy.Subscriber('/control/vehicle_cmd', VehicleCommand, self.vehicle_cmd_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/carla/control', CarlaControl, self.carla_control_callback, queue_size=None)

        # static base_footprint -> base_link offset, used to lift the road-level model position to base_link
        # so current_pose and the map -> base_link TF report the base_link pose, as on the real vehicle. The
        # timeout is generous because in vehicle-in-the-loop base_footprint is published by Carla only once
        # the ego is spawned, which can take a while after startup (the /carla/status published above lets
        # the spawn proceed while this blocks).
        transform = self.tf_buffer.lookup_transform("base_footprint", "base_link", rospy.Time(0), rospy.Duration(100))
        self.base_footprint_to_base_link = transform_to_pose(transform)

        # start the update and publish loop
        rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.update_and_publish)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def initialpose_callback(self, msg):

        if msg.header.frame_id != "map":
            # Convert base_link pose to map frame using transform_pose and the existing tf_buffer
            transform = self.tf_buffer.lookup_transform(
                "map", msg.header.frame_id, msg.header.stamp, rospy.Duration(0.06)
            )
            msg.pose.pose = transform_pose(transform, msg.pose.pose)

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # set z coordinate from nearest lanelet
        self.z = get_height_at_position(self.lanelet2_map, msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)

        # extract heading angle from orientation
        self.heading_angle = get_heading_from_orientation(msg.pose.pose.orientation)

        rospy.loginfo("%s - initial position (%f, %f, %f) orientation (%f, %f, %f, %f) in %s frame", rospy.get_name(),
                    msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
                    msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w, msg.header.frame_id)

    def initialvelocity_callback(self, msg):
        # extract speed
        self.speed = msg.twist.linear.x

        rospy.loginfo("%s - initial speed %f", rospy.get_name(), self.speed)

    def vehicle_cmd_callback(self, msg):
        self.target_speed = msg.speed
        # calculate acceleration based on limits
        if self.target_speed > self.speed:
            if msg.acceleration > 0.0:
                self.acceleration = min(msg.acceleration, self.acceleration_limit)
            else:
                self.acceleration = self.default_acceleration
        elif self.target_speed < self.speed:
            if msg.acceleration < 0.0:
                self.acceleration = max(msg.acceleration, -self.deceleration_limit)
            else:
                self.acceleration = -self.default_deceleration
        else:
            self.acceleration = 0.0

        rospy.logdebug("%s - target speed: %.3f, current speed: %.3f, acceleration: %.3f", rospy.get_name(), msg.speed, self.speed, self.acceleration)

        # new steering angle takes effect instantaneously
        self.steering_angle = msg.steering_angle

        # remember turn signal state, just to be able to publish status
        self.turn_signal = msg.turn_signal

    def update_model_state(self, delta_t):
        # change speed by acceleration
        self.speed += self.acceleration * delta_t

        # clip speed at 0
        self.speed = max(self.speed, 0.0)

        # compute change according to bicycle model equations
        x_dot = self.speed * math.cos(self.heading_angle)
        y_dot = self.speed * math.sin(self.heading_angle)
        heading_angle_dot = self.speed * math.tan(self.steering_angle) / self.wheel_base

        # implment the change taking into account the update rate
        self.x += x_dot * delta_t
        self.y += y_dot * delta_t
        self.heading_angle += heading_angle_dot * delta_t

        # set z coordinate from nearby lanelets, check lanelets within a radius and choose the first one that has height within 3 meters of current z
        self.z = get_height_at_position(self.lanelet2_map, self.x, self.y, self.z, search_radius=2.0, max_height_difference=3.0)

        # the model integrates base_footprint at road level; build the pose there, then lift it to base_link
        # so current_pose and the map -> base_link TF report the base_link pose, as on the real vehicle
        self.current_pose = Pose(position=Point(x=self.x, y=self.y, z=self.z), orientation=get_orientation_from_heading(self.heading_angle))

        # map -> base_link = (map -> base_footprint) * (base_footprint -> base_link)
        self.current_pose = transform_pose(pose_to_transform(self.current_pose), self.base_footprint_to_base_link)

    def update_and_publish(self, timer_event):
        try:
            if self.paused:
                return

            # update model state
            self.update_model_state(1.0 / self.publish_rate)

            # publish localization messages and visualization markers
            stamp = rospy.Time.now()
            if self.broadcast_tf:
                self.publish_base_link_to_map_tf(stamp)
            self.publish_current_pose(stamp)
            self.publish_current_velocity(stamp)
            self.publish_vehicle_status(stamp)
            self.publish_bicycle_markers(stamp)
        except Exception as e:
            rospy.logerr_throttle(10, "%s - Exception in callback: %s", rospy.get_name(), traceback.format_exc())

    def carla_control_callback(self, msg):
        if msg.command == CarlaControl.PLAY:
            self.paused = False
            self.publish_carla_status()
            rospy.loginfo("Simulation resumed")

        elif msg.command == CarlaControl.PAUSE:
            self.paused = True
            self.publish_carla_status()
            rospy.loginfo("Simulation paused")

        elif msg.command == CarlaControl.STEP_ONCE:
            self.paused = True
            self.publish_carla_status()
            # play for 0.1 seconds
            self.paused = False
            self.publish_carla_status()
            rospy.sleep(0.1)
            # pause again
            self.paused = True
            self.publish_carla_status()
            rospy.loginfo("Stepped 0.1 seconds")

    def publish_carla_status(self):
        carla_status = CarlaStatus()
        carla_status.frame = 0
        carla_status.fixed_delta_seconds = 0.0
        carla_status.synchronous_mode = True
        carla_status.synchronous_mode_running = not self.paused
        self.carla_status_pub.publish(carla_status)

    def publish_base_link_to_map_tf(self, stamp):

        t = pose_to_transform(self.current_pose)
        t.header.stamp = stamp
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"

        self.br.sendTransform(t)

    def publish_current_pose(self, stamp):

        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = "map"
        pose_msg.pose = self.current_pose

        self.current_pose_pub.publish(pose_msg)

    def publish_current_velocity(self, stamp):

        vel_msg = TwistStamped()

        vel_msg.header.stamp = stamp
        vel_msg.header.frame_id = "base_link"

        vel_msg.twist.linear.x = self.speed
        vel_msg.twist.linear.y = 0.0
        vel_msg.twist.linear.z = 0.0

        self.current_velocity_pub.publish(vel_msg)


    def publish_vehicle_status(self, stamp):

        status_msg = VehicleStatus()

        status_msg.header.stamp = stamp
        status_msg.header.frame_id = "base_link"

        status_msg.drivemode = VehicleStatus.MODE_AUTO
        status_msg.steeringmode = VehicleStatus.MODE_AUTO
        status_msg.gear = VehicleStatus.GEAR_DRIVE
        status_msg.speed = self.speed
        status_msg.angle = self.steering_angle
        status_msg.turn_signal = self.turn_signal

        self.vehicle_status_pub.publish(status_msg)

    def publish_bicycle_markers(self, stamp):

        marker_array = MarkerArray()

        # all markers share the current pose, raised a bit above the map for visibility
        marker_pose = deepcopy(self.current_pose)
        marker_pose.position.z += 1.0

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = stamp
        marker.type = marker.LINE_LIST
        marker.action = marker.ADD
        marker.id = 0
        marker.scale.x = 0.2
        marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)
        marker.pose = marker_pose

        # draw wheel base
        marker.points.append(Point(x=0.0, y=0.0, z=0.0))
        marker.points.append(Point(x=self.wheel_base, y=0.0, z=0.0))

        marker_array.markers.append(marker)

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = stamp
        marker.type = marker.LINE_LIST
        marker.action = marker.ADD
        marker.id = 1
        marker.scale.x = 0.4
        marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)
        marker.pose = marker_pose

        wheel_length = 0.4

        # draw rear wheel
        marker.points.append(Point(x=-wheel_length, y=0.0, z=0.0))
        marker.points.append(Point(x=wheel_length, y=0.0, z=0.0))

        # draw front wheel
        marker.points.append(Point(x=self.wheel_base + wheel_length * math.cos(self.steering_angle), y=wheel_length * math.sin(self.steering_angle), z=0.0))
        marker.points.append(Point(x=self.wheel_base - wheel_length * math.cos(self.steering_angle), y=-wheel_length * math.sin(self.steering_angle), z=0.0))

        marker_array.markers.append(marker)

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = stamp
        marker.type = marker.SPHERE_LIST
        if self.turn_signal in (VehicleStatus.TURN_HAZARD, VehicleStatus.TURN_LEFT, VehicleStatus.TURN_RIGHT):
            marker.action = marker.ADD
        else:
            marker.action = marker.DELETE
        marker.id = 2
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color = ColorRGBA(0.0, 1.0, 0.0, round((rospy.get_time() % 0.5) * 2))
        marker.pose = marker_pose

        # draw turn signals
        if self.turn_signal in (VehicleStatus.TURN_LEFT, VehicleStatus.TURN_HAZARD):
            marker.points.append(Point(x=self.wheel_base, y=0.5, z=0.0))
            marker.points.append(Point(x=0.0, y=0.5, z=0.0))

        if self.turn_signal in (VehicleStatus.TURN_RIGHT, VehicleStatus.TURN_HAZARD):
            marker.points.append(Point(x=self.wheel_base, y=-0.5, z=0.0))
            marker.points.append(Point(x=0.0, y=-0.5, z=0.0))

        marker_array.markers.append(marker)

        self.bicycle_markers_pub.publish(marker_array)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('bicycle_simulation', log_level=rospy.INFO)
    node = BicycleSimulation()
    node.run()
