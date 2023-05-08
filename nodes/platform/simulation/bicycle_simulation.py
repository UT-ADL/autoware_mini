#!/usr/bin/env python3

import threading
import math

import rospy
from tf2_ros import TransformBroadcaster
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from geometry_msgs.msg import TransformStamped, PoseStamped, TwistStamped, PoseWithCovarianceStamped, Quaternion, Point
from autoware_msgs.msg import VehicleCmd, VehicleStatus, Gear

from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA

class BicycleSimulation:

    def __init__(self):
        # get parameters
        self.publish_rate = rospy.get_param("~publish_rate")
        self.wheel_base = rospy.get_param("wheel_base")
        self.acceleration_limit = abs(rospy.get_param("acceleration_limit"))
        self.deceleration_limit = -abs(rospy.get_param("deceleration_limit"))

        # internal state of bicycle model
        self.x = 0.0
        self.y = 0.0
        self.acceleration = 0.0
        self.velocity = 0.0
        self.heading_angle = 0.0
        self.target_velocity = 0.0
        self.steering_angle = 0.0
        self.orientation = Quaternion(0, 0, 0, 1)
        self.blinkers = 0

        # localization publishers
        self.current_pose_pub = rospy.Publisher('/localization/current_pose', PoseStamped, queue_size=1)
        self.current_velocity_pub = rospy.Publisher('/localization/current_velocity', TwistStamped, queue_size=1)
        self.vehicle_status_pub = rospy.Publisher('vehicle_status', VehicleStatus, queue_size=1)
        self.br = TransformBroadcaster()

        # visualization of the bicycle model
        self.bicycle_markers_pub = rospy.Publisher('bicycle_markers', MarkerArray, queue_size=1)

        # initial position and vehicle command from outside
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.initialpose_callback)
        rospy.Subscriber('/control/vehicle_cmd', VehicleCmd, self.vehicle_cmd_callback, queue_size=1)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def initialpose_callback(self, msg):
        # extract position
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # extract heading angle from orientation
        orientation = msg.pose.pose.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        _, _, self.heading_angle = euler_from_quaternion(quaternion)

        rospy.loginfo("%s - initial position (%f, %f, %f) orientation (%f, %f, %f, %f) in %s frame", rospy.get_name(), 
                    msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
                    msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, 
                    msg.pose.pose.orientation.w, msg.header.frame_id)

    def vehicle_cmd_callback(self, msg):
        self.target_velocity = msg.ctrl_cmd.linear_velocity
        # calculate acceleration based on limits
        if self.target_velocity > self.velocity:
            if msg.ctrl_cmd.linear_acceleration > 0.0:
                self.acceleration = min(msg.ctrl_cmd.linear_acceleration, self.acceleration_limit)
            else:
                self.acceleration = self.acceleration_limit
        elif self.target_velocity < self.velocity:
            if msg.ctrl_cmd.linear_acceleration < 0.0:
                self.acceleration = -min(-msg.ctrl_cmd.linear_acceleration, self.deceleration_limit)
            else:
                self.acceleration = -self.deceleration_limit
        else:
            self.acceleration = 0.0

        rospy.logdebug("%s - target velocity: %.3f, current velocity: %.3f, acceleration: %.3f", rospy.get_name(), msg.ctrl_cmd.linear_velocity, self.velocity, self.acceleration)

        # new steering angle takes effect instantaneously
        self.steering_angle = msg.ctrl_cmd.steering_angle

        # remember blinkers, just to be able to publish status
        if msg.lamp_cmd.l == 1 and msg.lamp_cmd.r == 1:
            self.blinkers = VehicleStatus.LAMP_HAZARD
        elif msg.lamp_cmd.l == 1:
            self.blinkers = VehicleStatus.LAMP_LEFT
        elif msg.lamp_cmd.r == 1:
            self.blinkers = VehicleStatus.LAMP_RIGHT
        else:
            self.blinkers = 0

    def update_model_state(self, delta_t):
        # change velocity by acceleration
        self.velocity += self.acceleration * delta_t

        # clip velocity at 0
        self.velocity = max(self.velocity, 0)

        # compute change according to bicycle model equations
        x_dot = self.velocity * math.cos(self.heading_angle)
        y_dot = self.velocity * math.sin(self.heading_angle)
        heading_angle_dot = self.velocity * math.tan(self.steering_angle) / self.wheel_base

        # implment the change taking into account the update rate
        self.x += x_dot * delta_t
        self.y += y_dot * delta_t
        self.heading_angle += heading_angle_dot * delta_t

        # create quaternion from heading angle to be used later in tf and pose and marker messages
        x, y, z, w = quaternion_from_euler(0, 0, self.heading_angle)
        self.orientation = Quaternion(x, y, z, w)

    def run(self):
        # start separate thread for spinning subcribers
        t = threading.Thread(target=rospy.spin)
        t.daemon = True # make sure Ctrl+C works
        t.start()

        # publish localization at fixed rate
        rate = rospy.Rate(self.publish_rate)
        delta_t = 1. / self.publish_rate

        while not rospy.is_shutdown():
            # update model state
            self.update_model_state(delta_t)

            # publish localization messages and visualization markers
            stamp = rospy.Time.now()
            self.publish_base_link_to_map_tf(stamp)
            self.publish_current_pose(stamp)
            self.publish_current_velocity(stamp)
            self.publish_vehicle_status(stamp)
            self.publish_bicycle_markers(stamp)

            rate.sleep()

    def publish_base_link_to_map_tf(self, stamp):
            
        self.br.sendTransform(
            (self.x, self.y, 0.0),
            (self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w),
            stamp,
            "base_link",
            "map"
        )

    def publish_current_pose(self, stamp):

        pose_msg = PoseStamped()

        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = "map"

        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        pose_msg.pose.position.z = 0
        pose_msg.pose.orientation = self.orientation

        self.current_pose_pub.publish(pose_msg)

    def publish_current_velocity(self, stamp):
        
        vel_msg = TwistStamped()

        vel_msg.header.stamp = stamp
        vel_msg.header.frame_id = "base_link"

        vel_msg.twist.linear.x = self.velocity
        vel_msg.twist.linear.y = 0.0
        vel_msg.twist.linear.z = 0.0

        self.current_velocity_pub.publish(vel_msg)


    def publish_vehicle_status(self, stamp):
        
        status_msg = VehicleStatus()

        status_msg.header.stamp = stamp
        status_msg.header.frame_id = "base_link"

        status_msg.drivemode = VehicleStatus.MODE_AUTO
        status_msg.steeringmode = VehicleStatus.MODE_AUTO
        status_msg.current_gear.gear = Gear.DRIVE
        status_msg.speed = self.velocity * 3.6
        status_msg.angle = self.steering_angle
        status_msg.lamp = self.blinkers

        self.vehicle_status_pub.publish(status_msg)

    def publish_bicycle_markers(self, stamp):

        marker_array = MarkerArray()

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = stamp
        marker.type = marker.LINE_LIST
        marker.action = marker.ADD
        marker.id = 0
        marker.scale.x = 0.2
        marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)

        # the location of the marker is current pose
        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.pose.position.z = 1.0 # raise a bit above map
        marker.pose.orientation = self.orientation

        # draw wheel base
        marker.points.append(Point(0, 0, 0))
        marker.points.append(Point(self.wheel_base, 0, 0))

        marker_array.markers.append(marker)

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = stamp
        marker.type = marker.LINE_LIST
        marker.action = marker.ADD
        marker.id = 1
        marker.scale.x = 0.4
        marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)

        # the location of the marker is current pose
        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.pose.position.z = 1.0 # raise a bit above map
        marker.pose.orientation = self.orientation

        wheel_length = 0.4

        # draw rear wheel
        marker.points.append(Point(-wheel_length, 0, 0))
        marker.points.append(Point(wheel_length, 0, 0))

        # draw front wheel
        marker.points.append(Point(self.wheel_base + wheel_length * math.cos(self.steering_angle), wheel_length * math.sin(self.steering_angle), 0))
        marker.points.append(Point(self.wheel_base - wheel_length * math.cos(self.steering_angle), -wheel_length * math.sin(self.steering_angle), 0))

        marker_array.markers.append(marker)

        self.bicycle_markers_pub.publish(marker_array)

if __name__ == '__main__':
    rospy.init_node('bicycle_simulation', log_level=rospy.INFO)
    node = BicycleSimulation()
    node.run()
