#!/usr/bin/env python3

import rospy
import math
import message_filters
import threading
import traceback
from shapely.geometry import Point as ShapelyPoint

from helpers.geometry import get_heading_from_orientation, get_heading_between_two_points, normalize_heading_error, get_point_using_heading_and_distance
from helpers.path import Path

from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose, PoseStamped, TwistStamped, Point
from std_msgs.msg import ColorRGBA, Float32MultiArray
from autoware_msgs.msg import Lane, VehicleCmd

class StanleyFollower:
    def __init__(self):

         # Parameters
        self.cte_gain = rospy.get_param("~cte_gain")       # gain for cross track error
        self.wheel_base = rospy.get_param("/vehicle/wheel_base")
        self.heading_angle_limit = rospy.get_param("heading_angle_limit")
        self.lateral_error_limit = rospy.get_param("lateral_error_limit")
        self.blinker_lookahead_time = rospy.get_param("blinker_lookahead_time")
        self.blinker_min_lookahead_distance = rospy.get_param("blinker_min_lookahead_distance")
        self.current_pose_to_car_front = rospy.get_param("/planning/current_pose_to_car_front")
        self.publish_debug_info = rospy.get_param("~publish_debug_info")
        self.default_acceleration = rospy.get_param("/planning/default_acceleration")
        self.default_deceleration = rospy.get_param("/planning/default_deceleration")
        self.max_deceleration = rospy.get_param("/planning/max_deceleration")
        self.stopping_speed_limit = rospy.get_param("/planning/stopping_speed_limit")
        self.simulate_cmd_delay = rospy.get_param("~simulate_cmd_delay")

        # Variables - init
        self.path = None
        self.closest_object_velocity = 0.0
        self.stopping_point_distance = 0.0
        self.lock = threading.Lock()

        # Publishers
        self.vehicle_command_pub = rospy.Publisher('vehicle_cmd', VehicleCmd, queue_size=1, tcp_nodelay=True)
        if self.publish_debug_info:
            self.stanley_markers_pub = rospy.Publisher('follower_markers', MarkerArray, queue_size=1, tcp_nodelay=True)
            self.follower_debug_pub = rospy.Publisher('follower_debug', Float32MultiArray, queue_size=1, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber('/planning/local_path', Lane, self.path_callback, queue_size=1, buff_size=2**20, tcp_nodelay=True)
        current_pose_sub = message_filters.Subscriber('/localization/current_pose', PoseStamped, queue_size=1, tcp_nodelay=True)
        current_velocity_sub = message_filters.Subscriber('/localization/current_velocity', TwistStamped, queue_size=1, tcp_nodelay=True)
        ts = message_filters.ApproximateTimeSynchronizer([current_pose_sub, current_velocity_sub], queue_size=2, slop=0.02)
        ts.registerCallback(self.current_status_callback)

        # output information to console
        rospy.loginfo("%s - initialized", rospy.get_name())


    def path_callback(self, path_msg):

        if len(path_msg.waypoints) == 0:
            # if path is cancelled and empty waypoints received
            path = None
            closest_object_velocity = 0.0
            stopping_point_distance = 0.0
        else:
            path = Path(path_msg.waypoints, velocities=True, blinkers=True)
            closest_object_velocity = path_msg.closest_object_velocity
            stopping_point_distance = path_msg.cost

        with self.lock:
            self.path = path
            self.closest_object_velocity = closest_object_velocity
            self.stopping_point_distance = stopping_point_distance


    def current_status_callback(self, current_pose_msg, current_velocity_msg):

        try:

            if self.publish_debug_info:
                start_time = rospy.get_time()

            with self.lock:
                path = self.path
                closest_object_velocity = self.closest_object_velocity
                stopping_point_distance = self.stopping_point_distance

            stamp = current_pose_msg.header.stamp

            if path is None:
                self.publish_vehicle_command(stamp)
                rospy.logwarn_throttle(30, "%s - no waypoints, stopping!", rospy.get_name())
                return

            current_position = current_pose_msg.pose.position
            current_velocity = current_velocity_msg.twist.linear.x
            current_heading = get_heading_from_orientation(current_pose_msg.pose.orientation)

            cross_track_error = path.get_cross_track_error(current_position)

            # simulate delay in vehicle command. Project car into future location and use it to calculate current steering command
            if self.simulate_cmd_delay > 0.0:
                current_position.x += math.cos(current_heading) * current_velocity * self.simulate_cmd_delay
                current_position.y += math.sin(current_heading) * current_velocity * self.simulate_cmd_delay

            ego_distance_from_path_start = path.linestring.project(ShapelyPoint(current_position.x, current_position.y))

            # if "waypoint planner" is used and no global and local planner involved
            if ego_distance_from_path_start >= path.linestring.length:
                self.publish_vehicle_command(stamp)
                rospy.logwarn_throttle(10, "%s - end of path reached", rospy.get_name())
                return

            front_wheel_position = get_point_using_heading_and_distance(current_position, current_heading, self.wheel_base)
            front_wheel_distance_from_path_start = path.linestring.project(ShapelyPoint(front_wheel_position.x, front_wheel_position.y, front_wheel_position.z))
            front_wheel_on_path = path.linestring.interpolate(front_wheel_distance_from_path_start)

            lookahead_on_path = path.linestring.interpolate(front_wheel_distance_from_path_start + self.wheel_base/10)
            lookback_on_path = path.linestring.interpolate(max(0, front_wheel_distance_from_path_start - self.wheel_base/10))
            track_heading = get_heading_between_two_points(lookback_on_path, lookahead_on_path)
            heading_error = normalize_heading_error(track_heading - current_heading)

            if abs(cross_track_error) > self.lateral_error_limit or abs(math.degrees(heading_error)) > self.heading_angle_limit:
                # stop vehicle if cross track error or heading angle difference is over limit
                self.publish_vehicle_command(stamp)
                rospy.logerr_throttle(10, "%s - lateral error or heading angle difference over limit", rospy.get_name())
                return

            # calculate steering angle
            delta_error = math.atan(self.cte_gain * cross_track_error / (current_velocity + 0.0001))
            steering_angle = heading_error + delta_error

            # find velocity at current position
            target_velocity = path.get_velocity_at_distance(ego_distance_from_path_start)
            if target_velocity < self.stopping_speed_limit:
                target_velocity = 0.0

            # if decelerating because of obstacle then calculate necessary deceleration
            emergency = 0
            if stopping_point_distance > 0.0 and target_velocity < current_velocity:
                # calculate distance from car front to stopping point
                car_front_to_stopping_point_distance = stopping_point_distance - ego_distance_from_path_start - self.current_pose_to_car_front
                if car_front_to_stopping_point_distance > 0:
                    # always allow minimum deceleration, to be able to adapt to map speeds
                    acceleration = min(0.5 * (closest_object_velocity**2 - current_velocity**2) / car_front_to_stopping_point_distance, -self.default_deceleration)
                else:
                    # emergency braking - car front over the stopping point
                    acceleration = -self.max_deceleration
                    emergency = 1
            else:
                # use the default acceleration/deceleration limits
                if target_velocity > current_velocity:
                    acceleration = self.default_acceleration
                else:
                    acceleration = -self.default_deceleration

            blinker_lookahead_distance = max(self.blinker_min_lookahead_distance, self.blinker_lookahead_time * current_velocity)
            left_blinker, right_blinker = path.get_blinker_state_with_lookahead(ego_distance_from_path_start, blinker_lookahead_distance)

            # Publish
            self.publish_vehicle_command(stamp, steering_angle, target_velocity, acceleration, left_blinker, right_blinker, emergency)
            if self.publish_debug_info:
                # convert shapely 2d points to geometry_msg/Point
                lookback_on_path_3d = Point(x=lookback_on_path.x, y=lookback_on_path.y, z=current_pose_msg.pose.position.z)
                lookahead_on_path_3d = Point(x=lookahead_on_path.x, y=lookahead_on_path.y, z=current_pose_msg.pose.position.z)
                front_wheel_on_path_3d = Point(x=front_wheel_on_path.x, y=front_wheel_on_path.y, z=current_pose_msg.pose.position.z)
                front_wheel_position_3d = Point(x=front_wheel_position.x, y=front_wheel_position.y, z=current_pose_msg.pose.position.z)

                self.publish_stanley_markers(current_pose_msg.header, lookback_on_path_3d, lookahead_on_path_3d, front_wheel_on_path_3d, front_wheel_position_3d, heading_error)
                self.follower_debug_pub.publish(Float32MultiArray(data=[(rospy.get_time() - start_time), current_heading, track_heading, heading_error, cross_track_error, target_velocity]))

        except Exception as e:
            rospy.logerr_throttle(10, "%s - Exception in callback: %s", rospy.get_name(), traceback.format_exc())


    def publish_vehicle_command(self, stamp, steering_angle=0.0, target_velocity=0.0, acceleration=0.0, left_blinker=0, right_blinker=0, emergency=0):
        vehicle_cmd = VehicleCmd()
        vehicle_cmd.header.stamp = stamp
        vehicle_cmd.header.frame_id = "base_link"
        # blinkers
        vehicle_cmd.lamp_cmd.l = left_blinker 
        vehicle_cmd.lamp_cmd.r = right_blinker 
        # velocity and steering
        vehicle_cmd.ctrl_cmd.linear_velocity = target_velocity
        vehicle_cmd.ctrl_cmd.linear_acceleration = acceleration
        vehicle_cmd.ctrl_cmd.steering_angle = steering_angle
        vehicle_cmd.emergency = emergency
        self.vehicle_command_pub.publish(vehicle_cmd)


    def publish_stanley_markers(self, header, lookback_on_path_3d, lookahead_on_path_3d, front_wheel_on_path_3d, front_wheel_position_3d, heading_error):

        marker_array = MarkerArray()

        # draws a line between current pose and nearest_wp point
        marker = Marker(header=header)
        marker.ns = "Line"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.color = ColorRGBA(1.0, 0.0, 1.0, 1.0)
        marker.points = ([lookback_on_path_3d, lookahead_on_path_3d, front_wheel_on_path_3d, front_wheel_position_3d])
        marker_array.markers.append(marker)

        # label of angle alpha
        average_pose = Pose()
        average_pose.position.x = (lookahead_on_path_3d.x + front_wheel_position_3d.x) / 2
        average_pose.position.y = (lookahead_on_path_3d.y + front_wheel_position_3d.y) / 2
        average_pose.position.z = (lookahead_on_path_3d.z + front_wheel_position_3d.z) / 2
        average_pose.orientation.w = 1.0

        marker_text = Marker(header=header)
        marker_text.ns = "Heading error"
        marker_text.id = 1
        marker_text.type = Marker.TEXT_VIEW_FACING
        marker_text.action = Marker.ADD
        marker_text.pose = average_pose
        marker_text.scale.z = 0.6
        marker_text.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
        marker_text.text = str(round(math.degrees(heading_error),1))
        marker_array.markers.append(marker_text)

        self.stanley_markers_pub.publish(marker_array)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('stanley_follower', log_level=rospy.INFO)
    node = StanleyFollower()
    node.run()