#!/usr/bin/env python3

import rospy
import math
import message_filters
import threading
import traceback

import numpy as np
from sklearn.neighbors import NearestNeighbors

from helpers.geometry import get_heading_from_orientation, get_heading_between_two_points, normalize_heading_error, get_closest_point_on_line, get_cross_track_error
from helpers.waypoints import get_blinker_state_with_lookahead_time, get_point_and_orientation_on_path_within_distance, interpolate_velocity_between_waypoints, get_two_nearest_waypoint_idx

from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose, PoseStamped, TwistStamped
from std_msgs.msg import ColorRGBA, Float32MultiArray
from autoware_msgs.msg import Lane, VehicleCmd


class PurePursuitFollower:
    def __init__(self):

        # Parameters
        self.planning_time = rospy.get_param("~planning_time")
        self.min_lookahead_distance = rospy.get_param("~min_lookahead_distance")
        self.wheel_base = rospy.get_param("/vehicle/wheel_base")
        self.heading_angle_limit = rospy.get_param("heading_angle_limit")
        self.lateral_error_limit = rospy.get_param("lateral_error_limit")
        self.blinker_lookahead_time = rospy.get_param("blinker_lookahead_time")
        self.blinker_lookahead_distance = rospy.get_param("blinker_lookahead_distance")
        self.publish_debug_info = rospy.get_param("~publish_debug_info")
        self.nearest_neighbor_search = rospy.get_param("~nearest_neighbor_search")
        self.braking_safety_distance = rospy.get_param("/planning/braking_safety_distance")
        self.default_deceleration = rospy.get_param("/planning/default_deceleration")

        # Variables - init
        self.waypoint_tree = None
        self.waypoints = None
        self.closest_object_distance = 0.0
        self.closest_object_velocity = 0.0
        self.lock = threading.Lock()

        # Publishers
        self.vehicle_command_pub = rospy.Publisher('vehicle_cmd', VehicleCmd, queue_size=1)
        if self.publish_debug_info:
            self.pure_pursuit_markers_pub = rospy.Publisher('follower_markers', MarkerArray, queue_size=1)
            self.follower_debug_pub = rospy.Publisher('follower_debug', Float32MultiArray, queue_size=1)

        # Subscribers
        rospy.Subscriber('/planning/local_path', Lane, self.path_callback)
        current_pose_sub = message_filters.Subscriber('/localization/current_pose', PoseStamped)
        current_velocity_sub = message_filters.Subscriber('/localization/current_velocity', TwistStamped)
        ts = message_filters.ApproximateTimeSynchronizer([current_pose_sub, current_velocity_sub], queue_size=2, slop=0.02)
        ts.registerCallback(self.current_status_callback)

        # output information to console
        rospy.loginfo("%s - initialized", rospy.get_name())

    def path_callback(self, path_msg):
        
        if len(path_msg.waypoints) < 2:
            # if path is cancelled and empty waypoints received
            rospy.logwarn_throttle(30, "%s - no waypoints, stopping!", rospy.get_name())
            with self.lock:
                self.waypoint_tree = None
                self.waypoints = None
                self.closest_object_distance = 0.0
                self.closest_object_velocity = 0.0
            return

        # prepare waypoints for nearest neighbor search
        waypoints_xy = np.array([(w.pose.pose.position.x, w.pose.pose.position.y) for w in path_msg.waypoints])
        waypoint_tree = NearestNeighbors(n_neighbors=1, algorithm=self.nearest_neighbor_search).fit(waypoints_xy)
        with self.lock:
            self.waypoint_tree = waypoint_tree
            self.waypoints = path_msg.waypoints
            self.closest_object_distance = path_msg.closest_object_distance
            self.closest_object_velocity = path_msg.closest_object_velocity

    def current_status_callback(self, current_pose_msg, current_velocity_msg):

        try:

            if self.publish_debug_info:
                start_time = rospy.get_time()

            with self.lock:
                waypoints = self.waypoints
                waypoint_tree = self.waypoint_tree
                closest_object_distance = self.closest_object_distance
                closest_object_velocity = self.closest_object_velocity

            stamp = current_pose_msg.header.stamp
            current_pose = current_pose_msg.pose
            current_velocity = current_velocity_msg.twist.linear.x

            if waypoint_tree is None:
                # if no waypoints received yet or global_path cancelled, stop the vehicle
                self.publish_vehicle_command(stamp, 0.0, 0.0, 0.0, 0, 0)
                return

            # Find 2 nearest waypoint idx's on path (from base_link)
            back_wp_idx, front_wp_idx = get_two_nearest_waypoint_idx(waypoint_tree, current_pose.position.x, current_pose.position.y)

            if front_wp_idx == len(waypoints)-1:
                # stop vehicle - last waypoint is reached
                self.publish_vehicle_command(stamp, 0.0, 0.0, 0.0, 0, 0)
                rospy.logwarn_throttle(10, "%s - last waypoint reached", rospy.get_name())
                return

            # get nearest point on path from base_link
            nearest_point = get_closest_point_on_line(current_pose.position, waypoints[back_wp_idx].pose.pose.position, waypoints[front_wp_idx].pose.pose.position)

            # calc lookahead distance (velocity * planning_time)
            lookahead_distance = current_velocity * self.planning_time
            if lookahead_distance < self.min_lookahead_distance:
                lookahead_distance = self.min_lookahead_distance

            cross_track_error = get_cross_track_error(current_pose.position, waypoints[back_wp_idx].pose.pose.position, waypoints[front_wp_idx].pose.pose.position)

            # lookahead_point - point on the path within given lookahead distance
            lookahead_point, _ = get_point_and_orientation_on_path_within_distance(waypoints, front_wp_idx, nearest_point, lookahead_distance)

            # find current_heading, lookahead_heading, heading error and cross_track_error
            current_heading = get_heading_from_orientation(current_pose.orientation)
            lookahead_heading = get_heading_between_two_points(current_pose.position, lookahead_point)
            heading_error = lookahead_heading - current_heading

            heading_angle_difference = normalize_heading_error(heading_error)

            if abs(cross_track_error) > self.lateral_error_limit or abs(math.degrees(heading_angle_difference)) > self.heading_angle_limit:
                # stop vehicle if cross track error or heading angle difference is over limit
                self.publish_vehicle_command(stamp, 0.0, 0.0, 0.0, 0, 0)
                rospy.logerr_throttle(10, "%s - lateral error or heading angle difference over limit", rospy.get_name())
                return
        
            # calculate steering angle
            curvature = 2 * math.sin(heading_error) / lookahead_distance
            steering_angle = math.atan(self.wheel_base * curvature)

            # target_velocity from map and based on closest object
            target_velocity = interpolate_velocity_between_waypoints(nearest_point, waypoints[back_wp_idx], waypoints[front_wp_idx])

            # if decelerating because of obstacle then calculate necessary deceleration to stop at safety distance
            if closest_object_distance - self.braking_safety_distance > 0:
                # always allow minimum deceleration, to be able to adapt to map speeds
                acceleration = min(0.5 * (closest_object_velocity**2 - current_velocity**2) / (closest_object_distance - self.braking_safety_distance), -self.default_deceleration)
            # otherwise use vehicle default deceleration limit
            else:
                acceleration = 0.0

            # fix for Carla - acceleration cannot be negative if target velocity is higher than current velocity
            if acceleration < 0.0 and target_velocity > current_velocity:
                acceleration = 0.0

            # blinkers
            left_blinker, right_blinker = get_blinker_state_with_lookahead_time(waypoints, front_wp_idx, current_velocity, self.blinker_lookahead_time, self.blinker_lookahead_distance)

            # Publish
            self.publish_vehicle_command(stamp, steering_angle, target_velocity, acceleration, left_blinker, right_blinker)
            if self.publish_debug_info:
                self.publish_pure_pursuit_markers(stamp, current_pose, lookahead_point, heading_angle_difference)
                self.follower_debug_pub.publish(Float32MultiArray(data=[1.0 / (rospy.get_time() - start_time), current_heading, lookahead_heading, heading_error, cross_track_error, target_velocity]))

        except Exception as e:
            rospy.logerr_throttle(10, "%s - Exception in callback: %s", rospy.get_name(), traceback.format_exc())


    def publish_vehicle_command(self, stamp, steering_angle, target_velocity, acceleration, left_blinker, right_blinker):
        vehicle_cmd = VehicleCmd()
        vehicle_cmd.header.stamp = stamp
        vehicle_cmd.header.frame_id =  "base_link"
        # blinkers
        vehicle_cmd.lamp_cmd.l = left_blinker
        vehicle_cmd.lamp_cmd.r = right_blinker
        # velocity and steering
        vehicle_cmd.ctrl_cmd.linear_velocity = target_velocity
        vehicle_cmd.ctrl_cmd.linear_acceleration = acceleration
        vehicle_cmd.ctrl_cmd.steering_angle = steering_angle
        self.vehicle_command_pub.publish(vehicle_cmd)


    def publish_pure_pursuit_markers(self, stamp, current_pose, lookahead_point, heading_error):

        marker_array = MarkerArray()

        # draws a line between current pose and lookahead point
        marker = Marker()
        marker.header.frame_id =  "map"
        marker.header.stamp = stamp
        marker.ns = "Lookahead line"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.color = ColorRGBA(1.0, 0.0, 1.0, 1.0)
        marker.points = ([current_pose.position, lookahead_point])
        marker_array.markers.append(marker)

        # label heading_error
        average_pose = Pose()
        average_pose.position.x = (current_pose.position.x + lookahead_point.x) / 2
        average_pose.position.y = (current_pose.position.y + lookahead_point.y) / 2
        average_pose.position.z = (current_pose.position.z + lookahead_point.z) / 2

        marker_text = Marker()
        marker_text.header.frame_id =  "map"
        marker_text.header.stamp = stamp
        marker_text.ns = "Heading error"
        marker_text.id = 1
        marker_text.type = Marker.TEXT_VIEW_FACING
        marker_text.action = Marker.ADD
        marker_text.pose = average_pose
        marker_text.scale.z = 0.6
        marker_text.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
        marker_text.text = str(round(math.degrees(heading_error),1))
        marker_array.markers.append(marker_text)

        self.pure_pursuit_markers_pub.publish(marker_array)


    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('pure_pursuit_follower', log_level=rospy.INFO)
    node = PurePursuitFollower()
    node.run()