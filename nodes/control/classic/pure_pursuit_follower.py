#!/usr/bin/env python

import rospy
import math
import message_filters
import threading
import numpy as np
from sklearn.neighbors import KDTree

from helpers import get_heading_from_pose_orientation, get_heading_between_two_points, get_blinker_state, \
    normalize_heading_error, get_point_on_path_within_distance, get_closest_point, \
    get_cross_track_error

from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose, PoseStamped, TwistStamped
from std_msgs.msg import ColorRGBA, Float32MultiArray
from autoware_msgs.msg import Lane, VehicleCmd


class PurePursuitFollower:
    def __init__(self):

        # Parameters
        self.planning_time = rospy.get_param("~planning_time", 2.0)
        self.min_lookahead_distance = rospy.get_param("~min_lookahead_distance", 6.0)
        self.wheel_base = rospy.get_param("~wheel_base", 2.789)
        self.heading_angle_limit = rospy.get_param("~heading_angle_limit", 90.0)
        self.lateral_error_limit = rospy.get_param("~lateral_error_limit", 2.0)
        self.publish_debug_info = rospy.get_param("~publish_debug_info", False)

        # Variables - init
        self.waypoint_tree = None
        self.waypoints = None
        self.lock = threading.Lock()

        # Subscribers
        self.path_sub = rospy.Subscriber('path', Lane, self.path_callback)
        self.current_pose_sub = message_filters.Subscriber('current_pose', PoseStamped)
        self.current_velocity_sub = message_filters.Subscriber('current_velocity', TwistStamped)
        ts = message_filters.ApproximateTimeSynchronizer([self.current_pose_sub, self.current_velocity_sub], queue_size=10, slop=0.1)
        ts.registerCallback(self.current_status_callback)

        # Publishers
        self.vehicle_command_pub = rospy.Publisher('vehicle_cmd', VehicleCmd, queue_size=1)
        if self.publish_debug_info:
            self.pure_pursuit_markers_pub = rospy.Publisher('follower_markers', MarkerArray, queue_size=1)
            self.follower_debug_pub = rospy.Publisher('follower_debug', Float32MultiArray, queue_size=1)

        # output information to console
        rospy.loginfo("pure_pursuit_follower - initialized")

    def path_callback(self, path_msg):
        
        if len(path_msg.waypoints) == 0:
            # if path is cancelled and empty waypoints received
            rospy.logwarn_throttle(30, "pure_pursuit_follower - no waypoints received or path cancelled, stopping!")
            self.lock.acquire()
            self.waypoint_tree = None
            self.waypoints = None
            self.lock.release()
            return

        # create kd-tree for nearest neighbor search
        waypoints_xy = np.array([(w.pose.pose.position.x, w.pose.pose.position.y) for w in path_msg.waypoints])
        waypoint_tree = KDTree(waypoints_xy)
        self.lock.acquire()
        self.waypoint_tree = waypoint_tree
        self.waypoints = path_msg.waypoints
        self.lock.release()


    def current_status_callback(self, current_pose_msg, current_velocity_msg):

        self.lock.acquire()
        waypoints = self.waypoints
        waypoint_tree = self.waypoint_tree
        self.lock.release()

        stamp = current_pose_msg.header.stamp
        current_pose = current_pose_msg.pose
        current_velocity = current_velocity_msg.twist.linear.x

        if waypoint_tree is None:
            # if no waypoints received yet or global_path cancelled, stop the vehicle
            self.publish_vehicle_command(stamp, 0.0, 0.0, 0, 0)
            return

        if self.publish_debug_info:
            start_time = rospy.get_time()

        # Find 2 nearest waypoint idx's on path (from base_link)
        back_wp_idx, front_wp_idx = self.find_two_nearest_waypoint_idx(waypoint_tree, current_pose.position.x, current_pose.position.y)

        if front_wp_idx == len(waypoints)-1:
            # stop vehicle - last waypoint is reached
            self.publish_vehicle_command(stamp, 0.0, 0.0, 0, 0)
            rospy.logwarn_throttle(10, "pure_pursuit_follower - last waypoint reached")
            return

        # get nearest point on path from base_link
        nearest_point = get_closest_point(current_pose.position, waypoints[back_wp_idx].pose.pose.position, waypoints[front_wp_idx].pose.pose.position)
        
        # calc lookahead distance (velocity * planning_time)
        lookahead_distance = current_velocity * self.planning_time
        if lookahead_distance < self.min_lookahead_distance:
            lookahead_distance = self.min_lookahead_distance

        cross_track_error = get_cross_track_error(current_pose, waypoints[back_wp_idx].pose.pose, waypoints[front_wp_idx].pose.pose)

        # lookahead_point - point on the path within given lookahead distance
        lookahead_point = get_point_on_path_within_distance(waypoints, front_wp_idx, nearest_point, lookahead_distance)

        # find current_heading, lookahead_heading, heading error and cross_track_error
        current_heading = get_heading_from_pose_orientation(current_pose)
        lookahead_heading = get_heading_between_two_points(current_pose.position, lookahead_point)
        heading_error = lookahead_heading - current_heading

        heading_angle_difference = normalize_heading_error(heading_error)

        if abs(cross_track_error) > self.lateral_error_limit or abs(math.degrees(heading_angle_difference)) > self.heading_angle_limit:
            # stop vehicle if cross track error or heading angle difference is over limit
            self.publish_vehicle_command(stamp, 0.0, 0.0, 0, 0)
            rospy.logerr_throttle(10, "pure_pursuit_follower - lateral error or heading angle difference over limit")
            return
    
        # calculate steering angle
        curvature = 2 * math.sin(heading_error) / lookahead_distance
        steering_angle = math.atan(self.wheel_base * curvature)

        # target_velocity and blinkers
        target_velocity = waypoints[front_wp_idx].twist.twist.linear.x
        left_blinker, right_blinker = get_blinker_state(waypoints[front_wp_idx].wpstate.steering_state)

        # Publish
        self.publish_vehicle_command(stamp, steering_angle, target_velocity, left_blinker, right_blinker)
        if self.publish_debug_info:
            self.publish_pure_pursuit_markers(stamp, current_pose, lookahead_point, heading_angle_difference)
            self.follower_debug_pub.publish(Float32MultiArray(data=[1.0 / (rospy.get_time() - start_time), current_heading, lookahead_heading, heading_error, cross_track_error, target_velocity]))


    def find_two_nearest_waypoint_idx(self, waypoint_tree, x, y):
        _, idx = waypoint_tree.query([(x, y)], 2)
        # sort to get them in ascending order - follow along path
        idx[0].sort()
        return idx[0][0], idx[0][1]


    def publish_vehicle_command(self, stamp, steering_angle, target_velocity, left_blinker, right_blinker):
        vehicle_cmd = VehicleCmd()
        vehicle_cmd.header.stamp = stamp
        vehicle_cmd.header.frame_id =  "base_link"
        # blinkers
        vehicle_cmd.lamp_cmd.l = left_blinker
        vehicle_cmd.lamp_cmd.r = right_blinker
        # velocity and steering
        vehicle_cmd.ctrl_cmd.linear_velocity = target_velocity
        vehicle_cmd.ctrl_cmd.linear_acceleration = 0.0
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