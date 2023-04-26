#!/usr/bin/env python3

import rospy
import math
import message_filters
import threading
import numpy as np
from sklearn.neighbors import NearestNeighbors

from helpers import get_heading_from_pose_orientation, get_blinker_state, get_heading_between_two_points, \
    get_closest_point_on_line, get_point_on_path_within_distance, get_cross_track_error, \
    get_pose_using_heading_and_distance, normalize_heading_error, interpolate_velocity_between_waypoints, \
    get_two_nearest_waypoint_idx
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose, PoseStamped, TwistStamped
from std_msgs.msg import ColorRGBA, Float32MultiArray
from autoware_msgs.msg import Lane, VehicleCmd


class StanleyFollower:
    def __init__(self):

         # Parameters
        self.cte_gain = rospy.get_param("~cte_gain", 0.3)       # gain for cross track error
        self.wheel_base = rospy.get_param("/vehicle/wheel_base", 2.789)
        self.heading_angle_limit = rospy.get_param("heading_angle_limit", 90.0)
        self.lateral_error_limit = rospy.get_param("lateral_error_limit", 2.0)
        self.publish_debug_info = rospy.get_param("~publish_debug_info", False)
        self.nearest_neighbor_search = rospy.get_param("~nearest_neighbor_search", "kd_tree")

        # Variables - init
        self.waypoint_tree = None
        self.waypoints = None
        self.lock = threading.Lock()

        # Publishers
        self.vehicle_command_pub = rospy.Publisher('vehicle_cmd', VehicleCmd, queue_size=1)
        if self.publish_debug_info:
            self.stanley_markers_pub = rospy.Publisher('follower_markers', MarkerArray, queue_size=1)
            self.follower_debug_pub = rospy.Publisher('follower_debug', Float32MultiArray, queue_size=1)

        # Subscribers
        rospy.Subscriber('/planning/local_path', Lane, self.path_callback)
        current_pose_sub = message_filters.Subscriber('/localization/current_pose', PoseStamped)
        current_velocity_sub = message_filters.Subscriber('/localization/current_velocity', TwistStamped)
        ts = message_filters.ApproximateTimeSynchronizer([current_pose_sub, current_velocity_sub], queue_size=2, slop=0.02)
        ts.registerCallback(self.current_status_callback)

        # output information to console
        rospy.loginfo("stanley_follower - initialized")


    def path_callback(self, path_msg):

        if len(path_msg.waypoints) < 2:
            # if path is cancelled and empty waypoints received
            rospy.logwarn_throttle(30, "stanley_follower - no waypoints, stopping!")
            with self.lock:
                self.waypoint_tree = None
                self.waypoints = None
            return

        # prepare waypoints for nearest neighbor search
        waypoints_xy = np.array([(w.pose.pose.position.x, w.pose.pose.position.y) for w in path_msg.waypoints])
        waypoint_tree = NearestNeighbors(n_neighbors=1, algorithm=self.nearest_neighbor_search).fit(waypoints_xy)
        with self.lock:
            self.waypoint_tree = waypoint_tree
            self.waypoints = path_msg.waypoints


    def current_status_callback(self, current_pose_msg, current_velocity_msg):

        if self.publish_debug_info:
            start_time = rospy.get_time()

        with self.lock:
            waypoints = self.waypoints
            waypoint_tree = self.waypoint_tree
    
        stamp = current_pose_msg.header.stamp
        current_pose = current_pose_msg.pose
        current_velocity = current_velocity_msg.twist.linear.x
        current_heading = get_heading_from_pose_orientation(current_pose)

        if waypoint_tree is None:
            # if no waypoints received yet or global_path cancelled, stop the vehicle
            self.publish_vehicle_command(stamp, 0.0, 0.0, 0, 0)
            return

        # Find pose for the front wheel and 2 closest waypoint idx (fw_)
        front_wheel_pose = get_pose_using_heading_and_distance(current_pose, current_heading, self.wheel_base)
        fw_back_wp_idx, fw_front_wp_idx = get_two_nearest_waypoint_idx(waypoint_tree, front_wheel_pose.position.x, front_wheel_pose.position.y)
        cross_track_error = get_cross_track_error(front_wheel_pose, waypoints[fw_back_wp_idx].pose.pose, waypoints[fw_front_wp_idx].pose.pose)

        # get closest point to base_link on path (line defined by 2 closest waypoints) - (bl_)
        bl_back_wp_idx, bl_front_wp_idx = get_two_nearest_waypoint_idx(waypoint_tree, current_pose.position.x, current_pose.position.y)

        if bl_front_wp_idx == len(waypoints)-1:
            # stop vehicle if last waypoint is reached
            self.publish_vehicle_command(stamp, 0.0, 0.0, 0, 0)
            rospy.logwarn_throttle(10, "stanley_follower - last waypoint reached")
            return
    
        bl_nearest_point = get_closest_point_on_line(current_pose.position, waypoints[bl_back_wp_idx].pose.pose.position, waypoints[bl_front_wp_idx].pose.pose.position)
        lookahead_point = get_point_on_path_within_distance(waypoints, bl_front_wp_idx, bl_nearest_point, self.wheel_base)

        track_heading = get_heading_between_two_points(bl_nearest_point, lookahead_point)
        heading_error = normalize_heading_error(track_heading - current_heading)

        if abs(cross_track_error) > self.lateral_error_limit or abs(math.degrees(heading_error)) > self.heading_angle_limit:
            # stop vehicle if cross track error or heading angle difference is over limit
            self.publish_vehicle_command(stamp, 0.0, 0.0, 0, 0)
            rospy.logerr_throttle(10, "stanley_follower - lateral error or heading angle difference over limit")
            return

        # calculate steering angle
        delta_error = math.atan(self.cte_gain * cross_track_error / (current_velocity + 0.0001))
        steering_angle = heading_error + delta_error

        # target_velocity from map and based on closest object
        target_velocity = interpolate_velocity_between_waypoints(bl_nearest_point, waypoints[bl_back_wp_idx], waypoints[bl_front_wp_idx])

        # blinkers
        left_blinker, right_blinker  = get_blinker_state(waypoints[bl_front_wp_idx].wpstate.steering_state)

        # Publish
        self.publish_vehicle_command(stamp, steering_angle, target_velocity, left_blinker, right_blinker)
        if self.publish_debug_info:
            self.publish_stanley_markers(stamp, bl_nearest_point, lookahead_point, front_wheel_pose, heading_error)
            self.follower_debug_pub.publish(Float32MultiArray(data=[(rospy.get_time() - start_time), current_heading, track_heading, heading_error, cross_track_error, target_velocity]))


    def publish_vehicle_command(self, stamp, steering_angle, target_velocity, left_blinker, right_blinker):
        vehicle_cmd = VehicleCmd()
        vehicle_cmd.header.stamp = stamp
        vehicle_cmd.header.frame_id = "base_link"
        # blinkers
        vehicle_cmd.lamp_cmd.l = left_blinker 
        vehicle_cmd.lamp_cmd.r = right_blinker 
        # velocity and steering
        vehicle_cmd.ctrl_cmd.linear_velocity = target_velocity
        vehicle_cmd.ctrl_cmd.linear_acceleration = 0.0
        vehicle_cmd.ctrl_cmd.steering_angle = steering_angle
        self.vehicle_command_pub.publish(vehicle_cmd)


    def publish_stanley_markers(self, stamp, nearest_point, lookahead_point, front_pose, heading_error):

        marker_array = MarkerArray()

        # draws a line between current pose and nearest_wp point
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = stamp
        marker.ns = "Line"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.color = ColorRGBA(1.0, 0.0, 1.0, 1.0)
        marker.points = ([nearest_point, lookahead_point, front_pose.position])
        marker_array.markers.append(marker)

        # label of angle alpha
        average_pose = Pose()
        average_pose.position.x = (front_pose.position.x + nearest_point.x) / 2
        average_pose.position.y = (front_pose.position.y + nearest_point.y) / 2
        average_pose.position.z = (front_pose.position.z + nearest_point.z) / 2

        marker_text = Marker()
        marker_text.header.frame_id = "map"
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

        self.stanley_markers_pub.publish(marker_array)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('stanley_follower', log_level=rospy.INFO)
    node = StanleyFollower()
    node.run()