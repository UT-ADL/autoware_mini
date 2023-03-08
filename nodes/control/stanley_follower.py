#!/usr/bin/env python

import rospy
import math
import message_filters
import numpy as np
from sklearn.neighbors import KDTree

from helpers import get_heading_from_pose_orientation, get_blinker_state, get_heading_between_two_poses, \
    get_intersection_point, get_point_on_path_within_distance, get_cross_track_error, \
    get_pose_using_heading_and_distance, get_relative_heading_error
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose, PoseStamped,TwistStamped
from std_msgs.msg import ColorRGBA
from autoware_msgs.msg import Lane, VehicleCmd


class StanleyFollower:
    def __init__(self):

         # Parameters
        self.wheel_base = rospy.get_param("~wheel_base", 2.789)
        self.cte_gain = rospy.get_param("~cte_gain", 0.3)       # gain for cross track error
        self.heading_angle_limit = rospy.get_param("~heading_angle_limit", 90.0)
        self.lateral_error_limit = rospy.get_param("~lateral_error_limit", 2.0)

        # Variables - init
        self.waypoint_tree = None
        self.waypoints = None
        self.last_wp_idx = 0

        # Subscribers
        self.path_sub = rospy.Subscriber('path', Lane, self.path_callback)
        self.current_pose_sub = message_filters.Subscriber('current_pose', PoseStamped)
        self.current_velocity_sub = message_filters.Subscriber('current_velocity', TwistStamped)
        ts = message_filters.ApproximateTimeSynchronizer([self.current_pose_sub, self.current_velocity_sub], queue_size=10, slop=0.1)
        ts.registerCallback(self.current_status_callback)

        # Publishers
        self.stanley_markers_pub = rospy.Publisher('follower_markers', MarkerArray, queue_size=1)
        self.vehicle_command_pub = rospy.Publisher('vehicle_cmd', VehicleCmd, queue_size=1)

        # output information to console
        rospy.loginfo("stanley_follower - wheel_base: " + str(self.wheel_base))
        rospy.loginfo("stanley_follower - cte_gain: " + str(self.cte_gain))
        rospy.loginfo("stanley_follower - initialized")


    def path_callback(self, path_msg):

        if len(path_msg.waypoints) == 0:
            rospy.logwarn("stanley_follower - no waypoints received")
            return
        else:
            self.waypoints = path_msg.waypoints
            self.last_wp_idx = len(self.waypoints) - 1
            # create kd-tree for nearest neighbor search
            waypoints_xy = np.array([(w.pose.pose.position.x, w.pose.pose.position.y) for w in self.waypoints])
            self.waypoint_tree = KDTree(waypoints_xy)


    def current_status_callback(self, current_pose_msg, current_velocity_msg):

        if self.waypoint_tree is None:
            return

        stamp = current_pose_msg.header.stamp
        current_pose = current_pose_msg.pose
        current_velocity = current_velocity_msg.twist.linear.x
        current_heading = get_heading_from_pose_orientation(current_pose)
        
        # Find pose for the front wheel
        front_wheel_pose = get_pose_using_heading_and_distance(current_pose, current_heading, self.wheel_base)
        
        # find two closest points on track to calculate cross track error
        _, idx = self.waypoint_tree.query([(front_wheel_pose.position.x, front_wheel_pose.position.y)], 2)
        sorted = np.sort(idx[0])
        waypoint1 = self.waypoints[sorted[0]]
        waypoint2 = self.waypoints[sorted[1]]

        if sorted[1] == self.last_wp_idx:
            # stop vehicle if last waypoint is reached
            self.publish_vehicle_command(stamp, 0.0, 0.0, 0, 0)
            rospy.logwarn_throttle(10, "stanley_follower - last waypoint reached")
            return
        
        # will give error with +/- depending on the side of the track
        cross_track_error = get_cross_track_error(front_wheel_pose, waypoint1.pose.pose, waypoint2.pose.pose)

        # get closest point to base_link on path (line defined by 2 closest waypoints)
        _, idx = self.waypoint_tree.query([(current_pose.position.x, current_pose.position.y)], 2)
        sorted = np.sort(idx[0])
        waypoint1 = self.waypoints[sorted[0]]
        waypoint2 = self.waypoints[sorted[1]]
        nearest_pose = get_intersection_point(current_pose, waypoint1.pose.pose, waypoint2.pose.pose)
        # find lookahead pose on path within distance (used for track heading) and take also target_velocity from closest waypoint
        lookahead_pose = get_point_on_path_within_distance(self.waypoints, self.last_wp_idx, sorted[1], nearest_pose, self.wheel_base)

        track_heading = get_heading_between_two_poses(nearest_pose, lookahead_pose)
        heading_error = get_relative_heading_error(track_heading, current_heading)

        if abs(cross_track_error) > self.lateral_error_limit or abs(math.degrees(heading_error)) > self.heading_angle_limit:
            # stop vehicle if cross track error or heading angle difference is over limit
            self.publish_vehicle_command(stamp, 0.0, 0.0, 0, 0)
            rospy.logerr_throttle(10, "stanley_follower - lateral error or heading angle difference over limit")
            return

        # calculate steering angle
        delta_error = math.atan(self.cte_gain * cross_track_error / (current_velocity + 0.0001))
        steering_angle = heading_error + delta_error

        # get blinker information and target velocity
        _, idx = self.waypoint_tree.query([(lookahead_pose.position.x, lookahead_pose.position.y)], 1)
        target_velocity = self.waypoints[idx[0][0]].twist.twist.linear.x
        left_blinker, right_blinker  = get_blinker_state(waypoint2.wpstate.steering_state)

        # Publish
        self.publish_vehicle_command(stamp, steering_angle, target_velocity, left_blinker, right_blinker)
        self.publish_stanley_markers(stamp, nearest_pose, lookahead_pose, front_wheel_pose, heading_error)


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


    def publish_stanley_markers(self, stamp, nearest_pose, lookahead_pose, front_pose, heading_error):
        
        marker_array = MarkerArray()

        # draws a line between current pose and nearest_wp point
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = stamp
        marker.ns = "nearest_wp distance"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.color = ColorRGBA(1.0, 0.0, 1.0, 1.0)
        marker.points = ([nearest_pose.position, lookahead_pose.position, front_pose.position])
        marker_array.markers.append(marker)

        # label of angle alpha
        average_pose = Pose()
        average_pose.position.x = (front_pose.position.x + nearest_pose.position.x) / 2
        average_pose.position.y = (front_pose.position.y + nearest_pose.position.y) / 2
        average_pose.position.z = (front_pose.position.z + nearest_pose.position.z) / 2

        marker_text = Marker()
        marker_text.header.frame_id = "map"
        marker_text.header.stamp = stamp
        marker_text.ns = "heading_error"
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