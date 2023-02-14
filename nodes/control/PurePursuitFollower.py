#!/usr/bin/env python

import rospy
import tf
import math

from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose
from std_msgs.msg import ColorRGBA


class PurePursuitFollower:
    def __init__(self, waypoints, waypoint_tree):

        self.waypoints = waypoints
        self.waypoint_tree = waypoint_tree

        # Parameters
        self.planning_time = rospy.get_param("~planning_time", 2.0)
        self.min_lookahead_distance = rospy.get_param("~min_lookahead_distance", 5.5)
        self.wheel_base = rospy.get_param("~wheel_base", 2.789)

        # Variables - init
        self.lookahead_distance = 0.0
        self.last_wp_idx = len(self.waypoints) - 1

        # Publishers TODO / publish to same topic as Stanley: "follower_markers"
        self.pure_pursuit_rviz_pub = rospy.Publisher('pure_pursuit_rviz', MarkerArray, queue_size=1)


    def calc_steering_and_velocity(self, current_pose, current_velocity):

        self.nearest_wp_distance, self.nearest_wp_idx = self.waypoint_tree.query([[current_pose.position.x, current_pose.position.y]], 1)

        # calc lookahead distance (velocity dependent)
        self.lookahead_distance = current_velocity * self.planning_time
        if self.lookahead_distance < self.min_lookahead_distance:
            self.lookahead_distance = self.min_lookahead_distance
        
        # TODO assume 1m distance between waypoints - currently OK, but need to make it more universal (add 5 - point in front of the car)
        lookahead_wp_idx = self.nearest_wp_idx + math.floor(self.lookahead_distance)

        if lookahead_wp_idx > self.last_wp_idx:
            lookahead_wp_idx = self.last_wp_idx
        lookahead_wp = self.waypoints[int(lookahead_wp_idx)]

        self.target_velocity = lookahead_wp.twist.twist.linear.x

        # calculate heading from current pose
        quaternion = (current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w)
        _, _, self.current_heading = tf.transformations.euler_from_quaternion(quaternion)

        self.lookahead_heading = get_heading_from_two_positions(current_pose.position, lookahead_wp.pose.pose.position)
        alpha = self.lookahead_heading - self.current_heading

        curvature = 2 * math.sin(alpha) / self.lookahead_distance
        self.steering_angle = math.atan(self.wheel_base * curvature)

        self.publish_pure_pursuit_rviz(current_pose, lookahead_wp.pose.pose, alpha)
        # publish also debug output to another topic (e.g. /waypoint_follower/debug) - lateral error

        return self.steering_angle, self.target_velocity


    def publish_pure_pursuit_rviz(self, current_pose, lookahead_pose, alpha):
        
        marker_array = MarkerArray()

        # draws a line between current pose and lookahead point
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "Lookahead distance"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
        marker.points = ([current_pose.position, lookahead_pose.position])
        marker_array.markers.append(marker)

        # label of angle alpha
        average_pose = Pose()
        average_pose.position.x = (current_pose.position.x + lookahead_pose.position.x) / 2
        average_pose.position.y = (current_pose.position.y + lookahead_pose.position.y) / 2
        average_pose.position.z = (current_pose.position.z + lookahead_pose.position.z) / 2

        marker_text = Marker()
        marker_text.header.frame_id = "/map"
        marker_text.header.stamp = rospy.Time.now()
        marker_text.ns = "Angle alpha"
        marker_text.id = 1
        marker_text.type = Marker.TEXT_VIEW_FACING
        marker_text.action = Marker.ADD
        marker_text.pose = average_pose
        marker_text.scale.z = 0.6
        marker_text.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
        marker_text.text = str(round(math.degrees(alpha),1))
        marker_array.markers.append(marker_text)

        self.pure_pursuit_rviz_pub.publish(marker_array)


def get_heading_from_two_positions(position1, position2):
    # calc heading from two poses
    heading = math.atan2(position2.y - position1.y, position2.x - position1.x)

    return heading
