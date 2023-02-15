#!/usr/bin/env python

import rospy
import tf
import math
import rospy
import message_filters
import numpy as np
from sklearn.neighbors import KDTree

from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose, PoseStamped,TwistStamped
from std_msgs.msg import ColorRGBA
from autoware_msgs.msg import Lane, VehicleCmd


class PurePursuitFollower:
    def __init__(self):

        # Parameters
        self.planning_time = rospy.get_param("~planning_time", 2.0)
        self.min_lookahead_distance = rospy.get_param("~min_lookahead_distance", 5.5)
        self.wheel_base = rospy.get_param("~wheel_base", 2.789)

        # Variables - init
        self.lookahead_distance = 0.0
        self.current_velocity = 0.0
        self.waypoint_tree = None
        self.waypoints = None
        self.last_wp_idx = 0
        self.target_velocity = 0.0


        # Subscribers
        self.waypoints_sub = rospy.Subscriber('/path', Lane, self.waypoints_callback)
        self.current_pose_sub = message_filters.Subscriber('/current_pose', PoseStamped)
        self.current_velocity_sub = message_filters.Subscriber('/current_velocity', TwistStamped)
        ts = message_filters.TimeSynchronizer([self.current_pose_sub, self.current_velocity_sub], queue_size=10)
        ts.registerCallback(self.current_status_callback)

        # Publishers TODO / publish to same topic as Stanley: "follower_markers"
        self.pure_pursuit_rviz_pub = rospy.Publisher('follower_markers', MarkerArray, queue_size=1)
        self.vehicle_command_pub = rospy.Publisher('vehicle_cmd', VehicleCmd, queue_size=10)

        # output information to console
        rospy.loginfo("pure_pursuit_follower - initiliazed")

    def waypoints_callback(self, waypoints_msg):
        self.waypoints = waypoints_msg.waypoints
        self.last_wp_idx = len(self.waypoints) - 1

        # create kd-tree for nearest neighbor search
        waypoints_xy = np.array([[w.pose.pose.position.x, w.pose.pose.position.y] for w in self.waypoints])
        self.waypoint_tree = KDTree(waypoints_xy)


    def current_status_callback(self, current_pose_msg, current_velocity_msg):

        if self.waypoint_tree is None:
            return

        self.current_pose = current_pose_msg.pose
        self.current_velocity = current_velocity_msg.twist.linear.x

        self.nearest_wp_distance, self.nearest_wp_idx = self.waypoint_tree.query([[self.current_pose.position.x, self.current_pose.position.y]], 1)

        # calc lookahead distance (velocity dependent)
        self.lookahead_distance = self.current_velocity * self.planning_time
        if self.lookahead_distance < self.min_lookahead_distance:
            self.lookahead_distance = self.min_lookahead_distance
        
        # TODO assume 1m distance between waypoints - currently OK, but need to make it more universal
        lookahead_wp_idx = self.nearest_wp_idx + math.floor(self.lookahead_distance)

        if lookahead_wp_idx > self.last_wp_idx:
            lookahead_wp_idx = self.last_wp_idx
        lookahead_wp = self.waypoints[int(lookahead_wp_idx)]

        # set target velocity taken from lookahead point
        self.target_velocity = lookahead_wp.twist.twist.linear.x

        # find current pose heading
        quaternion = (self.current_pose.orientation.x, self.current_pose.orientation.y, self.current_pose.orientation.z, self.current_pose.orientation.w)
        _, _, self.current_heading = tf.transformations.euler_from_quaternion(quaternion)
        self.lookahead_heading = get_heading_from_two_positions(self.current_pose.position, lookahead_wp.pose.pose.position)
        alpha = self.lookahead_heading - self.current_heading

        curvature = 2 * math.sin(alpha) / self.lookahead_distance
        self.steering_angle = math.atan(self.wheel_base * curvature)

        self.publish_pure_pursuit_rviz(self.current_pose, lookahead_wp.pose.pose, alpha)
        # publish also debug output to another topic (e.g. /waypoint_follower/debug) - lateral error
        self.publish_vehicle_command()


    def publish_vehicle_command(self):
        vehicle_cmd = VehicleCmd()
        vehicle_cmd.header.stamp = rospy.Time.now()
        vehicle_cmd.header.frame_id = "/map"
        vehicle_cmd.ctrl_cmd.linear_velocity = self.target_velocity
        # TODO
        vehicle_cmd.ctrl_cmd.linear_acceleration = 0.0
        vehicle_cmd.ctrl_cmd.steering_angle = self.steering_angle
        self.vehicle_command_pub.publish(vehicle_cmd)


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

    def run(self):
        rospy.spin()


def get_heading_from_two_positions(position1, position2):
    # calc heading from two positions
    heading = math.atan2(position2.y - position1.y, position2.x - position1.x)

    return heading


if __name__ == '__main__':
    rospy.init_node('pure_pursuit_follower', log_level=rospy.INFO)
    node = PurePursuitFollower()
    node.run()