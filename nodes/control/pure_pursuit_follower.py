#!/usr/bin/env python

import rospy
import math
import message_filters
import numpy as np
import helpers
from sklearn.neighbors import KDTree

from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose, PoseStamped,TwistStamped
from std_msgs.msg import ColorRGBA, Float32MultiArray
from autoware_msgs.msg import LaneArray, VehicleCmd, WaypointState


class PurePursuitFollower:
    def __init__(self):

        # Parameters
        self.planning_time = rospy.get_param("~planning_time", 2.0)
        self.min_lookahead_distance = rospy.get_param("~min_lookahead_distance", 6.0)
        self.wheel_base = rospy.get_param("~wheel_base", 2.789)

        # Variables - init
        self.waypoint_tree = None
        self.waypoints = None
        self.last_wp_idx = 0

        # Subscribers
        self.path_sub = rospy.Subscriber('path', LaneArray, self.path_callback)
        self.current_pose_sub = message_filters.Subscriber('current_pose', PoseStamped)
        self.current_velocity_sub = message_filters.Subscriber('current_velocity', TwistStamped)
        ts = message_filters.ApproximateTimeSynchronizer([self.current_pose_sub, self.current_velocity_sub], queue_size=10, slop=0.1)
        ts.registerCallback(self.current_status_callback)

        # Publishers
        self.pure_pursuit_markers_pub = rospy.Publisher('follower_markers', MarkerArray, queue_size=1)
        self.vehicle_command_pub = rospy.Publisher('vehicle_cmd', VehicleCmd, queue_size=1)
        self.follower_debug_pub = rospy.Publisher('follower_debug', Float32MultiArray, queue_size=1)

        # output information to console
        rospy.loginfo("pure_pursuit_follower - planning_time: " + str(self.planning_time))
        rospy.loginfo("pure_pursuit_follower - min_lookahead_distance: " + str(self.min_lookahead_distance))
        rospy.loginfo("pure_pursuit_follower - wheel_base: " + str(self.wheel_base))
        rospy.loginfo("pure_pursuit_follower - initialized")

    def path_callback(self, path_msg):
        self.waypoints = path_msg.lanes[0].waypoints
        self.last_wp_idx = len(self.waypoints) - 1

        # create kd-tree for nearest neighbor search
        waypoints_xy = np.array([(w.pose.pose.position.x, w.pose.pose.position.y) for w in self.waypoints])
        self.waypoint_tree = KDTree(waypoints_xy)


    def current_status_callback(self, current_pose_msg, current_velocity_msg):

        if self.waypoint_tree is None:
            return

        # timer start
        start_time = rospy.get_time()

        stamp = current_pose_msg.header.stamp
        current_pose = current_pose_msg.pose
        current_velocity = current_velocity_msg.twist.linear.x

        _, idx = self.waypoint_tree.query([(current_pose.position.x, current_pose.position.y)], 1)
        nearest_wp_idx = idx[0][0]
        nearest_wp = self.waypoints[nearest_wp_idx]

        # calc lookahead distance (velocity dependent)
        lookahead_distance = current_velocity * self.planning_time
        if lookahead_distance < self.min_lookahead_distance:
            lookahead_distance = self.min_lookahead_distance
        
        # TODO assume 1m distance between waypoints - currently OK, but need to make it more universal
        lookahead_wp_idx = nearest_wp_idx + int(lookahead_distance)

        if lookahead_wp_idx > self.last_wp_idx:
            lookahead_wp_idx = self.last_wp_idx
        lookahead_wp = self.waypoints[lookahead_wp_idx]

        # find current pose heading
        current_heading = helpers.get_heading_from_pose_orientation(current_pose)
        lookahead_heading = helpers.get_heading_between_two_poses(current_pose, lookahead_wp.pose.pose)
        heading_error = lookahead_heading - current_heading

        curvature = 2 * math.sin(heading_error) / lookahead_distance
        steering_angle = math.atan(self.wheel_base * curvature)

        # calc cross track error - used only for debug output
        cross_track_error = self.calc_cross_track_error(current_pose, nearest_wp_idx)

        # get blinker information from nearest waypoint and target velocity from lookahead waypoint
        left_blinker, right_blinker = helpers.get_blinker_state(nearest_wp.wpstate.steering_state)
        target_velocity = lookahead_wp.twist.twist.linear.x

        # Publish
        self.publish_vehicle_command(stamp, steering_angle, target_velocity, left_blinker, right_blinker)
        self.publish_pure_pursuit_markers(stamp, current_pose, lookahead_wp.pose.pose, heading_error)

        compute_time = rospy.get_time() - start_time
        self.follower_debug_pub.publish(Float32MultiArray(data=[compute_time, cross_track_error, heading_error]))


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


    def publish_pure_pursuit_markers(self, stamp, current_pose, lookahead_pose, heading_error):
        
        marker_array = MarkerArray()

        # draws a line between current pose and lookahead point
        marker = Marker()
        marker.header.frame_id =  "map"
        marker.header.stamp = stamp
        marker.ns = "Lookahead distance"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.color = ColorRGBA(1.0, 0.0, 1.0, 1.0)
        marker.points = ([current_pose.position, lookahead_pose.position])
        marker_array.markers.append(marker)

        # label heading_error
        average_pose = Pose()
        average_pose.position.x = (current_pose.position.x + lookahead_pose.position.x) / 2
        average_pose.position.y = (current_pose.position.y + lookahead_pose.position.y) / 2
        average_pose.position.z = (current_pose.position.z + lookahead_pose.position.z) / 2

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


    def calc_cross_track_error(self, current_pose, nearest_wp_idx):

        x_ego = current_pose.position.x
        y_ego = current_pose.position.y
        cte = 0.0

        # find nearest wp distance and id
        idx = nearest_wp_idx

        x_nearest = self.waypoints[idx].pose.pose.position.x
        y_nearest = self.waypoints[idx].pose.pose.position.y

        # calc based on backward point
        if idx > 0:
            x_back = self.waypoints[idx-1].pose.pose.position.x
            y_back = self.waypoints[idx-1].pose.pose.position.y
            cte_back = calc_dist_from_track(x_ego, y_ego, x_back, y_back, x_nearest, y_nearest)
            cte = cte_back
        
        # calc based on forward point
        if idx < self.last_wp_idx:
            x_front = self.waypoints[idx+1].pose.pose.position.x
            y_front = self.waypoints[idx+1].pose.pose.position.y
            cte_front = calc_dist_from_track(x_ego, y_ego, x_nearest, y_nearest, x_front, y_front)
            # select smaller one
            if abs(cte_front) < abs(cte):
                cte = cte_front

        return cte

    def run(self):
        rospy.spin()


def calc_dist_from_track(x_ego, y_ego, x1, y1, x2, y2):
    # calc distance from track
    # https://robotics.stackexchange.com/questions/22989/what-is-wrong-with-my-stanley-controller-for-car-steering-control

    numerator = (x2 - x1) * (y1 - y_ego) - (x1 - x_ego) * (y2 - y1)
    denominator = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return numerator / denominator


if __name__ == '__main__':
    rospy.init_node('pure_pursuit_follower', log_level=rospy.INFO)
    node = PurePursuitFollower()
    node.run()