#!/usr/bin/env python

import rospy
import tf
import math
import message_filters
import numpy as np
from sklearn.neighbors import KDTree

from autoware_msgs.msg import Lane
from geometry_msgs.msg import PoseStamped,TwistStamped
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA

class WaypointFollower:
    def __init__(self):

        # Parameters
        self.waypoint_topic = rospy.get_param("~waypoint_topic", "/waypoints")
        self.planning_time = rospy.get_param("~planning_time", 2.0)
        self.wheel_base = rospy.get_param("~wheel_base", 2.789)

        # Variables - init
        self.nearest_wp_distance = 0.0
        self.nearest_wp_idx = 0
        self.last_wp_idx = 0
        self.current_velocity = 0.0
        self.waypoint_tree = None
        self.waypoints = None
        self.target_velocity = 0.0

        # Subscribers
        self.waypoints_sub = rospy.Subscriber('/waypoints', Lane, self.waypoints_callback)
        self.current_pose_sub = message_filters.Subscriber('/current_pose', PoseStamped)
        self.current_velocity_sub = message_filters.Subscriber('/current_velocity', TwistStamped)
        ts = message_filters.TimeSynchronizer([self.current_pose_sub, self.current_velocity_sub], queue_size=10)
        ts.registerCallback(self.current_status_callback)

        # Publishers
        self.pure_pursuit_rviz_pub = rospy.Publisher('/pure_pursuit_rviz', MarkerArray, queue_size=10)


        # init also PurePursuit
        # is there any data that could be sent to PurePursuit?

        # output information to console
        rospy.loginfo("waypoint_follower - initiliazed")


    def waypoints_callback(self, waypoints_msg):
        self.waypoints = waypoints_msg.waypoints
        self.last_wp_idx = len(self.waypoints) - 1
        print("DEBUG - loaded %i waypoints" % len(self.waypoints))

        # create kd-tree for nearest neighbor search
        waypoints_xy = np.array([[w.pose.pose.position.x, w.pose.pose.position.y] for w in self.waypoints])
        self.waypoint_tree = KDTree(waypoints_xy)


    def current_status_callback(self, current_pose_msg, current_velocity_msg):

        current_pose = current_pose_msg.pose
        self.current_velocity = current_velocity_msg.twist.linear.x

        # get nearest waypoint distance and idx - make sure waypoints are loaded
        # or create / init this whole callback after waypoints are loaded - to skip the check here! 
        if self.waypoint_tree is not None:
            self.nearest_wp_distance, self.nearest_wp_idx = self.waypoint_tree.query([[current_pose.position.x, current_pose.position.y]], 1)
            print("DEBUG - nearest waypoint: d, idx: %f, %i" % (self.nearest_wp_distance, self.nearest_wp_idx))

            # calc lookahead distance (velocity dependent), get point idx and extract coordinates
            self.lookahead_distance = self.current_velocity * self.planning_time
            print("DEBUG - lookahead distance: %f" % self.lookahead_distance)
            # TODO assume 1m distance between waypoints - currently OK, but need make it more universal (add 5 - point in front of the car)
            lookahead_wp_idx = self.nearest_wp_idx + math.floor(self.lookahead_distance + 5)
            
            if lookahead_wp_idx > self.last_wp_idx:
                lookahead_wp_idx = self.last_wp_idx

            print("DEBUG - lookahead waypoint idx: %i" % lookahead_wp_idx)
            lookahead_wp = self.waypoints[int(lookahead_wp_idx)]

            # calculate heading from current pose
            self.current_heading = get_heading_from_orientation(current_pose.orientation)
            self.lookahead_heading = get_heading_from_two_poses(current_pose.position, lookahead_wp.pose.pose.position)

            alpha = self.lookahead_heading - self.current_heading
            print("DEBUG - alpha: %f" % (alpha))

            # calc curvature (lookahead distance, current pose, lookahead point)
            self.steer_output = math.atan((2 * self.wheel_base * math.sin(alpha)) / self.lookahead_distance)
            print("DEBUG - steer output: %f" % (self.steer_output))

            # limit steer output to -1.0 to 1.0?

            # publish lookahead distance (and arc) to rviz
            self.publish_pure_pursuit_rviz(current_pose, lookahead_wp.pose.pose, alpha)

            # publish also debug output to another topic (e.g. /waypoint_follower/debug) - lateral error
            # publish control commands (will be published 50Hz as current_pose and current_velocity are published 50Hz)
            


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
        marker_text = Marker()
        marker_text.header.frame_id = "/map"
        marker_text.header.stamp = rospy.Time.now()
        marker_text.ns = "Angle alpha"
        marker_text.id = 1
        marker_text.type = Marker.TEXT_VIEW_FACING
        marker_text.action = Marker.ADD
        marker_text.pose = lookahead_pose
        marker_text.scale.z = 2.0
        marker_text.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
        marker_text.text = str(round(alpha,3))
        marker_array.markers.append(marker_text)

        self.pure_pursuit_rviz_pub.publish(marker_array)



    def run(self):
        rospy.spin()

def get_heading_from_two_poses(pose1, pose2):
    # TODO check if this is correct - atan2. Empirical seemed ok.
    heading = math.atan2(pose2.y - pose1.y, pose2.x - pose1.x)

    heading = convert_heading_to_360(math.degrees(heading))

    return heading


def get_heading_from_orientation(orientation):
    # convert quaternion to euler angles
    quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = math.degrees(euler[2])  # from x axis: ccw up to 180, cw down to -180 degrees

    heading = convert_heading_to_360(yaw)

    return heading


def convert_heading_to_360(yaw):
    # convert yaw to heading from y axis (north) and cw from 0 up to 360
    if yaw < 0:
        heading = abs(yaw) + 90
    else:
        heading = 90 - yaw
        if heading < 0:
            heading += 360

    return heading


# class PurePursuit:
#     def __init__(self, velocity, lateral_error):

#         # Parameters
#         self.planning_time = rospy.get_param("~planning_time", 2.0)

#         # Variables - init
#         self.lookahead_distance = velocity * self.planning_time

        



if __name__ == '__main__':
    rospy.init_node('waypoint_follower', log_level=rospy.INFO)
    node = WaypointFollower()
    node.run()
