#!/usr/bin/env python

import rospy
import message_filters
import numpy as np
from sklearn.neighbors import KDTree

from autoware_msgs.msg import Lane
from geometry_msgs.msg import PoseStamped,TwistStamped

class WaypointFollower:
    def __init__(self):

        # Parameters
        self.waypoint_topic = rospy.get_param("~waypoint_topic", "/waypoints")

        # Variables - init
        self.nearest_wp_distance = 0.0
        self.nearest_wp_idx = 0
        self.last_wp_idx = 0
        self.current_velocity = 0.0
        self.waypoint_tree = None

        # Subscribers
        self.waypoints_sub = rospy.Subscriber('/waypoints', Lane, self.waypoints_callback)
        self.current_pose_sub = message_filters.Subscriber('/current_pose', PoseStamped)
        self.current_velocity_sub = message_filters.Subscriber('/current_velocity', TwistStamped)
        ts = message_filters.TimeSynchronizer([self.current_pose_sub, self.current_velocity_sub], queue_size=10)
        ts.registerCallback(self.current_status_callback)

        # Publishers


        # init also PurePursuit
        # is there any data that could be sent to PurePursuit?

        # output information to console
        rospy.loginfo("waypoint_follower - initiliazed")


    def waypoints_callback(self, waypoints_msg):
        waypoints = waypoints_msg.waypoints
        self.last_wp_idx = len(waypoints) - 1
        print("DEBUG - loaded %i waypoints" % len(waypoints))

        # create kd-tree for nearest neighbor search
        waypoints_xy = [[w.pose.pose.position.x, w.pose.pose.position.y] for w in waypoints]
        waypoints_xy = np.array(waypoints_xy)
        print("DEBUG - waypoints shape: %s" % str(waypoints_xy.shape))
        self.waypoint_tree = KDTree(waypoints_xy)


    def current_status_callback(self, current_pose_msg, current_velocity_msg):

        current_pose = current_pose_msg.pose
        self.current_velocity = current_velocity_msg.twist.linear.x

        # get nearest waypoint distance and idx - make sure waypoints are loaded
        # or create / init this whole callback after waypoints are loaded - to skip the check here! 
        if self.waypoint_tree is not None:
            self.nearest_wp_distance, self.nearest_wp_idx = self.waypoint_tree.query([[current_pose.position.x, current_pose.position.y]], 1)
            print("DEBUG - nearest waypoint: d, idx: %f, %i" % (self.nearest_wp_distance, self.nearest_wp_idx))

        # Might need to calc additional stuff here (lookahead distance, ... ) or send raw data from messages to PurePursuit and calc there

        """
        1. calc lookahead distance (velocity dependent), get point idx and extract coordinates
        2. calc curvature (lookahead distance, current pose, lookahead point)
        3. calc steering angle (curvature, wheelbase)
        4. calc throttle (velocity, velocity setpoint)
        5. publish control commands
        """


        # call PurePursuit.solver or smth like that

        # publish control commands (will be published 50Hz as current_pose and current_velocity are published 50Hz)
        # publish also debug output to another topic (e.g. /waypoint_follower/debug) - lateral error


# class PurePursuit:
#     def __init__(self):



    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('waypoint_follower', log_level=rospy.INFO)
    node = WaypointFollower()
    node.run()
