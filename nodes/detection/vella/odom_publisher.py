#!/usr/bin/env python3

from __future__ import print_function
from __future__ import division

import rospy
import message_filters
from geometry_msgs.msg import PoseStamped,TwistStamped
from nav_msgs.msg import Odometry

class OdomPub:
    def __init__(self):
        rospy.loginfo(self.__class__.__name__ + " - Initializing")

        # subscribers
        ego_speed_sub = message_filters.Subscriber('/current_velocity', TwistStamped)
        ego_pose_sub = message_filters.Subscriber('/current_pose', PoseStamped)

        # odometry publisher for vella
        self.odometry_pub = rospy.Publisher('/odom', Odometry, queue_size=1)

        # Sync - get both topics in a synchronized callback
        ts = message_filters.ApproximateTimeSynchronizer([ego_speed_sub, ego_pose_sub], queue_size=3, slop=0.05)
        ts.registerCallback(self.odom_callback)
    def odom_callback(self, ego_velocity, ego_pose):

        odom_msg = Odometry()
        odom_msg.header.stamp = ego_velocity.header.stamp
        odom_msg.header.frame_id = 'vehicle'
        odom_msg.child_frame_id = 'vehicle'
        odom_msg.pose.pose = ego_pose.pose
        odom_msg.twist.twist = ego_velocity.twist

        self.odometry_pub.publish(odom_msg)
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('odom_publisher', anonymous=True, log_level=rospy.INFO)
    node = OdomPub()
    node.run()
