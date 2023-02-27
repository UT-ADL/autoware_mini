#!/usr/bin/env python

import math

import rospy
import numpy as np

from ros_numpy import numpify, msgify
from sensor_msgs.msg import PointCloud2

class NaiveGroundRemovalNode:
    def __init__(self):
        self.threshold = rospy.get_param('threshold', -1.8)

        self.ground_pub = rospy.Publisher('points_ground', PointCloud2, queue_size=1)
        self.no_ground_pub = rospy.Publisher('points_no_ground', PointCloud2, queue_size=1)
        self.pointcloud_sub = rospy.Subscriber('points_raw_center', PointCloud2, self.pointcloud_callback, queue_size=1)
    
    def pointcloud_callback(self, msg):
        data = numpify(msg)
        
        # filter out point below ground level
        ground_mask = (data['z'] < self.threshold)
        ground_data = data[ground_mask]
        non_ground_data = data[~ground_mask]

        # publish ground points
        ground_msg = msgify(PointCloud2, ground_data)
        ground_msg.header.stamp = msg.header.stamp
        ground_msg.header.frame_id = msg.header.frame_id
        self.ground_pub.publish(ground_msg)

        # publish non-ground points
        non_ground_msg = msgify(PointCloud2, non_ground_data)
        non_ground_msg.header.stamp = msg.header.stamp
        non_ground_msg.header.frame_id = msg.header.frame_id
        self.no_ground_pub.publish(non_ground_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('naive_ground_removal', log_level=rospy.INFO)
    node = NaiveGroundRemovalNode()
    node.run()