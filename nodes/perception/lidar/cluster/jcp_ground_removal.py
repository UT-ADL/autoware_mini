#!/usr/bin/env python3

import rospy
import numpy as np

from numpy.lib.recfunctions import structured_to_unstructured
from ros_numpy import numpify, msgify
from sensor_msgs.msg import PointCloud2

from autoware_mini.jcp import JPCGroundRemove

class JCPGroundRemovalNode:
    def __init__(self):
        self.range_img_width = rospy.get_param('~range_img_width')
        self.range_img_height = rospy.get_param('~range_img_height')
        self.sensor_height = rospy.get_param('~sensor_height')
        self.delta_R = rospy.get_param('~delta_R')

        self.ground_pub = rospy.Publisher('points_ground', PointCloud2, queue_size=1, tcp_nodelay=True)
        self.no_ground_pub = rospy.Publisher('points_no_ground', PointCloud2, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('points_raw', PointCloud2, self.pointcloud_callback, queue_size=1, buff_size=2**24, tcp_nodelay=True)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def pointcloud_callback(self, msg):
        data = numpify(msg)
        
        # convert point cloud into ndarray, take only xyz coordinates
        points = structured_to_unstructured(data[['x', 'y', 'z']], dtype=np.float32)

        # filter ground points
        ground_mask = JPCGroundRemove(
            pcl=points,
            range_img_width=self.range_img_width,
            range_img_height=self.range_img_height,
            sensor_height=self.sensor_height,
            delta_R=self.delta_R)

        # publish non-ground points
        non_ground_data = data[~ground_mask]
        non_ground_msg = msgify(PointCloud2, non_ground_data)
        non_ground_msg.header.stamp = msg.header.stamp
        non_ground_msg.header.frame_id = msg.header.frame_id
        self.no_ground_pub.publish(non_ground_msg)

        # publish ground points
        ground_data = data[ground_mask]
        ground_msg = msgify(PointCloud2, ground_data)
        ground_msg.header.stamp = msg.header.stamp
        ground_msg.header.frame_id = msg.header.frame_id
        self.ground_pub.publish(ground_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('jcp_ground_removal', log_level=rospy.INFO)
    node = JCPGroundRemovalNode()
    node.run()