#!/usr/bin/env python3

import math

import rospy
import numpy as np
import cv2

from ros_numpy import numpify, msgify
from sensor_msgs.msg import PointCloud2

class GroundRemovalNode:
    def __init__(self):
        self.min_x = rospy.get_param('~min_x')
        self.max_x = rospy.get_param('~max_x')
        self.min_y = rospy.get_param('~min_y')
        self.max_y = rospy.get_param('~max_y')
        self.min_z = rospy.get_param('~min_z')
        self.max_z = rospy.get_param('~max_z')
        self.cell_size = rospy.get_param('~cell_size')
        self.tolerance = rospy.get_param('~tolerance')
        self.filter = rospy.get_param('~filter')
        self.filter_size = rospy.get_param('~filter_size')

        self.width = int(math.ceil((self.max_x - self.min_x) / self.cell_size))
        self.height = int(math.ceil((self.max_y - self.min_y) / self.cell_size))
        self.cols = np.empty((self.width, self.height), dtype=np.float32)

        self.ground_pub = rospy.Publisher('points_ground', PointCloud2, queue_size=1, tcp_nodelay=True)
        self.no_ground_pub = rospy.Publisher('points_no_ground', PointCloud2, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('points_raw', PointCloud2, self.pointcloud_callback, queue_size=1, buff_size=2**24, tcp_nodelay=True)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def pointcloud_callback(self, msg):
        data = numpify(msg)
        
        # filter out of range points
        filter = (self.min_x <= data['x']) & (data['x'] < self.max_x) \
               & (self.min_y <= data['y']) & (data['y'] < self.max_y) \
               & (self.min_z <= data['z']) & (data['z'] < self.max_z)
        data_filtered = data[filter]

        # convert x and y coordinates into indexes
        xi = ((data_filtered['x'] - self.min_x) / self.cell_size).astype(np.int32)
        yi = ((data_filtered['y'] - self.min_y) / self.cell_size).astype(np.int32)
        zi = data_filtered['z']

        # write minimum height for each cell to cols
        # thanks to sorting in descending order,
        # the minimum value will overwrite previous values
        self.cols.fill(np.nan)
        idx = np.argsort(-zi)
        self.cols[xi[idx], yi[idx]] = zi[idx]

        # bring cell minimum lower, if all cells around it are lower
        if self.filter == 'median':
            cols_filtered = cv2.medianBlur(self.cols, self.filter_size)
            np.fmin(self.cols, cols_filtered, out=self.cols)
        elif self.filter == 'average':
            mask = np.isnan(self.cols)
            self.cols[mask] = 0
            cols_filtered = cv2.blur(self.cols, (self.filter_size, self.filter_size), cv2.BORDER_REPLICATE) / \
                    cv2.blur((~mask).astype(np.float32), (self.filter_size, self.filter_size), cv2.BORDER_REPLICATE)
            np.fmin(self.cols, cols_filtered, out=self.cols)
        elif self.filter == 'minimum':
            mask = np.isnan(self.cols)
            self.cols[mask] = np.inf
            cols_filtered = cv2.erode(self.cols, np.ones((self.filter_size, self.filter_size)), cv2.BORDER_REPLICATE)
            np.fmin(self.cols, cols_filtered, out=self.cols)
        elif self.filter != 'none':
            assert False, "Unknown filter value: " + self.filter

        # filter out closest points to minimum point up to some tolerance
        ground_mask = (zi <= (self.cols[xi, yi] + self.tolerance))

        # publish non-ground points
        non_ground_data = data_filtered[~ground_mask]
        non_ground_msg = msgify(PointCloud2, non_ground_data)
        non_ground_msg.header.stamp = msg.header.stamp
        non_ground_msg.header.frame_id = msg.header.frame_id
        self.no_ground_pub.publish(non_ground_msg)

        # publish ground points
        ground_data = data_filtered[ground_mask]
        ground_msg = msgify(PointCloud2, ground_data)
        ground_msg.header.stamp = msg.header.stamp
        ground_msg.header.frame_id = msg.header.frame_id
        self.ground_pub.publish(ground_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('ground_removal', log_level=rospy.INFO)
    node = GroundRemovalNode()
    node.run()