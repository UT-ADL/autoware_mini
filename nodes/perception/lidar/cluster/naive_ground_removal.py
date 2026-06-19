#!/usr/bin/env python3

import math

import rospy
import numpy as np
import cv2

from ros_numpy import numpify, msgify
import dynamic_reconfigure.server
from autoware_mini.cfg import NaiveGroundRemovalConfig

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

class NaiveGroundRemovalNode:
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
        self.filter_iterations = rospy.get_param('~filter_iterations')

        self.reconfigure_server = dynamic_reconfigure.server.Server(NaiveGroundRemovalConfig, self.reconfigure_callback)

        self.width = int(math.ceil((self.max_x - self.min_x) / self.cell_size))
        self.height = int(math.ceil((self.max_y - self.min_y) / self.cell_size))
        self.cols = np.empty((self.width, self.height), dtype=np.float32)

        self.ground_pub = rospy.Publisher('points_ground', PointCloud2, queue_size=1, tcp_nodelay=True)
        self.no_ground_pub = rospy.Publisher('points_no_ground', PointCloud2, queue_size=1, tcp_nodelay=True)
        self.ground_level_pub = rospy.Publisher('ground_level', Marker, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('points_raw', PointCloud2, self.pointcloud_callback, queue_size=1, buff_size=2**24, tcp_nodelay=True)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def reconfigure_callback(self, config, level):
        rospy.loginfo("%s - reconfiguring parameters", rospy.get_name())
        
        # Update parameters
        self.min_x = config.min_x
        self.max_x = config.max_x
        self.min_y = config.min_y
        self.max_y = config.max_y
        self.min_z = config.min_z
        self.max_z = config.max_z
        self.cell_size = config.cell_size
        self.tolerance = config.tolerance
        self.filter = config.filter
        self.filter_size = config.filter_size
        self.filter_iterations = config.filter_iterations
        
        # Recalculate grid dimensions and reinitialize array
        self.width = int(math.ceil((self.max_x - self.min_x) / self.cell_size))
        self.height = int(math.ceil((self.max_y - self.min_y) / self.cell_size))
        self.cols = np.empty((self.width, self.height), dtype=np.float32)
        
        return config

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
        self.cols.fill(np.inf)
        np.minimum.at(self.cols, (xi, yi), zi)

        # bring cell minimum lower, if all cells around it are lower
        for _ in range(self.filter_iterations):
            if self.filter == 'median':
                mask = np.isinf(self.cols)
                self.cols[mask] = np.nan
                cols_filtered = cv2.medianBlur(self.cols, self.filter_size)
                np.fmin(self.cols, cols_filtered, out=self.cols)
            elif self.filter == 'average':
                mask = np.isinf(self.cols)
                self.cols[mask] = 0
                sums = cv2.blur(self.cols, (self.filter_size, self.filter_size), borderType=cv2.BORDER_REPLICATE)
                counts = cv2.blur((~mask).astype(np.float32), (self.filter_size, self.filter_size), borderType=cv2.BORDER_REPLICATE)
                cols_filtered = np.divide(sums, counts, where=counts > 0)
                np.fmin(self.cols, cols_filtered, out=self.cols)
            elif self.filter == 'minimum':
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

        # publish ground level markers only if there are subscribers
        if self.ground_level_pub.get_num_connections() > 0:
            # generate cube markers for all cells
            marker = Marker()
            marker.header.stamp = msg.header.stamp
            marker.header.frame_id = msg.header.frame_id
            marker.type = Marker.CUBE_LIST
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.color = ColorRGBA(1.0, 1.0, 0.0, 0.5)
            marker.scale.x = self.cell_size
            marker.scale.y = self.cell_size
            marker.scale.z = self.tolerance
            for i in range(self.width):
                for j in range(self.height):
                    if not np.isnan(self.cols[i, j]) and not np.isinf(self.cols[i, j]) and self.cols[i, j] != 0:
                        x = float(self.min_x + (i + 0.5) * self.cell_size)
                        y = float(self.min_y + (j + 0.5) * self.cell_size)
                        z = float(self.cols[i, j] + self.tolerance / 2.0)
                        marker.points.append(Point(x=x, y=y, z=z))
            self.ground_level_pub.publish(marker)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('naive_ground_removal', log_level=rospy.INFO)
    node = NaiveGroundRemovalNode()
    node.run()