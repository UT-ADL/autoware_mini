#!/usr/bin/env python3
import traceback
import numpy as np
from numpy.lib.recfunctions import structured_to_unstructured, unstructured_to_structured

import rospy
import message_filters
from tf2_ros import TransformListener, Buffer, TransformException

from sensor_msgs.msg import PointCloud2
from ros_numpy import numpify, msgify


class PointsConcatenator:
    def __init__(self):

        # Publisher
        self.points_concatenated_pub = rospy.Publisher("points_concatenated", PointCloud2, queue_size=1, tcp_nodelay=True)

        # Subscribers
        points1_sub = message_filters.Subscriber('points1', PointCloud2, queue_size=1, buff_size=2**24, tcp_nodelay=True)
        points2_sub = message_filters.Subscriber('points2', PointCloud2, queue_size=1, buff_size=2**24, tcp_nodelay=True)

        # Strict Time Sync
        ts = message_filters.ApproximateTimeSynchronizer([points1_sub, points2_sub], queue_size=15, slop=0.15)
        ts.registerCallback(self.syncronised_callback)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def syncronised_callback(self, points1_msg, points2_msg):
        """
        points1_msg: sensor_msgs/PointCloud2
        points2_msg: sensor_msgs/PointCloud2
        publish: sensor_msgs/PointCloud2
        """
        try:
            if points1_msg.header.frame_id != points2_msg.header.frame_id:
                raise Exception("Pointcloud frame_ids do not match: {} vs {}".format(points1_msg.header.frame_id, points2_msg.header.frame_id))

            points1_array = numpify(points1_msg)
            points2_array = numpify(points2_msg)

            # concatenate the two structured arrays using only the required fields
            common_fields = [field for field in points1_array.dtype.names if field in points2_array.dtype.names]
            points_concatenated = np.concatenate((points1_array[common_fields], points2_array[common_fields]), axis=0)

            # msgify, stamp and publish the concatenated cloud
            points_concatenated_msg = msgify(PointCloud2, points_concatenated)
            points_concatenated_msg.header = points1_msg.header
            self.points_concatenated_pub.publish(points_concatenated_msg)

        except Exception as e:
            rospy.logerr_throttle(10, "%s - Exception in callback: %s", rospy.get_name(), traceback.format_exc())

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('points_concatentor', log_level=rospy.INFO)
    node = PointsConcatenator()
    node.run()
