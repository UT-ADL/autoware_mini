#!/usr/bin/env python3

import rospy
import numpy as np

from numpy.lib.recfunctions import structured_to_unstructured, unstructured_to_structured
from ros_numpy import numpify, msgify

from sensor_msgs.msg import PointCloud2

class PointsClusterer:
    def __init__(self):
        self.sample_size = rospy.get_param('~sample_size')
        self.cluster_epsilon = rospy.get_param('~cluster_epsilon')
        self.cluster_min_size = rospy.get_param('~cluster_min_size')

        try:
            from cuml.cluster import DBSCAN
            self.clusterer = DBSCAN(eps=self.cluster_epsilon, min_samples=self.cluster_min_size)
            rospy.loginfo("Using DBSCAN from cuML")
        except ImportError:
            try:
                from sklearnex.cluster import DBSCAN
                self.clusterer = DBSCAN(eps=self.cluster_epsilon, min_samples=self.cluster_min_size, algorithm='auto')
                rospy.loginfo("Using DBSCAN from Intel® Extension for Scikit-learn")
            except ImportError:
                from sklearn.cluster import DBSCAN
                self.clusterer = DBSCAN(eps=self.cluster_epsilon, min_samples=self.cluster_min_size, algorithm='ball_tree')
                rospy.loginfo("Using DBSCAN from Scikit-learn")

        self.cluster_pub = rospy.Publisher('points_clustered', PointCloud2, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('points_no_ground', PointCloud2, self.points_callback, queue_size=1, buff_size=2**24, tcp_nodelay=True)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def points_callback(self, msg):
        data = numpify(msg)

        # downsample random points to reduce processing time
        if len(data) > self.sample_size:
            data = np.random.choice(data, size=self.sample_size, replace=False)

        # convert point cloud into ndarray, take only xyz coordinates
        points = structured_to_unstructured(data[['x', 'y', 'z']], dtype=np.float32)

        # get labels for clusters
        labels = self.clusterer.fit_predict(points)

        # concatenate points with labels
        points_labeled = np.hstack((points, labels.reshape(-1, 1)))

        # filter out noise points
        points_labeled = points_labeled[labels != -1]

        # convert labeled points to PointCloud2 format
        data = unstructured_to_structured(points_labeled, dtype=np.dtype([
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('label', np.int32)
        ]))

        # publish clustered points message
        cluster_msg = msgify(PointCloud2, data)
        cluster_msg.header.stamp = msg.header.stamp
        cluster_msg.header.frame_id = msg.header.frame_id
        self.cluster_pub.publish(cluster_msg)

        rospy.logdebug("%s - %d points, %d clusters", rospy.get_name(), len(points), np.max(labels) + 1)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('points_clusterer', log_level=rospy.INFO)
    node = PointsClusterer()
    node.run()
