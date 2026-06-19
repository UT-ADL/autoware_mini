#!/usr/bin/env python3

import rospy
import numpy as np

from numpy.lib.recfunctions import structured_to_unstructured, unstructured_to_structured
from ros_numpy import numpify, msgify
import dynamic_reconfigure.server
from autoware_mini.cfg import PointsClustererConfig

from sensor_msgs.msg import PointCloud2

class PointsClusterer:
    def __init__(self):
        self.cluster_epsilon = rospy.get_param('~cluster_epsilon')
        self.cluster_min_samples = rospy.get_param('~cluster_min_samples')
        self.cluster_in_2d = rospy.get_param('~cluster_in_2d')

        try:
            from cuml.cluster import DBSCAN
            self.clusterer = DBSCAN(eps=self.cluster_epsilon, min_samples=self.cluster_min_samples)
            rospy.loginfo("%s - using DBSCAN from cuML", rospy.get_name())
        except ImportError:
            try:
                from sklearnex.cluster import DBSCAN
                self.clusterer = DBSCAN(eps=self.cluster_epsilon, min_samples=self.cluster_min_samples, algorithm='auto')
                rospy.loginfo("%s - using DBSCAN from Intel® Extension for Scikit-learn", rospy.get_name())
            except ImportError:
                from sklearn.cluster import DBSCAN
                self.clusterer = DBSCAN(eps=self.cluster_epsilon, min_samples=self.cluster_min_samples, algorithm='ball_tree')
                rospy.loginfo("%s - using DBSCAN from Scikit-learn", rospy.get_name())

        # Warmup DBSCAN to avoid slow first callback
        dummy_points = np.random.rand(1000, 3).astype(np.float32)
        self.clusterer.fit_predict(dummy_points[:, :2] if self.cluster_in_2d else dummy_points)

        self.reconfigure_server = dynamic_reconfigure.server.Server(PointsClustererConfig, self.reconfigure_callback)

        self.cluster_pub = rospy.Publisher('points_clustered', PointCloud2, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('points_filtered', PointCloud2, self.points_callback, queue_size=1, buff_size=2**24, tcp_nodelay=True)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def reconfigure_callback(self, config, level):
        rospy.loginfo("%s - reconfiguring parameters", rospy.get_name())
        
        # Update parameters
        self.cluster_epsilon = config.cluster_epsilon
        self.cluster_min_samples = config.cluster_min_samples
        self.cluster_in_2d = config.cluster_in_2d
        
        # Reinitialize clusterer with new parameters
        self.clusterer.set_params(eps=self.cluster_epsilon, min_samples=self.cluster_min_samples)
        
        return config

    def points_callback(self, msg):
        data = numpify(msg)

        # convert point cloud into ndarray, take only xyz coordinates
        points = structured_to_unstructured(data[['x', 'y', 'z']], dtype=np.float32)

        # get labels for clusters
        labels = self.clusterer.fit_predict(np.ascontiguousarray(points[:, :2], dtype=np.float32) if self.cluster_in_2d else points)

        filter_idx = np.nonzero(labels != -1) # remove noise label (-1)

        cluster_labels = labels[filter_idx]
        points_clustered = points[filter_idx]

        # concatenate points with labels
        points_labeled = np.hstack((points_clustered, cluster_labels.reshape(-1, 1)))

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
