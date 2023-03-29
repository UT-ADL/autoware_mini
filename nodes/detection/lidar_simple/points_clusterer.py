#!/usr/bin/env python

import rospy
import numpy as np

from ros_numpy import numpify, msgify
try:
    from sklearnex.cluster import DBSCAN
    DBSCAN_ALGORITHM = 'auto'
except ImportError:
    rospy.logwarn("Intel® Extension for Scikit-learn not found, reverting to original Scikit-learn. To speed up clustering install Intel® Extension for Scikit-learn, see https://intel.github.io/scikit-learn-intelex/.")
    from sklearn.cluster import DBSCAN
    DBSCAN_ALGORITHM = 'ball_tree'

from sensor_msgs.msg import PointCloud2

class PointsClusterer:
    def __init__(self):
        self.sample_size = rospy.get_param('~sample_size', 10000)
        self.cluster_epsilon = rospy.get_param('~cluster_epsilon', 1.0)
        self.cluster_min_size = rospy.get_param('~cluster_min_size', 7)

        self.clusterer = DBSCAN(eps=self.cluster_epsilon, min_samples=self.cluster_min_size, algorithm=DBSCAN_ALGORITHM)

        self.cluster_pub = rospy.Publisher('points_clustered', PointCloud2, queue_size=5)
        rospy.Subscriber('points_no_ground', PointCloud2, self.points_callback, queue_size=1, buff_size=1024*1024)

        rospy.loginfo("points_clusterer - initialized")

    def points_callback(self, msg):
        data = numpify(msg)

        # downsample random points to reduce processing time
        if len(data) > self.sample_size:
            data = np.random.choice(data, size=self.sample_size, replace=False)

        # convert point cloud into ndarray, take only xyz coordinates
        points = np.empty((data.shape[0], 3), dtype=np.float32)
        points[:, 0] = data['x']
        points[:, 1] = data['y']
        points[:, 2] = data['z']

        # get labels for clusters
        labels = self.clusterer.fit_predict(points)

        # populate data with labels
        data = np.empty(points.shape[0], dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('label', np.int32)
        ])
        data['x'] = points[:, 0]
        data['y'] = points[:, 1]
        data['z'] = points[:, 2]
        data['label'] = labels

        # filter out noise points
        data = data[labels != -1]

        # publish clustered points message
        cluster_msg = msgify(PointCloud2, data)
        cluster_msg.header.stamp = msg.header.stamp
        cluster_msg.header.frame_id = msg.header.frame_id
        self.cluster_pub.publish(cluster_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('points_clusterer', log_level=rospy.INFO)
    node = PointsClusterer()
    node.run()
