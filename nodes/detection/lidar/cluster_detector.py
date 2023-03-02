#!/usr/bin/env python

import rospy
import numpy as np

from ros_numpy import numpify, msgify
from scipy.spatial import ConvexHull

from sensor_msgs.msg import PointCloud2
from autoware_msgs.msg import DetectedObjectArray, DetectedObject
from std_msgs.msg import ColorRGBA, Header
from geometry_msgs.msg import Point, Quaternion, Vector3, Point32

class ClusterDetector:
    def __init__(self):
        self.min_cluster_size = rospy.get_param('~min_cluster_size', 5)
        self.enable_pointcloud = rospy.get_param('~enable_pointcloud', False)
        self.enable_convex_hull = rospy.get_param('~enable_convex_hull', True)

        self.cluster_sub = rospy.Subscriber('points_clustered', PointCloud2, self.cluster_callback, queue_size=1, buff_size=1024*1024)
        self.objects_pub = rospy.Publisher('detected_objects', DetectedObjectArray, queue_size=1)

    def cluster_callback(self, msg):
        data = numpify(msg)

        # prepare header for all objects
        header = Header(stamp=msg.header.stamp, frame_id=msg.header.frame_id)

        labels = data['label']
        objects = DetectedObjectArray(header=header)
        for i in range(np.max(labels) + 1):
            # fetch points for this cluster
            points = data[labels == i]

            # ignore clusters smaller than certain size
            if len(points) < self.min_cluster_size:
                continue

            # convert points to ndarray for simpler processing
            ndpoints = np.empty((len(points), 3))
            ndpoints[:, 0] = points['x']
            ndpoints[:, 1] = points['y']
            ndpoints[:, 2] = points['z']

            # calculate centroid and dimensions
            center_x, center_y, center_z = np.mean(ndpoints, axis=0)
            dim_x, dim_y, dim_z = np.max(ndpoints, axis=0) - np.min(ndpoints, axis=0)

            # create DetectedObject
            object = DetectedObject(header=header)
            object.id = i
            object.label = "unknown"
            object.color = ColorRGBA(0.0, 0.0, 1.0, 0.8)
            object.valid = True
            object.space_frame = msg.header.frame_id
            object.pose.position = Point(center_x, center_y, center_z)
            object.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
            object.dimensions = Vector3(dim_x, dim_y, dim_z)
            object.pose_reliable = True
            object.velocity_reliable = False
            object.acceleration_reliable = False
            #object.velocity
            #object.acceleration
            #object.candidate_trajectories

            # create pointcloud
            if self.enable_pointcloud:
                object.pointcloud = msgify(PointCloud2, points)

            # create convex hull
            if self.enable_convex_hull:
                assert len(ndpoints) > 2
                ndpoints = ndpoints[:, :2]
                hull_points = ndpoints[ConvexHull(ndpoints).vertices]
                object.convex_hull.polygon.points = [Point32(x, y, center_z) for x, y in hull_points]

            objects.objects.append(object)

        # publish detected objects message
        self.objects_pub.publish(objects)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('cluster_detector', log_level=rospy.INFO)
    node = ClusterDetector()
    node.run()
