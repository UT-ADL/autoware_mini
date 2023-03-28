#!/usr/bin/env python3

import rospy
import numpy as np
import cv2

from ros_numpy import numpify, msgify

from sensor_msgs.msg import PointCloud2
from autoware_msgs.msg import DetectedObjectArray, DetectedObject
from std_msgs.msg import ColorRGBA, Header
from geometry_msgs.msg import Point32

BLUE80P = ColorRGBA(0.0, 0.0, 1.0, 0.8)

class ClusterDetector:
    def __init__(self):
        self.min_cluster_size = rospy.get_param('~min_cluster_size', 5)
        self.bounding_box_type = rospy.get_param('~bounding_box_type', 'axis_aligned')
        self.enable_pointcloud = rospy.get_param('~enable_pointcloud', False)
        self.enable_convex_hull = rospy.get_param('~enable_convex_hull', True)

        self.objects_pub = rospy.Publisher('detected_objects', DetectedObjectArray, queue_size=5)
        rospy.Subscriber('points_clustered', PointCloud2, self.cluster_callback, queue_size=1, buff_size=1024*1024)

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
            points3d = np.empty((len(points), 3), dtype=np.float32)
            points3d[:, 0] = points['x']
            points3d[:, 1] = points['y']
            points3d[:, 2] = points['z']

            # calculate centroid and dimensions
            maxs = np.max(points3d, axis=0)
            mins = np.min(points3d, axis=0)
            center_x, center_y, center_z = (maxs + mins) / 2.0
            dim_x, dim_y, dim_z = maxs - mins

            # always pointing forward
            qx = qy = qz = 0.0
            qw = 1.0

            # create DetectedObject
            object = DetectedObject(header=header)
            object.id = i
            object.label = "unknown"
            object.color = BLUE80P
            object.valid = True
            object.space_frame = msg.header.frame_id
            object.pose.position.x = center_x
            object.pose.position.y = center_y
            object.pose.position.z = center_z
            object.pose.orientation.x = qx
            object.pose.orientation.y = qy
            object.pose.orientation.z = qz
            object.pose.orientation.w = qw
            object.dimensions.x = dim_x
            object.dimensions.y = dim_y
            object.dimensions.z = dim_z
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
                points2d = np.ascontiguousarray(points3d[:,:2])
                hull_points = cv2.convexHull(points2d)[:,0,:]
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
