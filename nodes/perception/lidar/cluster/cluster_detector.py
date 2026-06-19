#!/usr/bin/env python3

import rospy
import math
import numpy as np
import cv2
import shapely

from collections import defaultdict

import tf2_ros
from ros_numpy import numpify

from sensor_msgs.msg import PointCloud2
from std_msgs.msg import ColorRGBA
from autoware_mini.msg import DetectedObjectArray, DetectedObject

BLUE = ColorRGBA(0.0, 0.0, 1.0, 0.5)

class ClusterDetector:
    def __init__(self):
        self.min_cluster_size = rospy.get_param('~min_cluster_size')
        self.bounding_box_type = rospy.get_param('~bounding_box_type')
        self.output_frame = rospy.get_param('/perception/output_frame')
        self.transform_timeout = rospy.get_param('~transform_timeout')
        self.concave_hull_area_threshold = rospy.get_param('~concave_hull_area_threshold')
        self.concave_hull_ratio = rospy.get_param('~concave_hull_ratio')

        if self.bounding_box_type not in ["axis_aligned", "min_area"]:
            raise ValueError(f"{rospy.get_name()} - 'bounding_box_type' must be one of 'axis_aligned' or 'min_area', not '{self.bounding_box_type}'")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.object_pool = defaultdict(DetectedObject)

        self.objects_pub = rospy.Publisher('detected_objects', DetectedObjectArray, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('points_clustered', PointCloud2, self.points_callback, queue_size=1, buff_size=2**24, tcp_nodelay=True)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def points_callback(self, msg):
        data = numpify(msg)

        # convert point cloud into ndarray, take only xyz coordinates
        points_homogeneous = np.stack([data['x'], data['y'], data['z'], data['z']], axis=-1).reshape(-1, 4).astype(np.float32)
        points_homogeneous[:, 3] = 1.0  # Add homogeneous coordinate
        labels = data['label']

        # if target frame does not match the header frame
        if msg.header.frame_id != self.output_frame:
            # fetch transform for target frame
            try:
                transform = self.tf_buffer.lookup_transform(self.output_frame, msg.header.frame_id, msg.header.stamp, rospy.Duration(self.transform_timeout))
            except (tf2_ros.TransformException, rospy.ROSTimeMovedBackwardsException) as e:
                rospy.logwarn("%s - %s", rospy.get_name(), e)
                return
            tf_matrix = numpify(transform.transform).astype(np.float32)
            # transform points to target frame
            points_homogeneous = points_homogeneous.dot(tf_matrix.T)
        
        # create detected objects
        objects = DetectedObjectArray()
        objects.header.stamp = msg.header.stamp
        objects.header.frame_id = self.output_frame

        # sort points by labels
        sorted_indices = np.argsort(labels)
        sorted_labels = labels[sorted_indices]
        sorted_points = points_homogeneous[sorted_indices]

        # get the split indices
        unique_labels, label_starts, label_counts = np.unique(sorted_labels, return_index=True, return_counts=True)
        label_ends = label_starts + label_counts

        # vectorized centroid and bounds computation across all clusters
        centroids = np.add.reduceat(sorted_points[:, :3], label_starts, axis=0) / label_counts[:, np.newaxis]
        maxs = np.maximum.reduceat(sorted_points[:, :3], label_starts)
        mins = np.minimum.reduceat(sorted_points[:, :3], label_starts)
        dims = maxs - mins
        centers = (maxs + mins) / 2
 
        for i, (label, start, end, count) in enumerate(zip(unique_labels, label_starts, label_ends, label_counts)):
            # filter out small clustersc
            if count < self.min_cluster_size:
                continue

            # fetch points for this cluster
            centroid_x, centroid_y, centroid_z = centroids[i].tolist()
            # cv2.convexHull needs contiguous array of 2D points
            points2d = np.ascontiguousarray(sorted_points[start:end, :2])

            if self.bounding_box_type == 'axis_aligned':
                dim_x, dim_y, dim_z = dims[i].tolist()
                center_x, center_y, center_z = centers[i].tolist()

                # always pointing forward
                heading = 0.0

            elif self.bounding_box_type == 'min_area':
                # calculate minimum area bounding box
                (center_x, center_y), (dim_x, dim_y), heading_angle = cv2.minAreaRect(points2d)

                # convert degrees to radians for heading angle
                heading = math.radians(heading_angle)

                # height and vertical position
                dim_z = float(dims[i, 2])
                center_z = float(centers[i, 2])

            else:
                assert False, "wrong bounding_box_type: " + self.bounding_box_type

            # create DetectedObject
            object = self.object_pool[label]
            object.id = int(label)
            object.label = "unknown"
            object.color = BLUE
            object.valid = True
            object.centroid.x = centroid_x
            object.centroid.y = centroid_y
            object.centroid.z = centroid_z
            object.center.x = center_x
            object.center.y = center_y
            object.center.z = center_z
            object.heading = heading
            object.dimensions.x = dim_x
            object.dimensions.y = dim_y
            object.dimensions.z = dim_z
            object.position_reliable = False
            object.velocity_reliable = False
            object.acceleration_reliable = False
            
            # use concave hull for large clusters to avoid inflating into road space
            rect_area = dim_x * dim_y
            if rect_area > self.concave_hull_area_threshold:
                multipoint = shapely.multipoints(points2d)
                hull_polygon = shapely.concave_hull(multipoint, ratio=self.concave_hull_ratio)
                hull_points = shapely.get_coordinates(hull_polygon)[:-1]
            else:
                hull_points = cv2.convexHull(points2d)[:, 0, :]

            object.convex_hull = np.concatenate((hull_points, np.full((hull_points.shape[0], 1), mins[i, 2])), axis=1).ravel().tolist()
            objects.objects.append(object)

        # publish detected objects message
        self.objects_pub.publish(objects)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('cluster_detector', log_level=rospy.INFO)
    node = ClusterDetector()
    node.run()
