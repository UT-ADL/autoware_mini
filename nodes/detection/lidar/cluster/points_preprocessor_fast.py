#!/usr/bin/env python3

import rospy
import numpy as np
import cupy as cp
import message_filters

from tf2_ros import TransformListener, Buffer, TransformException
from sensor_msgs.msg import PointCloud2
from cuml.cluster import DBSCAN
from ros_numpy import numpify, msgify

from autoware_mini.naive_ground_detector import NaiveGroundDetectorFast

class PointsPreprocessorFast:
    def __init__(self):

        points_topics = rospy.get_param("~points_topics")
        self.output_frame = rospy.get_param("~output_frame")
        self.transform_timeout = rospy.get_param('~transform_timeout')

        # Crop box parameters
        outer_min_x = rospy.get_param('~outer_min_x')
        outer_max_x = rospy.get_param('~outer_max_x')
        outer_min_y = rospy.get_param('~outer_min_y')
        outer_max_y = rospy.get_param('~outer_max_y')
        outer_min_z = rospy.get_param('~outer_min_z')
        outer_max_z = rospy.get_param('~outer_max_z')

        inner_min_x = rospy.get_param('~inner_min_x')
        inner_max_x = rospy.get_param('~inner_max_x')
        inner_min_y = rospy.get_param('~inner_min_y')
        inner_max_y = rospy.get_param('~inner_max_y')
        inner_min_z = rospy.get_param('~inner_min_z')
        inner_max_z = rospy.get_param('~inner_max_z')

        # Ground detection parameters
        cell_size = rospy.get_param('~ground_removal_cell_size')
        tolerance = rospy.get_param('~ground_removal_tolerance')
        filter_size = rospy.get_param('~ground_removal_filter_size')
        filter_iterations = rospy.get_param('~ground_removal_filter_iterations')

        # Voxel grid filter parameters
        voxel_size = cp.asarray(rospy.get_param('~voxel_grid_filter_leaf_size'))
        self.voxel_grid_filter_leaf_size = cp.asanyarray((voxel_size, voxel_size, voxel_size))
        # Assumption: coordinates are within reasonable bounds (e.g., [-1000, 1000])
        self.hash_scale = cp.array([73856093, 19349669, 83492791], dtype=cp.int64)  # large primes

        # Clustering parameters
        self.cluster_epsilon = rospy.get_param('~cluster_epsilon')
        self.cluster_min_samples = rospy.get_param('~cluster_min_samples')
        self.cluster_in_2d = rospy.get_param('~cluster_in_2d')

        self.outer_min_gpu = cp.array([outer_min_x, outer_min_y, outer_min_z])
        self.outer_max_gpu = cp.array([outer_max_x, outer_max_y, outer_max_z])
        self.inner_min_gpu = cp.array([inner_min_x, inner_min_y, inner_min_z])
        self.inner_max_gpu = cp.array([inner_max_x, inner_max_y, inner_max_z])

        self.ground_detector = NaiveGroundDetectorFast(outer_min_x, outer_max_x, outer_min_y, outer_max_y, cell_size, tolerance, filter_size, filter_iterations)
        
        self.clusterer = DBSCAN(eps=self.cluster_epsilon, min_samples=self.cluster_min_samples)

        # TF buffer setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        # Warmup GPU with dummy data
        for _ in range(3):
            points_gpu = cp.random.rand(131072, 4).astype(cp.float32) * 100
            nan_idx = cp.nonzero(cp.random.rand(*points_gpu.shape) < .1) # Randomly set 10% of elements to NaN
            points_gpu[nan_idx] = cp.nan
            idx = cp.nonzero(cp.all(~cp.isnan(points_gpu), axis=1))
            points_gpu[idx]

        # Publisher
        self.points_clustered_pub = rospy.Publisher('points_clustered', PointCloud2, queue_size=1, tcp_nodelay=True)
        self.points_ground_pub = rospy.Publisher('points_ground', PointCloud2, queue_size=1, tcp_nodelay=True)
        self.points_no_ground_pub = rospy.Publisher('points_no_ground', PointCloud2, queue_size=1, tcp_nodelay=True)

        subscribers = []
        for topic in points_topics:
            subscribers.append(message_filters.Subscriber(topic, PointCloud2, queue_size=1, buff_size=2**24, tcp_nodelay=True))

        if not subscribers:
            raise ValueError("No topics to subscribe to.")

        ts = message_filters.ApproximateTimeSynchronizer(subscribers, queue_size=4, slop=0.1)
        ts.registerCallback(self.synced_pointcloud_callback)

        self.transforms = [None] * len(subscribers)
        rospy.loginfo("%s - initialized", rospy.get_name())

    def synced_pointcloud_callback(self, *msgs):
        pointclouds = []
        for i, msg in enumerate(msgs):
            points_array = numpify(msg)
            if msg.header.frame_id == self.output_frame:
                points = np.stack([points_array['x'], points_array['y'], points_array['z']], axis=-1).reshape(-1, 3)
                points = cp.asarray(points)

                idx = cp.nonzero(cp.all(~cp.isnan(points), axis=1))
                points = points[idx]
                pointclouds.append(points)
            
            else:
                # Static transforms, fetch only once
                if self.transforms[i] is None:
                    try:
                        transform = self.tf_buffer.lookup_transform(self.output_frame, msg.header.frame_id, rospy.Time(0), rospy.Duration(self.transform_timeout))
                    except (TransformException, rospy.ROSTimeMovedBackwardsException) as e:
                        rospy.logwarn("%s - %s", rospy.get_name(), e)
                        return
                
                    transfrom_matrix = numpify(transform.transform).T
                    transfrom_matrix_gpu = cp.asarray(transfrom_matrix).astype(cp.float32) # Use GPU acceleration
                    self.transforms[i] = transfrom_matrix_gpu

                untransformed_points = np.stack([points_array['x'], points_array['y'], points_array['z'], points_array['z']], axis=-1).reshape(-1, 4)
                untransformed_points[:, 3] = 1.0 # Add homogeneous coordinate
                untransformed_points = cp.asarray(untransformed_points)
                
                idx = cp.nonzero(cp.all(~cp.isnan(untransformed_points), axis=1))
                untransformed_points = untransformed_points[idx]
                # Transform points to output frame
                points = cp.matmul(untransformed_points, self.transforms[i])
                pointclouds.append(points[:, :3])

        # Concatenate poionts
        points_concatenated = cp.concatenate(pointclouds, axis=0)

        # Filter points
        in_outer = cp.all((points_concatenated >= self.outer_min_gpu) & (points_concatenated <= self.outer_max_gpu), axis=1)
        in_inner = cp.all((points_concatenated >= self.inner_min_gpu) & (points_concatenated <= self.inner_max_gpu), axis=1)
        keep_idx = cp.nonzero(in_outer & (~in_inner))
        points_filtered = points_concatenated[keep_idx]

        # Remove ground points
        ground_mask = self.ground_detector.detect_ground(points_filtered)
        no_ground_idx = cp.nonzero(~ground_mask)
        points_no_ground = points_filtered[no_ground_idx]
        
        # Downsample points
        points_downsampled = self.voxel_grid_filter_gpu(points_no_ground)

        # Cluster points
        labels = self.clusterer.fit_predict(points_downsampled[:, :2] if self.cluster_in_2d else points_downsampled)
        valid_idx = cp.nonzero(labels != -1) # remove noise label (-1)
        valid_labels = labels[valid_idx] 
        valid_points = points_downsampled[valid_idx]

        points_clustered = cp.asnumpy(valid_points).astype(np.float32)
        cluster_labels = cp.asnumpy(valid_labels).astype(np.int32)

        # Publish points
        self.publish_points(points_clustered, cluster_labels, msgs[0].header.stamp, self.points_clustered_pub)

        if self.points_ground_pub.get_num_connections() > 0:
            points_ground = points_filtered[np.nonzero(ground_mask)]
            points_ground = cp.asnumpy(points_ground).astype(np.float32)
            self.publish_points(points_ground, None, msgs[0].header.stamp, self.points_ground_pub)

        if self.points_no_ground_pub.get_num_connections() > 0:
            points_no_ground = cp.asnumpy(points_no_ground).astype(np.float32)
            self.publish_points(points_no_ground, None, msgs[0].header.stamp, self.points_no_ground_pub)
    
    def voxel_grid_filter_gpu(self, points):
        """
        Voxel grid downsampling with GPU acceleration.
        Args:
            points: Nx3 cupy array (x, y, z)
            voxel_size: tuple or float, e.g., (0.1, 0.1, 0.1)
        Returns:
            Downsampled points (as cupy array).
        """
        # Compute voxel indices
        voxel_indices = cp.floor(points / self.voxel_grid_filter_leaf_size).astype(cp.int64)

        # Hash voxel indices into scalar keys
        voxel_hashes = cp.sum(voxel_indices * self.hash_scale, axis=1)

        # Unique voxel hashes and corresponding first indices
        _, unique_indices = cp.unique(voxel_hashes, return_index=True)
        
        # Select representative points (first in voxel)
        downsampled = points[unique_indices]
    
        return downsampled
    
    def publish_points(self, points, labels, stamp, publisher):
        """
        Publish points as PointCloud2 message.
        """
        dtype = [('x', np.float32), ('y', np.float32), ('z', np.float32)]
        if labels is not None:
            dtype.append(('label', np.int32))

        data = np.empty(points.shape[0], dtype=dtype)
        data['x'] = points[:, 0]
        data['y'] = points[:, 1]
        data['z'] = points[:, 2]
        if labels is not None:
            data['label'] = labels
        
        points_msg = msgify(PointCloud2, data)
        points_msg.header.stamp = stamp
        points_msg.header.frame_id = self.output_frame
        publisher.publish(points_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('points_preprocessor_fast', log_level=rospy.INFO)
    node = PointsPreprocessorFast()
    node.run()
