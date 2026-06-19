#!/usr/bin/env python3

import math
import rospy
import numpy as np
import cupy as cp
import cupyx
import cupyx.scipy.ndimage
import message_filters

import tf2_ros
from sensor_msgs.msg import PointCloud2
import cuml.cluster
from ros_numpy import numpify, msgify

class PointsPreprocessorGpu:
    def __init__(self):

        points_topics = rospy.get_param("~points_topics")
        if not points_topics:
            raise ValueError("No topics to subscribe to.")

        self.filter_frame = rospy.get_param("~filter_frame")
        self.output_frame = rospy.get_param("~output_frame")
        self.synchronizer_queue_size = rospy.get_param('~synchronizer_queue_size')
        self.synchronizer_slop = rospy.get_param('~synchronizer_slop')
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

        self.outer_min_gpu = cp.array([outer_min_x, outer_min_y, outer_min_z])
        self.outer_max_gpu = cp.array([outer_max_x, outer_max_y, outer_max_z])
        self.inner_min_gpu = cp.array([inner_min_x, inner_min_y, inner_min_z])
        self.inner_max_gpu = cp.array([inner_max_x, inner_max_y, inner_max_z])

        # Ground removal parameters
        filter_size = rospy.get_param('~ground_removal_filter_size')
        self.ground_removal_cell_size = rospy.get_param('~ground_removal_cell_size')
        self.ground_removal_tolerance = rospy.get_param('~ground_removal_tolerance')
        self.ground_removal_filter_iterations = rospy.get_param('~ground_removal_filter_iterations')
        self.ground_removal_kernel = cp.ones((filter_size, filter_size), dtype=cp.float32) / filter_size**2
        self.outer_min_x = outer_min_x
        self.outer_max_x = outer_max_x
        self.outer_min_y = outer_min_y
        self.outer_max_y = outer_max_y

        # Voxel grid filter parameters
        voxel_size = cp.asarray(rospy.get_param('~voxel_grid_filter_leaf_size'))
        self.voxel_point = rospy.get_param('~voxel_grid_filter_point')
        self.voxel_grid_filter_leaf_size = cp.asanyarray((voxel_size, voxel_size, voxel_size))
        # Assumption: coordinates are within reasonable bounds (e.g., [-1000, 1000])
        self.hash_scale = cp.array([73856093, 19349669, 83492791], dtype=cp.int64)  # large primes
        if self.voxel_point not in ['random', 'centroid']:
            raise ValueError(f"{rospy.get_name()} - 'voxel_grid_filter_point' must be one of 'random' or 'centroid', not '{self.voxel_point}'")

        # Deskewing parameters
        deskew_lidar_scan_time = rospy.get_param('~deskew_lidar_scan_time')
        self.deskew_num_bins = rospy.get_param('~deskew_num_bins')
        self.deskew_bins = cp.linspace(0, deskew_lidar_scan_time, self.deskew_num_bins + 1)
        self.deskew_step = deskew_lidar_scan_time / self.deskew_num_bins

        # Clustering parameters
        cluster_epsilon = rospy.get_param('~cluster_epsilon')
        cluster_min_samples = rospy.get_param('~cluster_min_samples')
        self.cluster_in_2d = rospy.get_param('~cluster_in_2d')
        self.clusterer = cuml.cluster.DBSCAN(eps=cluster_epsilon, min_samples=cluster_min_samples)

        # TF buffer setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Warmup GPU with dummy data
        for _ in range(3):
            points_gpu = cp.random.rand(131072, 4).astype(cp.float32) * 100
            nan_idx = cp.nonzero(cp.random.rand(*points_gpu.shape) < .1) # Randomly set 10% of elements to NaN
            points_gpu[nan_idx] = cp.nan
            idx = cp.nonzero(cp.all(~cp.isnan(points_gpu), axis=1))
            points_gpu[idx]
            self.clusterer.fit_predict(points_gpu[:, :2] if self.cluster_in_2d else points_gpu)

        # Publisher
        self.points_ground_pub = rospy.Publisher('points_ground', PointCloud2, queue_size=1, tcp_nodelay=True)
        self.points_no_ground_pub = rospy.Publisher('points_no_ground', PointCloud2, queue_size=1, tcp_nodelay=True)
        self.points_clustered_pub = rospy.Publisher('points_clustered', PointCloud2, queue_size=1, tcp_nodelay=True)

        subscribers = []
        for topic in points_topics:
            subscribers.append(message_filters.Subscriber(topic, PointCloud2, queue_size=1, buff_size=2**24, tcp_nodelay=True))

        ts = message_filters.ApproximateTimeSynchronizer(subscribers, queue_size=self.synchronizer_queue_size, slop=self.synchronizer_slop)
        ts.registerCallback(self.synced_pointcloud_callback)

        self.transforms = [None] * len(subscribers)
        rospy.loginfo("%s - initialized", rospy.get_name())

    def synced_pointcloud_callback(self, *msgs):
        apply_deskew = True

        pointclouds = []
        stamps = []
        for i, msg in enumerate(msgs):
            stamps.append(msg.header.stamp)        
            points = numpify(msg)
            points_xyz = np.stack([points['x'], points['y'], points['z']], axis=-1)
            points_xyz = cp.asarray(points_xyz)
            if len(points.shape) > 1: # Handle organized pointclouds
                points_xyz = points_xyz.reshape(-1, 3)
                idx = cp.nonzero(~cp.isnan(points_xyz[:, 0]))
                points_xyz = points_xyz[idx]

            if msg.header.frame_id != self.filter_frame:
                # Static transforms, fetch only once
                if self.transforms[i] is None:
                    try:
                        transform = self.tf_buffer.lookup_transform(self.filter_frame, msg.header.frame_id, rospy.Time(0), rospy.Duration(self.transform_timeout))
                    except (tf2_ros.TransformException, rospy.ROSTimeMovedBackwardsException) as e:
                        rospy.logwarn("%s - %s", rospy.get_name(), e)
                        return
                
                    transfrom_matrix = numpify(transform.transform).T
                    transfrom_matrix_gpu = cp.asarray(transfrom_matrix, dtype=cp.float32)
                    self.transforms[i] = transfrom_matrix_gpu

                untransformed_points = cp.hstack((points_xyz, cp.ones_like(points_xyz[:, :1])))
                
                # Transform points to output frame
                points_xyz = cp.matmul(untransformed_points, self.transforms[i])[:, :3]

            # Add timestamp field
            if 't' in points.dtype.names:
                if len(points['t'].shape) > 1:
                    t = cp.asarray(points["t"].flatten())[idx]
                else:
                    t = cp.asarray(points["t"])
                t = t / 1e9  # Convert to seconds
                t = t.astype(cp.float32) # cp.float64 is slow
            elif 'time' in points.dtype.names:
                t = cp.asarray(points['time'])
                t = t.astype(cp.float32)
            else:
                apply_deskew = False
                t = cp.zeros(points_xyz.shape[0], dtype=cp.float32)
                rospy.logwarn("%s - No timestamp field ('t' or 'time') found in pointcloud.", rospy.get_name())

            points_xyzti = cp.hstack((points_xyz, t[:, cp.newaxis], cp.full_like(t, i)[:, cp.newaxis])) # Append timestamp and scan index
            pointclouds.append(points_xyzti)

        # Concatenate points
        points_concatenated = cp.concatenate(pointclouds, axis=0)

        # Filter points
        in_outer = cp.all((points_concatenated[:, :3] >= self.outer_min_gpu) & (points_concatenated[:, :3] <= self.outer_max_gpu), axis=1)
        in_inner = cp.all((points_concatenated[:, :3] >= self.inner_min_gpu) & (points_concatenated[:, :3] <= self.inner_max_gpu), axis=1)
        keep_idx = cp.nonzero(in_outer & (~in_inner))
        points_filtered = points_concatenated[keep_idx]

        # Remove ground points
        ground_mask = self.detect_ground_gpu(points_filtered[:, :3])
        no_ground_idx = cp.nonzero(~ground_mask)
        points_no_ground = points_filtered[no_ground_idx]

        # Deskew points
        if apply_deskew:
            points_before_downsampling = self.deskew_lidar_points_gpu(points_no_ground, stamps)
        else:
            points_before_downsampling = self.transform_points_gpu(points_no_ground, stamps)

        if points_before_downsampling is None:
            return
        
        # Downsample points
        points_downsampled = self.voxel_grid_filter_gpu(points_before_downsampling[:, :3])

        # Cluster points
        if points_downsampled.size > 0:
            labels = self.clusterer.fit_predict(points_downsampled[:, :2] if self.cluster_in_2d else points_downsampled)
            valid_idx = cp.nonzero(labels != -1) # remove noise label (-1)
            points_clustered = cp.asnumpy(points_downsampled[valid_idx]).astype(np.float32)
            cluster_labels = cp.asnumpy(labels[valid_idx]).astype(np.int32)
        else:
            points_clustered = np.empty((0, 3), dtype=np.float32)
            cluster_labels = np.empty(0, dtype=np.int32)

        # Publish points
        self.publish_points(points_clustered, cluster_labels, msgs[0].header.stamp, self.output_frame, self.points_clustered_pub)

        if self.points_ground_pub.get_num_connections() > 0:
            points_ground = points_filtered[cp.nonzero(ground_mask)]
            points_ground = cp.asnumpy(points_ground).astype(np.float32)
            self.publish_points(points_ground, None, msgs[0].header.stamp, self.filter_frame, self.points_ground_pub)

        if self.points_no_ground_pub.get_num_connections() > 0:
            points_no_ground = cp.asnumpy(points_no_ground).astype(np.float32)
            self.publish_points(points_no_ground, None, msgs[0].header.stamp, self.filter_frame, self.points_no_ground_pub)

    def detect_ground_gpu(self, pointcloud):
        """
        GPU-accelerated ground detection.
        Args:
            pointcloud: Nx3 cupy array (x, y, z)
        Returns:
            ground_mask: boolean cupy array of length N
        """

        width = int(math.ceil((self.outer_max_x - self.outer_min_x) / self.ground_removal_cell_size))
        height = int(math.ceil((self.outer_max_y - self.outer_min_y) / self.ground_removal_cell_size))
        cols = cp.full((width, height), cp.inf, dtype=cp.float32)

        # convert x and y coordinates into indexes
        xi = ((pointcloud[:, 0] - self.outer_min_x) / self.ground_removal_cell_size).astype(cp.int32)
        yi = ((pointcloud[:, 1] - self.outer_min_y) / self.ground_removal_cell_size).astype(cp.int32)
        zi = pointcloud[:, 2]

        # write minimum height for each cell to cols
        cupyx.scatter_min(cols, (xi, yi), zi)
        
        # bring cell minimum lower, if all cells around it are lower
        for _ in range(self.ground_removal_filter_iterations):
            mask_gpu = cp.isinf(cols)
            cols[mask_gpu] = 0
            cols_filtered = (cupyx.scipy.ndimage.convolve(cols, self.ground_removal_kernel, mode='nearest') / 
                    cupyx.scipy.ndimage.convolve((~mask_gpu).astype(cp.float32), self.ground_removal_kernel, mode='nearest'))
            cp.fmin(cols, cols_filtered, out=cols)

        # filter out closest points to minimum point up to some tolerance
        ground_mask = (zi <= (cols[xi, yi] + self.ground_removal_tolerance))

        # return ground mask
        return ground_mask

    def deskew_lidar_points_gpu(self, points, stamps):
        """
        GPU-accelerated deskewing of lidar points.
        Args:
            points: Nx5 cupy array (x, y, z, time, scan_index)
            stamps: list of rospy.Time, length = number of scans
        Returns:
            deskewed_points: Nx3 cupy array (x, y, z)
        """
        
        num_stamps = len(stamps)
        transforms = np.empty((num_stamps, self.deskew_num_bins, 4, 4), dtype=np.float32)
        
        for stamp_idx, scan_stamp in enumerate(stamps):
            for i in range(self.deskew_num_bins):
                try:
                    # Assumption: scan_stamp is the start time of the scan
                    transform = self.tf_buffer.lookup_transform(self.output_frame, self.filter_frame, 
                                                                scan_stamp + rospy.Duration.from_sec((i + 0.5) * self.deskew_step), 
                                                                rospy.Duration(self.transform_timeout))
                    transforms[stamp_idx, i] = numpify(transform.transform)
                except (tf2_ros.TransformException, rospy.ROSTimeMovedBackwardsException) as e:
                    rospy.logwarn("%s - %s", rospy.get_name(), e)
                    return None
                
        transforms_gpu = cp.asarray(transforms)

        # Get bin indices for each point based on its timestamp
        bin_idxs = cp.digitize(points[:, 3], self.deskew_bins, right=False) - 1
        bin_idxs = cp.clip(bin_idxs, 0, self.deskew_num_bins - 1)
        scan_idxs = points[:, 4].astype(cp.int32)

        # Select transform for each point
        transform_per_point = transforms_gpu[scan_idxs, bin_idxs]
        points[:, 3] = 1 # Homogeneous coordinates

        # Apply transforms
        deskewed_points = cp.einsum('nij,nj->ni', transform_per_point, points[:, :4])
        return deskewed_points[:, :3]

    def transform_points_gpu(self, points, stamps):
        """
        GPU-accelerated transformation of points to output frame.
        Args:
            points: Nx5 cupy array (x, y, z, time, scan_index)
            stamps: list of rospy.Time, length = number of scans
        Returns:
            transformed_points: Nx3 cupy array (x, y, z)
        """

        transforms = np.empty((len(stamps), 4, 4), dtype=np.float32)
        for stamp_idx, stamp in enumerate(stamps):
            try:
                transform = self.tf_buffer.lookup_transform(self.output_frame, self.filter_frame, stamp, rospy.Duration(self.transform_timeout))
                transforms[stamp_idx] = numpify(transform.transform)
            except (tf2_ros.TransformException, rospy.ROSTimeMovedBackwardsException) as e:
                rospy.logwarn("%s - %s", rospy.get_name(), e)
                return None
            
        transforms_gpu = cp.asarray(transforms)
        scan_idxs = points[:, 4].astype(cp.int32)

        # Select transform for each point
        transform_per_point = transforms_gpu[scan_idxs]
        points[:, 3] = 1 # Homogeneous coordinates

        # Apply transforms
        points_transformed = cp.einsum('nij,nj->ni', transform_per_point, points[:, :4])
        return points_transformed
    
    def voxel_grid_filter_gpu(self, points):
        """
        Voxel grid downsampling with GPU acceleration.
        Args:
            points: Nx3 cupy array (x, y, z)
        Returns:
            Mx3 cupy array of points representing voxels.
        """
        # Compute voxel indices
        voxel_indices = cp.floor(points / self.voxel_grid_filter_leaf_size).astype(cp.int64)

        # Hash voxel indices into scalar keys
        voxel_hashes = cp.sum(voxel_indices * self.hash_scale, axis=1)

        if self.voxel_point == 'random': # select a random point from each voxel
            # Unique voxel hashes and corresponding first indices
            _, unique_indices = cp.unique(voxel_hashes, return_index=True)
            return points[unique_indices]

        elif self.voxel_point == 'centroid': # compute centroid of points in each voxel
            # Find unique voxels and map each point -> voxel group id
            # inverse[i] gives the index of the unique voxel that points[i] belongs to
            _, inverse = cp.unique(voxel_hashes, return_inverse=True)

            counts = cp.bincount(inverse)
            sum_x = cp.bincount(inverse, weights=points[:, 0])
            sum_y = cp.bincount(inverse, weights=points[:, 1])
            sum_z = cp.bincount(inverse, weights=points[:, 2])
            centroids = cp.stack((sum_x / counts, sum_y / counts, sum_z / counts), axis=1)
        
            return centroids.astype(cp.float32)
    
    def publish_points(self, points, labels, stamp, frame_id, publisher):
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
        points_msg.header.frame_id = frame_id
        publisher.publish(points_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('points_preprocessor_gpu', log_level=rospy.INFO)
    node = PointsPreprocessorGpu()
    node.run()
