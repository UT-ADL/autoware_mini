#!/usr/bin/env python3

import rospy
import numpy as np

from numpy.lib.recfunctions import structured_to_unstructured, unstructured_to_structured
from ros_numpy import numpify, msgify

from tf2_ros import TransformListener, Buffer, TransformException
from sensor_msgs.msg import PointCloud2

class PointsDeskewer:
    def __init__(self):

        # Parameters
        self.output_frame = rospy.get_param('~output_frame')
        deskew_lidar_scan_time = rospy.get_param('~deskew_lidar_scan_time')
        self.deskew_num_bins = rospy.get_param('~deskew_num_bins')
        self.transform_timeout = rospy.get_param('~transform_timeout')

        # Internal variables
        self.deskew_bins = np.linspace(0, deskew_lidar_scan_time, self.deskew_num_bins + 1)
        self.deskew_step = deskew_lidar_scan_time / self.deskew_num_bins

        # TF buffer setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        # Publishers
        self.deskewed_pub = rospy.Publisher('points_deskewed', PointCloud2, queue_size=1, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber('points_no_ground', PointCloud2, self.points_callback, queue_size=1, buff_size=2**24, tcp_nodelay=True)

        rospy.loginfo("%s - initialized", rospy.get_name())


    def points_callback(self, msg):
        data = numpify(msg)

        # convert point cloud into ndarray, take only xyz coordinates
        points = structured_to_unstructured(data[['x', 'y', 'z']], dtype=np.float32)

        apply_deskewing = True
        if 't' in data.dtype.names:
            t = data["t"].flatten() / 1e9  # Convert to seconds
        elif 'time' in data.dtype.names:
            t = data['time']
        else:
            apply_deskewing = False
            rospy.logwarn("%s - Lidar with has no time field.", rospy.get_name())

        if apply_deskewing:
            points = np.hstack((points, t[:, np.newaxis]))  # Append time as 4th column
            transformed_points = self.deskew_lidar_points(points, msg.header.stamp, msg.header.frame_id)
        else:
            transformed_points = self.transform_without_deskewing(points, msg.header.stamp, msg.header.frame_id)

        if transformed_points is None:
            return
        
        self.publish_pointcloud(transformed_points, msg.header.stamp)

    def deskew_lidar_points(self, points, stamp, frame_id):
        """
        Deskew lidar points
        :param points: Nx4 ndarray (x, y, z, time) of lidar points with time field in seconds
        :param stamp: timestamp of the lidar scan message
        :param frame_id: frame_id of the lidar scans
        :return: Nx3 ndarray of deskewed lidar points
        """

        transforms = []
        for i in range(self.deskew_num_bins):
            try:
                transform = self.tf_buffer.lookup_transform(self.output_frame, frame_id, stamp + rospy.Duration.from_sec((i + 0.5) * self.deskew_step), 
                                                            rospy.Duration(self.transform_timeout))
                transfrom_matrix = numpify(transform.transform)
                transforms.append(transfrom_matrix)
            except (TransformException, rospy.ROSTimeMovedBackwardsException) as e:
                rospy.logwarn("%s - %s", rospy.get_name(), e)
                return None
        
        bin_idxs = np.digitize(points[:, 3], self.deskew_bins, right=False) - 1
        bin_idxs = np.clip(bin_idxs, 0, self.deskew_num_bins - 1)

        transforms_arr = np.stack(transforms, axis=0)
        transform_per_point = transforms_arr[bin_idxs] # Set transform for each point
        points[:, 3] = 1 # Homogeneous coordinates
        
        # Apply transforms
        deskewed_points = np.einsum('nij,nj->ni', transform_per_point, points)
        return deskewed_points[:, :3]
    
    def transform_without_deskewing(self, points, stamp, frame_id):
        """
        Transform points to output frame without deskewing
        :param points: Nx3 ndarray of lidar points
        :param stamp: timestamp of the lidar scan message
        :param frame_id: frame_id of the lidar scans
        :return: Nx3 ndarray of transformed lidar points
        """
        
        try:
            transform = self.tf_buffer.lookup_transform(self.output_frame, frame_id, stamp, rospy.Duration(self.transform_timeout))
            transfrom_matrix = numpify(transform.transform)
        except (TransformException, rospy.ROSTimeMovedBackwardsException) as e:
            rospy.logwarn("%s - %s", rospy.get_name(), e)
            return None

        points_homogeneous = np.hstack((points, np.ones((points.shape[0], 1))))
        points_transformed = points_homogeneous @ transfrom_matrix.T

        return points_transformed[:, :3]
    
    def publish_pointcloud(self, points, stamp):
        """
        Publish the pointcloud in output frame
        :param points: Nx3 ndarray of lidar points
        :param stamp: timestamp of the lidar scan message
        """

        # convert labeled points to PointCloud2 format
        data = unstructured_to_structured(points, dtype=np.dtype([
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32)
        ]))

        msg = msgify(PointCloud2, data)
        msg.header.stamp = stamp
        msg.header.frame_id = self.output_frame
        self.deskewed_pub.publish(msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('points_deskewer', log_level=rospy.INFO)
    node = PointsDeskewer()
    node.run()
