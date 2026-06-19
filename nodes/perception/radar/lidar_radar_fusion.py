#!/usr/bin/env python3

import rospy
import message_filters

import numpy as np
import traceback

import scipy.spatial.distance

from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import Imu
from autoware_mini.msg import DetectedObjectArray

from autoware_mini.geometry import get_speed_from_velocity

VIOLET = ColorRGBA(0.8, 0.0, 1.0, 0.8)

class LidarRadarFusion:
    def __init__(self):

        # Parameters
        self.max_euclidean_distance = rospy.get_param('~max_euclidean_distance')
        self.angular_velocity_threshold = rospy.get_param("~angular_velocity_threshold")
        self.radar_speed_threshold = rospy.get_param("~radar_speed_threshold")
        self.synchronizer_queue_size = rospy.get_param('~synchronizer_queue_size')
        self.synchronizer_slop = rospy.get_param('~synchronizer_slop')

        # Internal variables
        self.current_angular_velocity = 0.0

        # Publisher
        self.detected_object_array_pub = rospy.Publisher('detected_objects', DetectedObjectArray, queue_size=1, tcp_nodelay=True)

        # Subscribers
        radar_detections_sub = message_filters.Subscriber('radar/detected_objects', DetectedObjectArray, queue_size=1, buff_size=2**20, tcp_nodelay=True)
        lidar_detections_sub = message_filters.Subscriber('lidar/detected_objects', DetectedObjectArray, queue_size=1, buff_size=2**20, tcp_nodelay=True)
        rospy.Subscriber('/imu/data_raw', Imu, self.imu_callback, queue_size=1, tcp_nodelay=True)

        # Sync
        ts = message_filters.ApproximateTimeSynchronizer([lidar_detections_sub, radar_detections_sub], queue_size=self.synchronizer_queue_size, slop=self.synchronizer_slop)
        ts.registerCallback(self.lidar_radar_callback)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def imu_callback(self, msg):
        self.current_angular_velocity = msg.angular_velocity.z

    def lidar_radar_callback(self, lidar_detections, radar_detections):
        """
        radar_detections: DetectedObjectArray
        lidar_detections: DetectedObjectArray
        publish: DetectedObjectArray
        """
        try:
            # Only do fusion if the vehicle is not turning too much
            if abs(self.current_angular_velocity) > self.angular_velocity_threshold:
                self.detected_object_array_pub.publish(lidar_detections)
                return

            lidar_objects = lidar_detections.objects  # alias — appends mutate the outgoing message
            # Ignore stationary radar objects — they don't participate in fusion or get published
            radar_objects = [obj for obj in radar_detections.objects if get_speed_from_velocity(obj.velocity) > self.radar_speed_threshold]

            matched_radar_indices = []

            if lidar_objects and radar_objects:

                # Collect centroids for the lidar objects and the radar objects
                lidar_objects_centroids = np.array([(obj.centroid.x, obj.centroid.y) for obj in lidar_objects], dtype=np.float32)
                radar_objects_centroids = np.array([(obj.centroid.x, obj.centroid.y) for obj in radar_objects], dtype=np.float32)

                # Calculate euclidean distance between the tracked object and the detected object centroids
                dists = scipy.spatial.distance.cdist(lidar_objects_centroids, radar_objects_centroids)
                assert dists.shape == (len(lidar_objects), len(radar_objects))

                # Calculate one-to-many association between the radar detections and lidar detections with the following constraint:
                # don't allow pairing for elements with a distance value greater than self.max_euclidean_distance
                min_idx = np.argmin(dists, axis=1)
                min_dists = dists[np.arange(dists.shape[0]), min_idx]
                matched_lidar_indices = np.nonzero(min_dists < self.max_euclidean_distance)[0]
                matched_radar_indices = min_idx[matched_lidar_indices]
                assert len(matched_lidar_indices) == len(matched_radar_indices)

                # fuse matched detections: propagate any field the radar source vouches for
                for matched_lidar_index, matched_radar_index in zip(matched_lidar_indices, matched_radar_indices):
                    lidar_obj = lidar_objects[matched_lidar_index]
                    radar_obj = radar_objects[matched_radar_index]
                    if radar_obj.velocity_reliable:
                        lidar_obj.velocity = radar_obj.velocity
                        lidar_obj.velocity_reliable = True
                    if radar_obj.acceleration_reliable:
                        lidar_obj.acceleration = radar_obj.acceleration
                        lidar_obj.acceleration_reliable = True
                    lidar_obj.position_reliable = True
                    lidar_obj.color = VIOLET

            # add unmatched non-stationary radar detections to final detections
            matched_radar_set = set(matched_radar_indices)
            for i, radar_obj in enumerate(radar_objects):
                if i not in matched_radar_set:
                    lidar_objects.append(radar_obj)

            self.detected_object_array_pub.publish(lidar_detections)

        except Exception as e:
            rospy.logerr_throttle(10, "%s - Exception in callback: %s", rospy.get_name(), traceback.format_exc())

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('lidar_radar_fusion', log_level=rospy.INFO)
    node = LidarRadarFusion()
    node.run()
