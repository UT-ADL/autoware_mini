#!/usr/bin/env python3

import numpy as np
import traceback
from scipy.optimize import linear_sum_assignment
from scipy.spatial.distance import cdist

import rospy
import message_filters

from autoware_msgs.msg import DetectedObjectArray
from std_msgs.msg import ColorRGBA

from helpers.geometry import get_distance_between_two_points_2d, get_vector_norm_3d
from helpers.detection import calculate_iou, get_axis_oriented_bounding_box


GREEN = ColorRGBA(0.0, 1.0, 0.0, 0.8)

class LidarRadarFusion:
    def __init__(self):

        # Parameters
        self.radar_speed_threshold = rospy.get_param("~radar_speed_threshold")  # Threshold for filtering out stationary objects based on speed
        self.association_method = rospy.get_param('~association_method',)
        self.max_euclidean_distance = rospy.get_param('~max_euclidean_distance')

        # Publisher
        self.detected_object_array_pub = rospy.Publisher('/detection/detected_objects', DetectedObjectArray, queue_size=1)

        # Subscribers
        radar_detections_sub = message_filters.Subscriber('/detection/radar/detected_objects', DetectedObjectArray, queue_size=1)
        lidar_detections_sub = message_filters.Subscriber('/detection/lidar/detected_objects', DetectedObjectArray, queue_size=1)

        # Sync
        ts = message_filters.ApproximateTimeSynchronizer([lidar_detections_sub, radar_detections_sub], queue_size=20, slop=0.06)
        ts.registerCallback(self.lidar_radar_callback)
        self.objects_array_dtype = [
                ('centroid', np.float32, (2,)),
                ('bbox', np.float32, (4,)),
                ]

        rospy.loginfo("%s - initialized", rospy.get_name())

    def lidar_radar_callback(self, lidar_detections, radar_detections):
        """
        radar_detections: DetectedObjectArray
        lidar_detections: DetectedObjectArray
        publish: DetectedObjectArray
        """
        try:
            final_detections = DetectedObjectArray()
            final_detections.header = lidar_detections.header
            max_id = 0

            lidar_detected_objects = lidar_detections.objects
            radar_detected_objects = radar_detections.objects
            lidar_objects_array = np.empty((len(lidar_detected_objects)), dtype=self.objects_array_dtype)
            radar_objects_array = np.empty((len(radar_detected_objects)), dtype=self.objects_array_dtype)

            # Collect lidar and radar object centroids and bboxes in arrays
            for i, lidar_object in enumerate(lidar_detected_objects):
                lidar_objects_array[i]['centroid'] = (lidar_object.pose.position.x, lidar_object.pose.position.y)
                lidar_objects_array[i]['bbox'] = get_axis_oriented_bounding_box(lidar_object)
                # keep track of the largest id
                if lidar_object.id > max_id:
                    max_id = lidar_object.id
            assert len(lidar_detected_objects) == len(lidar_objects_array)

            for i, radar_object in enumerate(radar_detected_objects):
                radar_objects_array[i]['centroid'] = (radar_object.pose.position.x, radar_object.pose.position.y)
                radar_objects_array[i]['bbox'] = get_axis_oriented_bounding_box(radar_object)
            assert len(radar_detected_objects) == len(radar_objects_array)

            if self.association_method == 'iou':
                # Calculate the IOU between the tracked objects and the detected objects
                iou = calculate_iou(lidar_objects_array['bbox'], radar_objects_array['bbox'])
                assert iou.shape == (len(lidar_objects_array), len(radar_objects_array))

                # Calculate the association between the tracked objects and the detected objects
                matched_lidar_indices, matched_radar_indices = linear_sum_assignment(-iou)
                assert len(matched_lidar_indices) == len(matched_radar_indices)

                # Only keep those matches where the IOU is greater than 0.0
                matches = iou[matched_lidar_indices, matched_radar_indices] > 0.0
                matched_lidar_indices = matched_lidar_indices[matches]
                matched_radar_indices = matched_radar_indices[matches]
                assert len(matched_lidar_indices) == len(matched_radar_indices)

            elif self.association_method == 'euclidean':
                # Calculate euclidean distance between the tracked object and the detected object centroids
                dists = cdist(lidar_objects_array['centroid'], radar_objects_array['centroid'])
                assert dists.shape == (len(lidar_objects_array), len(radar_objects_array))

                # Calculate the association between the tracked objects and the detected objects
                matched_lidar_indices, matched_radar_indices = linear_sum_assignment(dists)

                # Only keep those matches where the distance is less than threshold
                matches = dists[matched_lidar_indices, matched_radar_indices] <= self.max_euclidean_distance
                matched_lidar_indices = matched_lidar_indices[matches]
                matched_radar_indices = matched_radar_indices[matches]
                assert len(matched_lidar_indices) == len(matched_radar_indices)
            else:
                assert False, 'Unknown association method: ' + self.association_method

            # fuse matched detections
            for matched_lidar_index, matched_radar_index in zip(matched_lidar_indices, matched_radar_indices):
                radar_speed = get_vector_norm_3d(radar_detected_objects[matched_radar_index].velocity.linear)
                # filter out all stationary radar objects
                if radar_speed <= self.radar_speed_threshold:
                    continue

                lidar_detected_objects[matched_lidar_index].velocity = radar_detected_objects[matched_radar_index].velocity
                lidar_detected_objects[matched_lidar_index].velocity_reliable = True
                lidar_detected_objects[matched_lidar_index].acceleration = radar_detected_objects[matched_radar_index].acceleration
                lidar_detected_objects[matched_lidar_index].acceleration_reliable = True
                lidar_detected_objects[matched_lidar_index].color = GREEN

            # Add all lidar objects (fused and unfused) to final objects
            final_detections.objects = lidar_detected_objects
            # add radar detections that are not matched to final detections
            for i, radar_detection in enumerate(radar_detected_objects):
                if i not in matched_radar_indices:
                    # calculate norm of radar detection's speed
                    radar_speed = get_vector_norm_3d(radar_detection.velocity.linear)

                    # add only moving radar objects
                    if radar_speed >= self.radar_speed_threshold:
                        # generate new id for the radar object
                        max_id += 1
                        radar_detection.id = max_id
                        final_detections.objects.append(radar_detection)

            self.detected_object_array_pub.publish(final_detections)

        except Exception as e:
            rospy.logerr_throttle(10, "%s - Exception in callback: %s", rospy.get_name(), traceback.format_exc())

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('lidar_radar_fusion', log_level=rospy.INFO)
    node = LidarRadarFusion()
    node.run()
