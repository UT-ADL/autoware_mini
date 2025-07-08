#!/usr/bin/env python3

import rospy
import message_filters

import numpy as np
import traceback

from scipy.spatial.distance import cdist
from lapsolver import solve_dense

from autoware_msgs.msg import DetectedObjectArray
from std_msgs.msg import ColorRGBA

from helpers.geometry import get_vector_norm_3d
from helpers.detection import calculate_iou, get_axis_oriented_bounding_box


GREEN = ColorRGBA(0.0, 1.0, 0.0, 0.8)

class LidarRadarFusion:
    def __init__(self):

        # Parameters
        self.radar_speed_threshold = rospy.get_param("~radar_speed_threshold")  # Threshold for filtering out stationary objects based on speed
        self.association_method = rospy.get_param('~association_method',)
        self.max_euclidean_distance = rospy.get_param('~max_euclidean_distance')

        # Publisher
        self.detected_object_array_pub = rospy.Publisher('detected_objects', DetectedObjectArray, queue_size=1, tcp_nodelay=True)

        # Subscribers
        radar_detections_sub = message_filters.Subscriber('radar/detected_objects', DetectedObjectArray, queue_size=1, buff_size=2**20, tcp_nodelay=True)
        lidar_detections_sub = message_filters.Subscriber('lidar/detected_objects', DetectedObjectArray, queue_size=1, buff_size=2**20, tcp_nodelay=True)

        # Sync
        ts = message_filters.ApproximateTimeSynchronizer([lidar_detections_sub, radar_detections_sub], queue_size=20, slop=0.06)
        ts.registerCallback(self.lidar_radar_callback)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def lidar_radar_callback(self, lidar_detections, radar_detections):
        """
        radar_detections: DetectedObjectArray
        lidar_detections: DetectedObjectArray
        publish: DetectedObjectArray
        """
        try:
            if self.association_method == 'iou':
                # Collect the axis oriented bounding boxes for the lidar objects and the radar objects
                lidar_objects_bboxes = np.array([get_axis_oriented_bounding_box(obj) for obj in lidar_detections.objects], dtype=np.float32)
                radar_objects_bboxes = np.array([get_axis_oriented_bounding_box(obj) for obj in radar_detections.objects], dtype=np.float32)

                # Calculate the IOU between the tracked objects and the detected objects
                iou = calculate_iou(lidar_objects_bboxes, radar_objects_bboxes)
                assert iou.shape == (len(lidar_detections.objects), len(radar_detections.objects))

                # Don't allow pairing where IOU is 0
                iou[iou <= 0.0] = np.nan

                # Calculate the association between the tracked objects and the detected objects
                matched_lidar_indices, matched_radar_indices = solve_dense(-iou)
                assert len(matched_lidar_indices) == len(matched_radar_indices)
            elif self.association_method == 'euclidean':
                # Collect centroids for the lidar objects and the radar objects
                lidar_objects_centroids = np.array([(obj.pose.position.x, obj.pose.position.y) for obj in lidar_detections.objects], dtype=np.float32)
                radar_objects_centroids = np.array([(obj.pose.position.x, obj.pose.position.y) for obj in radar_detections.objects], dtype=np.float32)

                # Calculate euclidean distance between the tracked object and the detected object centroids
                dists = cdist(lidar_objects_centroids, radar_objects_centroids)
                assert dists.shape == (len(lidar_detections.objects), len(radar_detections.objects))

                # Calculate the association between the tracked objects and the detected objects but with the following constraint:
                # don't allow pairing for elements with a distance value greater than self.max_euclidean_distance
                dists[dists > self.max_euclidean_distance] = np.nan
                matched_lidar_indices, matched_radar_indices = solve_dense(dists)
                assert len(matched_lidar_indices) == len(matched_radar_indices)
            else:
                assert False, 'Unknown association method: ' + self.association_method

            # fuse matched detections
            for matched_lidar_index, matched_radar_index in zip(matched_lidar_indices, matched_radar_indices):
                lidar_detections.objects[matched_lidar_index].velocity = radar_detections.objects[matched_radar_index].velocity
                lidar_detections.objects[matched_lidar_index].velocity_reliable = True
                lidar_detections.objects[matched_lidar_index].acceleration = radar_detections.objects[matched_radar_index].acceleration
                lidar_detections.objects[matched_lidar_index].acceleration_reliable = True
                lidar_detections.objects[matched_lidar_index].color = GREEN

            # Add all lidar objects (fused and unfused) to final objects
            final_detections = DetectedObjectArray()
            final_detections.header = lidar_detections.header
            final_detections.objects = lidar_detections.objects

            # Find the maximum id of the lidar detections
            max_id = 0
            for obj in final_detections.objects:
                if obj.id > max_id:
                    max_id = obj.id

            # add radar detections that are not matched to final detections
            for i, radar_detection in enumerate(radar_detections.objects):
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
