#!/usr/bin/env python3

import numpy as np
import rospy
import cv2
import message_filters
from autoware_msgs.msg import DetectedObjectArray
from std_msgs.msg import ColorRGBA
from math import sqrt

GREEN = ColorRGBA(0.0, 1.0, 0.0, 0.8)

class LidarRadarFusion:
    def __init__(self):

        # Parameters
        self.matching_distance = rospy.get_param("~matching_distance") # radius threshold value around lidar centroid for a radar object to be considered matched
        self.radar_speed_threshold = rospy.get_param("~radar_speed_threshold")  # Threshold for filtering out stationary objects based on speed

        # Publisher
        self.detected_object_array_pub = rospy.Publisher('detected_objects', DetectedObjectArray, queue_size=1)

        # Subscribers
        radar_detections_sub = message_filters.Subscriber('radar/detected_objects', DetectedObjectArray, queue_size=1)
        lidar_detections_sub = message_filters.Subscriber('lidar/detected_objects', DetectedObjectArray, queue_size=1)

        # Sync
        ts = message_filters.ApproximateTimeSynchronizer([radar_detections_sub, lidar_detections_sub], queue_size=15, slop=0.05)
        ts.registerCallback(self.lidar_radar_callback)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def lidar_radar_callback(self, radar_detections, lidar_detections):
        """
        radar_detections: DetectedObjectArray
        lidar_detections: DetectedObjectArray
        publish: DetectedObjectArray
        """
        final_detections = DetectedObjectArray()
        final_detections.header = lidar_detections.header

        for lidar_detection in lidar_detections.objects:
            # Extracting lidar hull points
            lidar_hull = np.array([(hull_point.x, hull_point.y) for hull_point in lidar_detection.convex_hull.polygon.points], dtype=np.float32) #unpacking geometry_msg/Point32 to float values

            min_radar_speed = np.inf
            matched_radar_detection = None
            # For each radar object check if its centroid lies within the convex hull of the lidar object
            # or if the distance between the two centroids is smaller than the self.maching_distance param
            for radar_detection in radar_detections.objects:
                radar_object_centroid = (radar_detection.pose.position.x, radar_detection.pose.position.y)
                distance = self.compute_distance(lidar_detection, radar_detection)

                # calculate norm of radar detection's speed
                radar_speed = self.compute_norm(radar_detection.velocity.linear)
                # check if the radar object falls within(+1) or one the edge(0) of the lidar hull. Side note: outside of hull = -1
                is_within_hull = cv2.pointPolygonTest(lidar_hull, radar_object_centroid, measureDist=False) >= 0

                # Add all moving radar objects falling outside lidar hulls  to final objects
                if not is_within_hull and radar_speed >= self.radar_speed_threshold:
                    final_detections.objects.append(radar_detection)

                # check if matched
                if is_within_hull or distance < self.matching_distance:
                    # match the radar object with the lowest speed
                    if radar_speed < min_radar_speed:
                        min_radar_speed = radar_speed
                        matched_radar_detection = radar_detection

            if matched_radar_detection is not None:
                # if a match found, fuse detections and publish the fused one
                lidar_detection.velocity = matched_radar_detection.velocity
                lidar_detection.velocity_reliable = True
                lidar_detection.acceleration = matched_radar_detection.acceleration
                lidar_detection.acceleration_reliable = True
                lidar_detection.color = GREEN
                final_detections.objects.append(lidar_detection)
            else:
                # if unmatched, publish unfused lidar detection
                final_detections.objects.append(lidar_detection)

        self.detected_object_array_pub.publish(final_detections)

    @staticmethod
    def compute_distance(obj1, obj2):
        return sqrt((obj1.pose.position.x - obj2.pose.position.x)**2 + (obj1.pose.position.y - obj2.pose.position.y)**2)

    @staticmethod
    def compute_norm(obj_velocity):
        return sqrt(obj_velocity.x**2 + obj_velocity.y**2 + obj_velocity.z**2)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('lidar_radar_fusion', log_level=rospy.INFO)
    node = LidarRadarFusion()
    node.run()
