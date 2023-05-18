#!/usr/bin/env python3

from math import sqrt
import numpy as np
import traceback
import cv2

import rospy
import message_filters

from autoware_msgs.msg import DetectedObjectArray
from std_msgs.msg import ColorRGBA

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
        ts = message_filters.ApproximateTimeSynchronizer([lidar_detections_sub, radar_detections_sub], queue_size=2, slop=0.04)
        ts.registerCallback(self.lidar_radar_callback)

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
            used_radar_detections = {}
            max_id = 0

            for lidar_detection in lidar_detections.objects:
                # Extracting lidar hull points
                hull_points = [(hull_point.x, hull_point.y) for hull_point in lidar_detection.convex_hull.polygon.points]
                lidar_hull = np.array(hull_points, dtype=np.float32)

                min_radar_speed = np.inf
                matched_radar_detection = None
                # For each radar object check if its centroid lies within the convex hull of the lidar object
                # or if the distance between the two centroids is smaller than the self.maching_distance param
                for radar_detection in radar_detections.objects:
                    # calculate norm of radar detection's speed
                    radar_speed = self.compute_norm(radar_detection.velocity.linear)
                    # filter out all stationary radar objects
                    if radar_speed <= self.radar_speed_threshold:
                        continue
                    # check if the radar object falls within(+1) or one the edge(0) of the lidar hull. Side note: outside of hull = -1
                    radar_object_centroid = (radar_detection.pose.position.x, radar_detection.pose.position.y)
                    is_within_hull = cv2.pointPolygonTest(lidar_hull, radar_object_centroid, measureDist=False) >= 0

                    # calculate distance between lidar and radar objects
                    distance = self.compute_distance(lidar_detection, radar_detection)

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
                    # record matched radar detections
                    used_radar_detections[matched_radar_detection.id] = True

                # keep track of the largest id
                if lidar_detection.id > max_id:
                    max_id = lidar_detection.id

                # lidar detections are always published
                final_detections.objects.append(lidar_detection)

            # add radar detections that are not matched to final detections
            for radar_detection in radar_detections.objects:
                if radar_detection.id not in used_radar_detections:
                    # calculate norm of radar detection's speed
                    radar_speed = self.compute_norm(radar_detection.velocity.linear)

                    # add only moving radar objects
                    if radar_speed >= self.radar_speed_threshold:
                        # generate new id for the radar object
                        max_id += 1
                        radar_detection.id = max_id
                        final_detections.objects.append(radar_detection)

            self.detected_object_array_pub.publish(final_detections)

        except Exception as e:
            rospy.logerr_throttle(10, "%s - Exception in callback: %s", rospy.get_name(), traceback.format_exc())

    @staticmethod
    def compute_distance(obj1, obj2):
        return sqrt((obj1.pose.position.x - obj2.pose.position.x)**2 + (obj1.pose.position.y - obj2.pose.position.y)**2)

    @staticmethod
    def compute_norm(vec):
        return sqrt(vec.x**2 + vec.y**2 + vec.z**2)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('lidar_radar_fusion', log_level=rospy.INFO)
    node = LidarRadarFusion()
    node.run()
