#!/usr/bin/env python3
import numpy as np
import rospy
import cv2
import message_filters
from autoware_msgs.msg import DetectedObjectArray, DetectedObject
from std_msgs.msg import ColorRGBA
from math import sqrt

GREEN = ColorRGBA(0.0, 1.0, 0.0, 0.8)

class LidarRadarFusion:
    def __init__(self):

        # Parameters
        self.matching_distance = rospy.get_param("~matching_distance") # radius threshold value around lidar centroid for a radar object to be considered matched
        self.radar_speed_threshold = rospy.get_param("~radar_speed_threshold")  # Threshold for filtering out stationary objects based on speed
        self.replace_speed_with_norm = rospy.get_param("~replace_speed_with_norm")  # Replace lidar and radar object speeds with norms in their respective x components

        # Subscribers and tf listeners
        radar_detections_sub = message_filters.Subscriber('radar/detected_objects', DetectedObjectArray, queue_size=1)
        lidar_detections_sub = message_filters.Subscriber('lidar/detected_objects', DetectedObjectArray, queue_size=1)

        # Sync
        ts = message_filters.ApproximateTimeSynchronizer([radar_detections_sub, lidar_detections_sub], queue_size=15, slop=0.05)
        ts.registerCallback(self.radar_lidar_data_callback)
        # Publishers
        self.detected_object_array_pub = rospy.Publisher('detected_objects', DetectedObjectArray, queue_size=1)

        rospy.loginfo(rospy.get_name().split('/')[-1] + " - Initialized")

    def radar_lidar_data_callback(self, radar_detections, lidar_detections):
        """
        :type radar_detections: DetectedObjectArray
        :type lidar_detections: DetectedObjectArray
        """
        radar_prepared, lidar_prepared = self.prepare_detections(radar_detections, lidar_detections)

        matches, within_hull_radar_ids = self.match_objects(radar_prepared, lidar_prepared)

        merged_objects = self.process_matches(matches, radar_prepared, lidar_prepared, within_hull_radar_ids)
        merged_objects.header = lidar_detections.header
        self.detected_object_array_pub.publish(merged_objects)

    def prepare_detections(self, radar_detections, lidar_detections):
        lidar_prepared = {}
        for lidar_detection in lidar_detections.objects:  # type: DetectedObject
            if self.replace_speed_with_norm:
                lidar_detection.velocity.linear.x = self.compute_norm(lidar_detection)
                lidar_detection.velocity.linear.y = 0
                lidar_detection.velocity.linear.z = 0
            lidar_prepared[lidar_detection.id] = lidar_detection

        radar_prepared = {}
        for radar_detection in radar_detections.objects:  # type: DetectedObject
            if self.replace_speed_with_norm:
                radar_detection.velocity.linear.x = self.compute_norm(radar_detection)
                radar_detection.velocity.linear.y = 0
                radar_detection.velocity.linear.z = 0
            radar_prepared[radar_detection.id] = radar_detection

        return radar_prepared, lidar_prepared

    def match_objects(self, radar_prepared, lidar_prepared):

        """
        radar_prepared: radar object dictionary containing DetectedObjects and their ids
        lidar_prepared: lidar object dictionary containing DetectedObjects and their ids
        return: matches and within_hull_radar_ids
        """

        matches = []
        within_hull_radar_ids = []

        for id_lidar, lidar_object in lidar_prepared.items():

            # Extracting lidar hull points
            lidar_hull = np.array([[hull_point.x, hull_point.y] for hull_point in lidar_object.convex_hull.polygon.points], dtype=np.float32) #unpacking geometry_msg/Point32 to float values

            min_radar_speed = 9999
            match_radar_id = None
            # For each radar object check if its centroid lies within the convex hull of the lidar object
            # or if the distance between the two centroids is smaller than the matching distance
            for id_radar, radar_object in radar_prepared.items():
                radar_object_centroid = (radar_object.pose.position.x, radar_object.pose.position.y)
                distance = self.compute_distance(radar_prepared[id_radar], lidar_prepared[id_lidar])

                # check if the radar object falls within(+1) or one the edge(0) of the lidar hull
                is_within_hull = cv2.pointPolygonTest(lidar_hull, radar_object_centroid, measureDist=False) >= 0
                if is_within_hull:
                    within_hull_radar_ids.append(id_radar)

                # check if matched
                if is_within_hull or distance < self.matching_distance:
                    # calculate norms of the radar object speed
                    radar_speed = self.compute_norm(radar_prepared[id_radar])
                    # match the radar object with the lowest speed
                    if radar_speed < min_radar_speed:
                        min_radar_speed = radar_speed
                        match_radar_id = id_radar

            if match_radar_id is not None:
                matches.append({"radar_id": match_radar_id, "lidar_id": id_lidar})

        return matches, within_hull_radar_ids

    def process_matches(self, matches, radar_prepared, lidar_prepared, within_hull_radar_ids):
        """
        :type matches: List of dict
        :type radar_prepared: Dict of DetectedObject
        :type lidar_prepared: Dict of DetectedObject
        """
        merged_objects = DetectedObjectArray()
        for match in matches:

            # merge all matched objects
            merged_object = self.merge_object(radar_prepared[match['radar_id']], lidar_prepared[match['lidar_id']])
            merged_objects.objects.append(merged_object)
            # delete the lidar obstacle that has already been merged since it has already
            # been added to the merged_objects array
            del lidar_prepared[match['lidar_id']]

        # commented out these lines fo now. Depends on our decision on what to do with unmatched lidar objects
        # these objects have zero speeds
        # We add unmatched lidar objects as fallback
        # for unmatched_lidar_object in lidar_prepared.values():
        #     merged_objects.objects.append(unmatched_lidar_object)

        # We add all moving radar objects falling outside lidar hulls  to merged objects
        for id_radar, radar_object in radar_prepared.items():
            # Do not add stationary radar objects, whose speed is less than threshold
            radar_speed = self.compute_norm(radar_object)
            if id_radar not in within_hull_radar_ids and radar_speed >= self.radar_speed_threshold:
                merged_objects.objects.append(radar_object)

        return merged_objects
    def merge_object(self, radar_object, lidar_object):
        """
        :type radar_object: DetectedObject
        :type lidar_object: DetectedObject
        """
        lidar_object.velocity = radar_object.velocity
        lidar_object.velocity_reliable = True
        lidar_object.acceleration = radar_object.acceleration
        lidar_object.acceleration_reliable = True
        lidar_object.color = GREEN
        return lidar_object

    @staticmethod
    def compute_distance(obj1, obj2):
        return sqrt((obj1.pose.position.x - obj2.pose.position.x)**2 + (obj1.pose.position.y - obj2.pose.position.y)**2)

    @staticmethod
    def compute_norm(object):
        return sqrt(object.velocity.linear.x**2 + object.velocity.linear.y**2 + object.velocity.linear.z**2)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('radar_lidar_fusion', anonymous=True, log_level=rospy.INFO)
    node = LidarRadarFusion()
    node.run()
