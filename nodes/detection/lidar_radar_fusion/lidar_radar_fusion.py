#!/usr/bin/env python3

import rospy
import copy
import message_filters

from visualization_msgs.msg import Marker, MarkerArray
from autoware_msgs.msg import DetectedObjectArray, DetectedObject
from geometry_msgs.msg import Vector3, Quaternion, Point
from std_msgs.msg import ColorRGBA
from genpy import Duration
from math import sqrt
from shapely.geometry import Polygon, Point

class LidarRadarFusion:
    def __init__(self):

        # Parameters
        radar_detections_topic = rospy.get_param("~in_radar_detections_topic", 'radar/detected_objects')
        lidar_detections_topic = rospy.get_param("~in_lidar_detections_topic", 'lidar/detected_objects')
        marker_array_topic = rospy.get_param("~out_marker_array_topic", 'detected_polygons')
        detected_object_array_topic = rospy.get_param("~out_tracked_objects", 'detected_objects')

        self.matching_distance = rospy.get_param("~matching_distance", 1.6) # radius threshold value aroud lidar centroid for a radar object to be considered matched
        self.radar_speed_margin = rospy.get_param("~radar_speed_margin", 2.78)  # Speed difference margin for fusing lidar with radar (in m/s)
        self.radar_speed_threshold = rospy.get_param("~radar_speed_threshold", 0.6)  # Threshold for filtering out stationary objects based on speed
        self.replace_speed_with_norm = rospy.get_param("~replace_speed_with_norm", True)  # Replace lidar and radar object speeds with norms in their respective x components
        self.map_frame = rospy.get_param("~map_frame", 'map')

        # Publishers
        self.marker_array_pub = rospy.Publisher(marker_array_topic, MarkerArray, queue_size=1)
        self.detected_object_array_pub = rospy.Publisher(detected_object_array_topic, DetectedObjectArray, queue_size=1)

        # Subscribers and tf listeners
        radar_detections_sub = message_filters.Subscriber(radar_detections_topic, DetectedObjectArray)
        lidar_detections_sub = message_filters.Subscriber(lidar_detections_topic, DetectedObjectArray)

        # Sync
        ts = message_filters.ApproximateTimeSynchronizer([radar_detections_sub, lidar_detections_sub],
                                                         queue_size=15, slop=0.05)
        ts.registerCallback(self.radar_lidar_data_callback)

        rospy.loginfo(self.__class__.__name__ + " - Initialized")
    def radar_lidar_data_callback(self, radar_detections, lidar_detections):
        """
        :type radar_detections: DetectedObjectArray
        :type lidar_detections: DetectedObjectArray
        """
        radar_prepared, lidar_prepared = self.prepare_detections(radar_detections, lidar_detections)

        matches, within_hull_radar_ids = self.match_objects(radar_prepared, lidar_prepared)

        merged_objects, viz_messages = self.process_matches(matches, radar_prepared, lidar_prepared, within_hull_radar_ids)
        merged_objects.header.stamp = lidar_detections.header.stamp

        self.detected_object_array_pub.publish(merged_objects)
        self.marker_array_pub.publish(viz_messages)

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
            lidar_hull = [(hull_point.x, hull_point.y) for hull_point in lidar_object.convex_hull.polygon.points] #unpacking geometry_msg/Point32 to float values

            # converting the convex hull to shapely Polygon for further processing
            shapely_lidar_hull = Polygon(lidar_hull)

            min_radar_speed = 9999
            match_radar_id = None
            # For each radar object check if its centroid lies within the convex hull of the lidar object
            # or if the distance between the two centroids is smaller than the matching distance
            for id_radar, radar_object in radar_prepared.items():

                radar_object_centroid = Point(radar_object.pose.position.x, radar_object.pose.position.y)
                distance = self.compute_distance(radar_prepared[id_radar], lidar_prepared[id_lidar])

                # check if the radar object falls within the lidar hull
                is_within_hull = shapely_lidar_hull.contains(radar_object_centroid)
                if is_within_hull:
                    within_hull_radar_ids.append(id_radar)

                # check if matched
                if is_within_hull or distance < self.matching_distance:
                    # calculate norms of lidar and radar object speeds
                    radar_speed = self.compute_norm(radar_prepared[id_radar])
                    lidar_speed = self.compute_norm(lidar_prepared[id_lidar])

                    # if radar object speed is less than lidar object speed up to margin
                    # or radar object speed is bigger than lidar object speed
                    # then out of such radar objects take the one with lowest speed
                    # HACK: comparing norms does not take into account movement direction,
                    #       but as objects are very close, we hope that it does not matter

                    if radar_speed >= lidar_speed - self.radar_speed_margin and radar_speed < min_radar_speed:
                        min_radar_speed = radar_speed
                        match_radar_id = id_radar

            if match_radar_id is not None:
                # We keep the radar obstacle with the min speed that is greater than lidar_speed - radar_speed_margin
                matches.append({"radar_id": match_radar_id, "lidar_id": id_lidar})

        return matches, within_hull_radar_ids

    def process_matches(self, matches, radar_prepared, lidar_prepared, within_hull_radar_ids):
        """
        :type matches: List of dict
        :type radar_prepared: Dict of DetectedObject
        :type lidar_prepared: Dict of DetectedObject
        """
        merged_objects = DetectedObjectArray()
        viz_messages = MarkerArray()
        for match in matches:

            # merge all matched objects
            merged_object = self.merge_object(radar_prepared[match['radar_id']], lidar_prepared[match['lidar_id']])
            merged_objects.objects.append(merged_object)
            viz_messages.markers += self.generate_rviz_marker(merged_object)
            # delete the lidar obstacle that has already been merged since it has already
            # been added to the merged_objects array
            del lidar_prepared[match['lidar_id']]

        # We add unmatched lidar objects as fallback
        for unmatched_lidar_object in lidar_prepared.values():
            # remove unmatched lidar objects with unreliable velocity
            if not unmatched_lidar_object.velocity_reliable:
                continue
            merged_objects.objects.append(unmatched_lidar_object)
            viz_messages.markers += self.generate_rviz_marker(unmatched_lidar_object)

        # We add all moving radar objects falling outside lidar hulls  to merged objects
        for id_radar, radar_object in radar_prepared.items():
            # Do not add stationary radar objects, whose speed is less than threshold
            radar_speed = self.compute_norm(radar_object)
            if id_radar not in within_hull_radar_ids and radar_speed >= self.radar_speed_threshold:
                merged_objects.objects.append(radar_object)
                viz_messages.markers += self.generate_rviz_marker(radar_object)

        return merged_objects, viz_messages

    def merge_object(self, radar_object, lidar_object):
        """
        :type radar_object: DetectedObject
        :type lidar_object: DetectedObject
        """
        lidar_object.velocity = radar_object.velocity
        lidar_object.velocity_reliable = True
        lidar_object.acceleration = radar_object.acceleration
        lidar_object.acceleration_reliable = True
        return lidar_object

    def generate_rviz_marker(self, detected_object):
        """
        :type detected_object: DetectedObject
        :returns Marker[]
        """
        marker_list = []

        centroid = Marker()
        centroid.header.frame_id = "map"
        centroid.ns = "fused_centroid"
        centroid.type = centroid.SPHERE
        centroid.action = centroid.ADD
        centroid.scale = Vector3(x=1., y=1., z=1.)
        centroid.pose.position = detected_object.pose.position
        centroid.pose.orientation = Quaternion(w=1., x=0., y=0., z=0.)
        centroid.id = detected_object.id
        centroid.lifetime = Duration(secs=0.1)

        hull = Marker()
        hull.header.frame_id = "map"
        hull.ns = "fused_hull"
        hull.type = hull.LINE_STRIP
        hull.action = hull.ADD
        hull.scale = Vector3(x=0.2, y=1., z=1.)
        hull.color = ColorRGBA(r=0.0, g=1.0, b=0.58, a=1.)
        hull.points = detected_object.convex_hull.polygon.points
        hull.id = detected_object.id
        hull.lifetime = Duration(secs=0.1)

        text = Marker()
        text.header.frame_id = "map"
        text.ns = "fused_text"
        text.type = text.TEXT_VIEW_FACING
        text.action = text.ADD
        text.scale = Vector3(x=0., y=0., z=1.)
        text.pose.position = copy.copy(detected_object.pose.position)
        text.pose.orientation = Quaternion(w=1., x=0., y=0., z=0.)
        text.id = detected_object.id + 1000
        text.lifetime = Duration(secs=0.1)

        #### Setting color properties of makers for easy debugging ###

        # Merged object has reliable acceleration and contains a pointcloud
        if detected_object.acceleration_reliable and detected_object.pointcloud.data:
            text.color = ColorRGBA(r=1., g=1., b=1., a=1.)
            text.pose.position.x += 1
            text.text = "M#%02d_(%05.2f)" % (detected_object.id, self.compute_norm(detected_object))  # Id & speed  # Id & speed
            centroid.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.)
        # Only radar object has reliable acceleration and no pointcloud data
        elif detected_object.acceleration_reliable and not detected_object.pointcloud.data:
            text.pose.position.x += 2
            text.color = ColorRGBA(r=1., g=0., b=0., a=1.)
            text.text = "R#%02d_(%05.2f)" % (detected_object.id, self.compute_norm(detected_object))  # Id & speed  # Id & speed
            centroid.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.)
        # Only lidar object has unreliable acceleration and contains a pointcloud
        elif detected_object.pointcloud.data:
            text.pose.position.x += 1
            text.color = ColorRGBA(r=0., g=1., b=0., a=1.)
            text.text = "L#%02d_(%05.2f)" % (detected_object.id, self.compute_norm(detected_object))  # Id & speed
            centroid.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.)

        marker_list += [centroid, text, hull]
        return marker_list

    @staticmethod
    def compute_distance(obj1, obj2):
        return sqrt((obj1.pose.position.x - obj2.pose.position.x)**2 + (obj1.pose.position.y - obj2.pose.position.y)**2)

    @staticmethod
    def compute_norm(object):
        return sqrt(object.velocity.linear.x**2 + object.velocity.linear.y**2 + object.velocity.linear.z**2)

    @staticmethod
    def run():
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('radar_lidar_fusion', anonymous=True, log_level=rospy.INFO)
    node = LidarRadarFusion()
    node.run()
