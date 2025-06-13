#!/usr/bin/env python3

import rospy
import json
import shapely
import numpy as np
from autoware_mini.msg import DetectedObjectArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
from autoware_mini.geometry import get_distance_between_two_points_2d, convert_geometry_to_line_list
from autoware_mini.lanelet2 import utm_origin


class RoadAreaFilter:
    def __init__(self):

        # get parameters
        self.road_area_file_path = rospy.get_param("~road_area_file_path")
        self.filtering_method = rospy.get_param("~filtering_method")
        self.use_map_extraction = rospy.get_param("~use_map_extraction")
        self.map_extraction_distance = rospy.get_param("~map_extraction_distance")
        self.local_path_length = rospy.get_param("/planning/local_path_length")

        if self.filtering_method not in ["centroid", "intersects", "within"]:
            raise ValueError(f"{rospy.get_name()} - 'filtering_method' must be one of 'centroid', 'intersects' or 'within', not '{self.filtering_method}'")

        self.map_extraction_location = None
        self.road_area_data = None
        self.road_area = None
        self.not_road_area = None

        rospy.loginfo("%s - loading road area from file %s", rospy.get_name(), self.road_area_file_path)
        easting, northing = utm_origin()

        # Read the GeoJSON file and create shapely geometries
        road_area_data = []
        with open(self.road_area_file_path, 'r') as f:
            self.geojson_data = json.load(f)
            for feature in self.geojson_data['features']:
                geometry = shapely.geometry.shape(feature['geometry'])
                road_area_data.append(geometry)
        road_area_data = shapely.unary_union(road_area_data)
        road_area_data = shapely.affinity.translate(road_area_data, xoff=-easting, yoff=-northing)
        shapely.prepare(road_area_data)
        self.road_area_data = road_area_data

        # detected objects publisher
        self.objects_pub = rospy.Publisher('detected_objects_filtered', DetectedObjectArray, queue_size=1, tcp_nodelay=True)
        self.road_area_pub = rospy.Publisher('road_area_markers', MarkerArray, queue_size=1, tcp_nodelay=True, latch=True)

        # Subscribers
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('detected_objects', DetectedObjectArray, self.detected_objects_callback, queue_size=1, tcp_nodelay=True)

        if not self.use_map_extraction:
            self.road_area_pub.publish(self.get_road_area_markers(self.road_area_data))

        rospy.loginfo("%s - initialized", rospy.get_name())

    def get_road_area_markers(self, road_area):

        road_area_markers = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "Road area"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 0.8
        marker.color.r = 0.9
        marker.color.g = 0.1
        marker.color.b = 0.1

        # in case of MultiPolygon the Polygons are listed in geoms. If no geoms, it is a single Polygon
        if hasattr(road_area, 'geoms'):
            road_area = road_area.geoms
        else:
            road_area = [road_area]

        for geom in road_area:
            if geom.is_empty:
                continue
            marker.points.extend(convert_geometry_to_line_list(geom.exterior.coords))
            if hasattr(geom, 'interiors'):
                for interior in geom.interiors:
                    marker.points.extend(convert_geometry_to_line_list(interior.coords))

        road_area_markers.markers.append(marker)
        return road_area_markers

    def current_pose_callback(self, msg):

        current_location = shapely.Point(msg.pose.position.x, msg.pose.position.y)

        if self.map_extraction_location is None or get_distance_between_two_points_2d(self.map_extraction_location, msg.pose.position) >= (self.map_extraction_distance - self.local_path_length):
            self.map_extraction_location = msg.pose.position
            map_extent_box = shapely.box(current_location.x - self.map_extraction_distance, current_location.y - self.map_extraction_distance, current_location.x + self.map_extraction_distance, current_location.y + self.map_extraction_distance)
            road_area = map_extent_box.intersection(self.road_area_data)
            shapely.prepare(road_area)
            self.road_area = road_area

            # create inverted road area
            if self.filtering_method == "within":
                not_road_area = map_extent_box.difference(road_area)
                shapely.prepare(not_road_area)
                self.not_road_area = not_road_area

            if self.use_map_extraction:
                self.road_area_pub.publish(self.get_road_area_markers(road_area))

    def detected_objects_callback(self, msg):

        if self.road_area is None:
            rospy.logwarn_throttle(3, "%s - road area not received!", rospy.get_name())
            return

        # Create detected objects array
        detected_objects = DetectedObjectArray()
        detected_objects.header = msg.header

        for obj in msg.objects:
            if self.filtering_method == "centroid":
                obj_geom = shapely.Point(obj.centroid.x, obj.centroid.y)
            else:
                obj_geom = shapely.polygons(np.array(obj.convex_hull).reshape(-1, 3))

            if self.filtering_method == "centroid" or self.filtering_method == "intersects":
                if self.road_area.intersects(obj_geom):
                    detected_objects.objects.append(obj)
            elif self.filtering_method == "within":
                if not self.not_road_area.intersects(obj_geom):
                    detected_objects.objects.append(obj)
            else:
                assert False, f"Unknown filtering method {self.filtering_method}"

        self.objects_pub.publish(detected_objects)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('road_area_filter', log_level=rospy.INFO)
    node = RoadAreaFilter()
    node.run()
