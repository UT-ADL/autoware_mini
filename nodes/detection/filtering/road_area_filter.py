#!/usr/bin/env python3

import rospy
import json

from autoware_msgs.msg import DetectedObjectArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point        

from shapely.geometry import shape
from shapely.affinity import translate
from shapely.ops import unary_union
from shapely import prepare, Polygon, Point as ShapelyPoint
from localization.WGS84ToUTMTransformer import WGS84ToUTMTransformer

class RoadAreaFilter:
    def __init__(self):

        # get parameters
        self.road_area_file = rospy.get_param("~road_area_file")
        self.use_centroid_filtering = rospy.get_param("~use_centroid_filtering")
        self.coordinate_transformer = rospy.get_param("/localization/coordinate_transformer")
        self.utm_origin_lat = rospy.get_param("/localization/utm_origin_lat")
        self.utm_origin_lon = rospy.get_param("/localization/utm_origin_lon")

        # initialize coordinate_transformer
        if self.coordinate_transformer == "utm":
            self.transformer = WGS84ToUTMTransformer(False, self.utm_origin_lat, self.utm_origin_lon)
        easting, northing = self.transformer.transform_lat_lon(self.utm_origin_lat, self.utm_origin_lon, 0)

        rospy.loginfo("%s - loading road area from file %s", rospy.get_name(), self.road_area_file)

        # Read the GeoJSON file
        with open(self.road_area_file, 'r') as f:
            geojson_data = json.load(f)

        self.road_area = []
        for feature in geojson_data['features']:
            geometry = shape(feature['geometry'])
            geometry = translate(geometry, xoff=-easting, yoff=-northing)
            self.road_area.append(geometry)
        prepare(self.road_area)

        # detected objects publisher
        self.objects_pub = rospy.Publisher('detected_objects', DetectedObjectArray, queue_size=1, tcp_nodelay=True)

        self.road_area_pub = rospy.Publisher('road_area', MarkerArray, queue_size=1, tcp_nodelay=True, latch=True)
        self.road_area_pub.publish(self.get_road_area_markers())

        # Subscribers
        rospy.Subscriber('detected_objects_unfiltered', DetectedObjectArray, self.detected_objects_callback, queue_size=1, tcp_nodelay=True)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def get_road_area_markers(self):
        boundary = unary_union(self.road_area)
        geometry = []
        for geom in boundary.geoms:
            geometry.append(geom.exterior.coords)
            for interior in geom.interiors:
                geometry.append(interior.coords)

        road_area_markers = MarkerArray()
        for i, polygon in enumerate(geometry):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "Road area"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.9
            marker.color.g = 0.1
            marker.color.b = 0.1
    
            for x, y, z in polygon:
                p = Point(x, y, z)
                marker.points.append(p)

            road_area_markers.markers.append(marker)

        return road_area_markers


    def detected_objects_callback(self, msg):

        # Create array objects
        objects = DetectedObjectArray()
        objects.header = msg.header

        for obj in msg.objects:
            if self.use_centroid_filtering:
                obj_geom = ShapelyPoint(obj.pose.position.x, obj.pose.position.y)
            else:
                obj_geom = Polygon([(p.x, p.y) for p in obj.convex_hull.polygon.points])
            prepare(obj_geom)
            if obj_geom.intersects(self.road_area).any():
                objects.objects.append(obj)

        self.objects_pub.publish(objects)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('road_area_filter', log_level=rospy.INFO)
    node = RoadAreaFilter()
    node.run()
