#!/usr/bin/env python3

import rospy
import shapely
import numpy as np
from datetime import datetime, timezone

from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import ColorRGBA
import lanelet2
from autoware_mini.lanelet2 import load_lanelet2_map
from autoware_mini.geometry import get_distance_between_two_points_2d, convert_geometry_to_line_list
from autoware_mini.visualization import triangulate_linestring, triangulate_polygon


# used for traffic lights
RED = ColorRGBA(1.0, 0.0, 0.0, 0.8)
YELLOW = ColorRGBA(1.0, 1.0, 0.0, 0.8)
GREEN = ColorRGBA(0.0, 1.0, 0.0, 0.8)

# colors for other map features
LIGHT_GREY = ColorRGBA(0.6, 0.6, 0.6, 0.6)
DARK_GREY = ColorRGBA(0.4, 0.4, 0.4, 0.6)
ORANGE = ColorRGBA(1.0, 0.5, 0.0, 0.6)
WHITE = ColorRGBA(1.0, 1.0, 1.0, 0.6)
CYAN = ColorRGBA(0.0, 1.0, 1.0, 0.6)
PINK = ColorRGBA(1.0, 0.0, 0.7, 0.6)
INDIGO = ColorRGBA(0.6, 0.0, 1.0, 0.6)
LIGHT_BLUE = ColorRGBA(0.0, 0.7, 0.9, 0.6)
DARK_BLUE = ColorRGBA(0.3, 0.3, 1.0, 0.6)

INDEX_TO_MARKER_COLOR = {
    0: RED,
    1: YELLOW,
    2: GREEN,
}

class Lanelet2MapVisualizer:

    def __init__(self):
    
        # Parameters
        lanelet2_map_path = rospy.get_param("~lanelet2_map_path")
        self.local_path_length = rospy.get_param("local_path_length")
        self.map_extraction_distance = rospy.get_param("~map_extraction_distance")
        self.use_map_extraction = rospy.get_param("~use_map_extraction")
        self.enable_road_closures = rospy.get_param("~enable_road_closures")

        self.map_extraction_location = None
        self.lanelet2_map = load_lanelet2_map(lanelet2_map_path)

        # Special publishers for stop line markers: traffic_lights and yielding
        self.lanelet2_map_markers_pub = rospy.Publisher('lanelet2_map_markers', MarkerArray, queue_size=1, latch=True, tcp_nodelay=True)

        if self.use_map_extraction:
            rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1, tcp_nodelay=True)
        else:
            lanelet_markers = self.visualize_laneletLayer(self.lanelet2_map.laneletLayer)
            linestring_markers = self.visualize_lineStringLayer(self.lanelet2_map.lineStringLayer)
            reg_el_markers = self.visualize_regulatoryElementLayer(self.lanelet2_map.regulatoryElementLayer)
            polygon_markers = self.visualize_polygonLayer(self.lanelet2_map.polygonLayer)
            self.marker_array = MarkerArray()
            self.marker_array.markers = lanelet_markers.markers + linestring_markers.markers + reg_el_markers.markers + polygon_markers.markers
            self.lanelet2_map_markers_pub.publish(self.marker_array)
            # Republish on bag replay wraparound
            self.prev_time = rospy.Time.now()
            rospy.Timer(rospy.Duration(1.0), self.republish_on_wraparound)

        rospy.loginfo("%s - map loaded with %i lanelets and %i regulatory elements from file: %s", rospy.get_name(),
                      len(self.lanelet2_map.laneletLayer), len(self.lanelet2_map.regulatoryElementLayer), lanelet2_map_path)

    def current_pose_callback(self, msg):

        if self.map_extraction_location is None or get_distance_between_two_points_2d(self.map_extraction_location, msg.pose.position) > (self.map_extraction_distance - 2*self.local_path_length):
            self.map_extraction_location = lanelet2.core.BasicPoint2d(msg.pose.position.x, msg.pose.position.y)
            search_box = lanelet2.core.BoundingBox2d(lanelet2.core.BasicPoint2d(msg.pose.position.x - self.map_extraction_distance, msg.pose.position.y - self.map_extraction_distance),
                                        lanelet2.core.BasicPoint2d(msg.pose.position.x + self.map_extraction_distance, msg.pose.position.y + self.map_extraction_distance))

            filtered_lanelets = self.lanelet2_map.laneletLayer.search(search_box)
            filtered_linestrings = self.lanelet2_map.lineStringLayer.search(search_box)
            filtered_regulatory_elements = self.lanelet2_map.regulatoryElementLayer.search(search_box)
            filtered_polygons = self.lanelet2_map.polygonLayer.search(search_box)

            # Visualize different parts of the map
            lanelet_markers = self.visualize_laneletLayer(filtered_lanelets)
            linestring_markers = self.visualize_lineStringLayer(filtered_linestrings)
            reg_el_markers = self.visualize_regulatoryElementLayer(filtered_regulatory_elements)
            polygon_markers = self.visualize_polygonLayer(filtered_polygons)

           # conactenate the MarkerArrays with delete all at front
            marker_array = MarkerArray()
            marker = Marker()
            marker.action = Marker.DELETEALL
            marker_array.markers = [marker] + lanelet_markers.markers + linestring_markers.markers + reg_el_markers.markers + polygon_markers.markers

            self.lanelet2_map_markers_pub.publish(marker_array)

    def republish_on_wraparound(self, event):
        new_time = rospy.Time.now()
        if self.prev_time > new_time:
            self.lanelet2_map_markers_pub.publish(self.marker_array)
        self.prev_time = new_time

    def run(self):
        rospy.spin()

    def visualize_laneletLayer(self, lanelets):
        stamp = rospy.Time.now()

        # Create a MarkerArray
        marker_array = MarkerArray()

        left_boundary_points = []
        right_boundary_points = []
        centerline_points = []
        bus_lane_points = []
        bicycle_lane_points = []
        crosswalk_points = []
        right_of_way_lanelet_points = []

        for lanelet in lanelets:
            if lanelet.attributes["subtype"] == "road" or lanelet.attributes["subtype"] == "bus_lane" or lanelet.attributes["subtype"] == "bicycle_lane":
                # Visualize left and right boundaries
                left_boundary_points.extend(convert_geometry_to_line_list(lanelet.leftBound))
                right_boundary_points.extend(convert_geometry_to_line_list(lanelet.rightBound))

                centerline = shapely.linestrings([[p.x, p.y, p.z] for p in lanelet.centerline])

                # triangulate centerline
                if lanelet.attributes["subtype"] == "road":
                    centerline_points.extend(triangulate_linestring(centerline, 1.5))
                elif lanelet.attributes["subtype"] == "bus_lane":
                    bus_lane_points.extend(triangulate_linestring(centerline, 1.5))
                elif lanelet.attributes["subtype"] == "bicycle_lane":
                    bicycle_lane_points.extend(triangulate_linestring(centerline, 0.8))

            elif lanelet.attributes["subtype"] == "crosswalk":
                crosswalk_border = get_polygon_from_lanelet(lanelet)
                crosswalk_points.extend(triangulate_polygon(crosswalk_border))

            elif lanelet.attributes["subtype"] == "right_of_way":
                right_of_way_lanelet_border = get_polygon_from_lanelet(lanelet)
                right_of_way_lanelet_points.extend(triangulate_polygon(right_of_way_lanelet_border))

        # skip markers without points, RViz reports them as errors
        if left_boundary_points:
            marker_array.markers.append(linelist_to_marker(left_boundary_points, "Left boundary", 0, DARK_GREY, 0.1, stamp))
        if right_boundary_points:
            marker_array.markers.append(linelist_to_marker(right_boundary_points, "Right boundary", 0, DARK_GREY, 0.1, stamp))
        if centerline_points:
            marker_array.markers.append(triangles_to_marker(centerline_points, "Centerline", 0, CYAN, 1.0, stamp))
        if bus_lane_points:
            marker_array.markers.append(triangles_to_marker(bus_lane_points, "Bus lane", 0, DARK_BLUE, 1.0, stamp))
        if bicycle_lane_points:
            marker_array.markers.append(triangles_to_marker(bicycle_lane_points, "Bicycle lane", 0, LIGHT_BLUE, 1.0, stamp))
        if crosswalk_points:
            marker_array.markers.append(triangles_to_marker(crosswalk_points, "Crosswalk", 0, ORANGE, 1.0, stamp))
        if right_of_way_lanelet_points:
            marker_array.markers.append(triangles_to_marker(right_of_way_lanelet_points, "Right of way area", 0, INDIGO, 1.0, stamp))

        return marker_array

    def visualize_regulatoryElementLayer(self, regulatory_elements):
        
        # Create a MarkerArray
        marker_array = MarkerArray()

        # Iterate over all the regulatory elements
        for reg_el in regulatory_elements:
            # Check if the regulatory element is a traffic light group
            if reg_el.attributes["subtype"] == "traffic_light":
                stamp = rospy.Time.now()
                # can have several individual traffic lights
                for tfl in reg_el.parameters["refers"]:
                    p1 = tfl[0]
                    p2 = tfl[1]

                    tfl_height = float(tfl.attributes["height"])

                    # calculate bulb positions
                    bulb_x = (p1.x + p2.x) / 2
                    bulb_y = (p1.y + p2.y) / 2
                    bulb_z = p1.z + 5*tfl_height/6

                    for i in range(3):
                        # Create a marker for the traffic light bulb
                        marker = Marker()
                        marker.header.frame_id = "map"
                        marker.header.stamp = stamp
                        marker.ns = "Traffic lights"
                        marker.id = len(marker_array.markers)
                        marker.type = marker.SPHERE
                        marker.action = marker.ADD
                        marker.scale.x = tfl_height/6
                        marker.scale.y = tfl_height/6
                        marker.scale.z = tfl_height/6
                        marker.color = INDEX_TO_MARKER_COLOR[i]
                        marker.pose.position.x = bulb_x
                        marker.pose.position.y = bulb_y
                        marker.pose.position.z = bulb_z
                        marker.pose.orientation.w = 1.0

                        marker_array.markers.append(marker)
                        bulb_z -= tfl_height/6
        return marker_array


    def visualize_lineStringLayer(self, linestrings):

        marker_array = MarkerArray()

        stop_line_points = []
        speed_bump_points = []
        for line in linestrings:
            # if has attributes, then select stop lines
            if line.attributes and "type" in line.attributes and line.attributes["type"] == "stop_line":
                points = convert_geometry_to_line_list(line)
                if "subtype" in line.attributes and line.attributes["subtype"] == "speed_bump":
                    speed_bump_points.extend(points)
                else:
                    stop_line_points.extend(points)

        if stop_line_points:
            marker_array.markers.append(linelist_to_marker(stop_line_points, "Stop lines", 0, WHITE, 0.3, rospy.Time.now()))
        if speed_bump_points:
            marker_array.markers.append(linelist_to_marker(speed_bump_points, "Speed bumps", 0, LIGHT_GREY, 0.3, rospy.Time.now()))

        return marker_array
    
    def visualize_polygonLayer(self, polygons):

        marker_array = MarkerArray()

        if not self.enable_road_closures:
            return marker_array
        
        now = datetime.now(timezone.utc)

        road_closure_polygon_points = []
        label_id = 0
        for polygon in polygons:
            # if has attributes, then select road closure areas
            if polygon.attributes and "type" in polygon.attributes and polygon.attributes["type"] == "road_closure":
                # check if the road closure is currently active
                start_time = datetime.fromisoformat(polygon.attributes["start_time"])
                end_time = datetime.fromisoformat(polygon.attributes["end_time"])
                if start_time <= now <= end_time:
                    polygon_points = [(point.x, point.y, point.z) for point in polygon]
                    points = triangulate_polygon(polygon_points)
                    road_closure_polygon_points.extend(points)

                    # add text marker to the road closure polygon
                    start_t = polygon.attributes["start_time"].split("+")[0] # remove timezone for display
                    end_t = polygon.attributes["end_time"].split("+")[0]
                    text = f"{polygon.attributes['description']}\n({start_t} - {end_t})"
                    marker_array.markers.append(polygon_text_marker(polygon_points, "Road closure labels", label_id, text, rospy.Time.now()))
                    label_id += 1

        if road_closure_polygon_points:
            marker_array.markers.append(triangles_to_marker(road_closure_polygon_points, "Road closures", 0, PINK, 1.0, rospy.Time.now()))

        return marker_array

def linelist_to_marker(points, namespace, id, color, scale, stamp):
    """
    Creates a Marker from a LineString
    :param linestring: LineString
    :param namespace: Marker namespace
    :param id: Marker id
    :param color: Marker color
    :param stamp: Marker timestamp
    :return: Marker
    """
    # Create a Marker
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = stamp
    marker.ns = namespace
    marker.id = id
    marker.type = marker.LINE_LIST
    marker.action = marker.ADD
    marker.scale.x = scale
    marker.color = color
    marker.pose.orientation.w = 1.0
    marker.points = points
    
    return marker

def triangles_to_marker(points, namespace, id, color, scale, stamp):
    """
    Creates a Marker from a list of triangle points
    :param points: 1D list,every set of 3 points is treated as a triangle
    :param namespace: Marker namespace
    :param id: Marker id
    :param color: Marker color
    :param stamp: Marker timestamp
    :return: Marker
    """
    # Create a Marker
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = stamp
    marker.ns = namespace
    marker.id = id
    marker.type = marker.TRIANGLE_LIST
    marker.action = marker.ADD
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    marker.pose.orientation.w = 1.0
    marker.color = color
    marker.colors = [color] * len(points)
    marker.points = points
    
    return marker

def polygon_text_marker(polygon_points, namespace, id, text, stamp):
    """
    Creates a text Marker for a polygon at its centroid
    :param polygon_points: 2d list of polygon points
    :param namespace: Marker namespace
    :param id: Marker id
    :param text: text to display
    :param stamp: Marker timestamp
    :return: Marker
    """

    centroid_x, centroid_y, centroid_z = np.mean(polygon_points, axis=0).tolist()

    # Create a Marker
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = stamp
    marker.ns = namespace
    marker.id = id
    marker.type = marker.TEXT_VIEW_FACING
    marker.action = marker.ADD
    marker.pose.position.x = centroid_x
    marker.pose.position.y = centroid_y
    marker.pose.position.z = centroid_z
    marker.pose.orientation.w = 1.0
    marker.scale.z = 1.0
    marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
    marker.text = text
    
    return marker

def get_polygon_from_lanelet(lanelet):
    """
    Creates a list of polygon points from a lanelet
    :param lanelet: lanelet2.core.Lanelet
    :return: list of polygon points
    """
    polygon_points = []
    polygon_points.extend([(point.x, point.y, point.z) for point in lanelet.leftBound])
    polygon_points.append((lanelet.centerline[-1].x, lanelet.centerline[-1].y, lanelet.centerline[-1].z))
    polygon_points.extend([(point.x, point.y, point.z) for point in lanelet.rightBound.invert()])
    polygon_points.append((lanelet.centerline[0].x, lanelet.centerline[0].y, lanelet.centerline[0].z))
    polygon_points.append((lanelet.leftBound[0].x, lanelet.leftBound[0].y, lanelet.leftBound[0].z))
    return polygon_points

if __name__ == '__main__':
    rospy.init_node('lanelet2_map_visualizer')
    node = Lanelet2MapVisualizer()
    node.run()