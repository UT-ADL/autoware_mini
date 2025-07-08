#!/usr/bin/env python3

import rospy
import time
import shapely

from autoware_mini.msg import TrafficLightResultArray
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import ColorRGBA, Int32
from lanelet2.core import BasicPoint2d, BoundingBox2d
from autoware_mini.lanelet2 import load_lanelet2_map, get_stop_lines_using_subtype
from autoware_mini.geometry import get_distance_between_two_points_2d, convert_geometry_to_line_list
from autoware_mini.visualization import triangulate_path, triangulate_polygon


# used for traffic lights
RED = ColorRGBA(1.0, 0.0, 0.0, 0.8)
YELLOW = ColorRGBA(1.0, 1.0, 0.0, 0.8)
GREEN = ColorRGBA(0.0, 1.0, 0.0, 0.8)

# colors for other map features
GREY = ColorRGBA(0.4, 0.4, 0.4, 0.6)
ORANGE = ColorRGBA(1.0, 0.5, 0.0, 0.6)
WHITE = ColorRGBA(1.0, 1.0, 1.0, 0.6)
CYAN = ColorRGBA(0.0, 1.0, 1.0, 0.6)
LIGHT_BLUE = ColorRGBA(0.0, 0.7, 0.9, 0.6)
DARK_BLUE = ColorRGBA(0.3, 0.3, 1.0, 0.6)
WHITE100 = ColorRGBA(1.0, 1.0, 1.0, 1.0)

TRAFFIC_LIGHT_STATE_TO_MARKER_COLOR = {
    0: RED,     # red and yellow
    1: GREEN,
    2: WHITE
}

LANELET_COLOR_TO_MARKER_COLOR = {
    "red": RED,
    "yellow": YELLOW,
    "green": GREEN,
}

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
        self.enable_auto_stop_checker = rospy.get_param("~enable_auto_stop_checker")

        self.map_extraction_location = None
        self.lanelet2_map = load_lanelet2_map(lanelet2_map_path)
        self.yield_stop_lines = get_stop_lines_using_subtype(self.lanelet2_map, subtypes=["yield_stop"])

        # Special publishers for stop line markers: traffic_lights and yielding
        self.tfl_stop_line_markers_pub = rospy.Publisher('tfl_stop_line_markers', MarkerArray, queue_size=1, latch=True, tcp_nodelay=True)
        self.yield_stop_line_markers_pub = rospy.Publisher('yield_stop_line_markers', MarkerArray, queue_size=1, latch=True, tcp_nodelay=True)
        self.lanelet2_map_markers_pub = rospy.Publisher('lanelet2_map_markers', MarkerArray, queue_size=1, latch=True, tcp_nodelay=True)

        rospy.Subscriber("/detection/traffic_light_status", TrafficLightResultArray, self.traffic_light_status_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/planning/lets_go', Int32, self.lets_go_callback, queue_size=1, tcp_nodelay=True)

        if self.use_map_extraction:
            rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1, tcp_nodelay=True)
            # for filtering trafiiclight stopline statuses
            self.filtered_linestrings = None
        else:
            lanelet_markers = self.visualize_laneletLayer(self.lanelet2_map.laneletLayer)
            linestring_markers = self.visualize_lineStringLayer(self.lanelet2_map.lineStringLayer)
            reg_el_markers = self.visualize_regulatoryElementLayer(self.lanelet2_map.regulatoryElementLayer)
            marker_array = MarkerArray()
            marker_array.markers = lanelet_markers.markers + linestring_markers.markers + reg_el_markers.markers
            self.lanelet2_map_markers_pub.publish(marker_array)

        rospy.loginfo("%s - map loaded with %i lanelets and %i regulatory elements from file: %s", rospy.get_name(),
                      len(self.lanelet2_map.laneletLayer), len(self.lanelet2_map.regulatoryElementLayer), lanelet2_map_path)

    def current_pose_callback(self, msg):

        if self.map_extraction_location is None or get_distance_between_two_points_2d(self.map_extraction_location, msg.pose.position) > (self.map_extraction_distance - 2*self.local_path_length):
            self.map_extraction_location = BasicPoint2d(msg.pose.position.x, msg.pose.position.y)
            search_box = BoundingBox2d(BasicPoint2d(msg.pose.position.x - self.map_extraction_distance, msg.pose.position.y - self.map_extraction_distance),
                                        BasicPoint2d(msg.pose.position.x + self.map_extraction_distance, msg.pose.position.y + self.map_extraction_distance))

            filtered_lanelets = self.lanelet2_map.laneletLayer.search(search_box)
            filtered_linestrings = self.lanelet2_map.lineStringLayer.search(search_box)
            filtered_regulatory_elements = self.lanelet2_map.regulatoryElementLayer.search(search_box)

            # Visualize different parts of the map
            lanelet_markers = self.visualize_laneletLayer(filtered_lanelets)
            linestring_markers = self.visualize_lineStringLayer(filtered_linestrings)
            reg_el_markers = self.visualize_regulatoryElementLayer(filtered_regulatory_elements)

           # conactenate the MarkerArrays with delete all at front
            marker_array = MarkerArray()
            marker = Marker()
            marker.action = Marker.DELETEALL
            marker_array.markers = [marker] + lanelet_markers.markers + linestring_markers.markers + reg_el_markers.markers

            self.lanelet2_map_markers_pub.publish(marker_array)
            self.filtered_linestrings = {linestring.id: linestring for linestring in filtered_linestrings}

    def lets_go_callback(self, msg):
        marker_array = MarkerArray()
        marker = Marker()
        marker.action = Marker.DELETEALL
        marker_array.markers.append(marker)

        if msg.data != -1:
            line = self.yield_stop_lines[msg.data].coords
            points = convert_geometry_to_line_list(line, delta_z=0.1)
            marker = linelist_to_marker(points, "Yield line", msg.data, GREEN, 0.5, rospy.Time.now())
            marker_array.markers.append(marker)

        self.yield_stop_line_markers_pub.publish(marker_array)

    def traffic_light_status_callback(self, msg):
        marker_array = MarkerArray()
        # delete all previous markers
        marker = Marker()
        marker.action = Marker.DELETEALL
        marker_array.markers.append(marker)

        states = {}
        for result in msg.results:
            # check if we have already outputted the status of this stopline
            if result.stopline_id in states:
                if states[result.stopline_id] != result.recognition_result_str:
                    rospy.logwarn("%s - multiple traffic lights with different states on the same stop line %d: %s != %s", rospy.get_name(), result.stopline_id, states[result.stopline_id], result.recognition_result_str)
                continue

            if self.use_map_extraction and self.filtered_linestrings is not None:
                # Check if the linestring with the target ID is in the filtered list
                if not (result.stopline_id in self.filtered_linestrings):
                    continue

            # fetch the stop line data
            stop_line = self.lanelet2_map.lineStringLayer.get(result.stopline_id)
            points = convert_geometry_to_line_list(stop_line, delta_z=0.1)

            # choose the color of stopline based on the traffic light state
            if result.recognition_result in TRAFFIC_LIGHT_STATE_TO_MARKER_COLOR:
                color = TRAFFIC_LIGHT_STATE_TO_MARKER_COLOR[result.recognition_result]
            else:
                rospy.logwarn("%s - unrecognized traffic light state: %d", rospy.get_name(), result.recognition_result)
                color = WHITE

            # check if string contains "FLASH" string in it
            if "FLASH" in result.recognition_result_str:
                color = ColorRGBA(color.r, color.g, color.b, color.a * 0.5 if time.time() % 1 < 0.5 else 1.0)

            # create linestring marker
            stopline_marker = linelist_to_marker(points, "Stop line", stop_line.id, color, 0.5, rospy.Time.now())

            marker_array.markers.append(stopline_marker)

            # create traffic light status marker
            text_marker = text_to_marker(result.recognition_result_str, points, "Status text", stop_line.id, WHITE100, 0.5, rospy.Time.now())
            marker_array.markers.append(text_marker)

            # record the state of this stop line
            states[result.stopline_id] = result.recognition_result_str

        self.tfl_stop_line_markers_pub.publish(marker_array)

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

        for lanelet in lanelets:
            if lanelet.attributes["subtype"] == "road" or lanelet.attributes["subtype"] == "bus_lane" or lanelet.attributes["subtype"] == "bicycle_lane":
                # Visualize left and right boundaries
                left_boundary_points.extend(convert_geometry_to_line_list(lanelet.leftBound))
                right_boundary_points.extend(convert_geometry_to_line_list(lanelet.rightBound))

                centerline = shapely.linestrings([[p.x, p.y, p.z] for p in lanelet.centerline])

                # triangulate centerline
                if lanelet.attributes["subtype"] == "road":
                    centerline_points.extend(triangulate_path(centerline, 1.5))
                elif lanelet.attributes["subtype"] == "bus_lane":
                    bus_lane_points.extend(triangulate_path(centerline, 1.5))
                elif lanelet.attributes["subtype"] == "bicycle_lane":
                    bicycle_lane_points.extend(triangulate_path(centerline, 0.8))

            elif lanelet.attributes["subtype"] == "crosswalk":
                # create "polygon points" from crosswalk lanelet and then create line list from them
                crosswalk_border = [(point.x, point.y, point.z) for point in lanelet.leftBound]
                crosswalk_border.extend([(point.x, point.y, point.z) for point in lanelet.rightBound.invert()])
                crosswalk_border.append((lanelet.leftBound[0].x, lanelet.leftBound[0].y, lanelet.leftBound[0].z))

                crosswalk_points.extend(triangulate_polygon(crosswalk_border))

        left_boundary_marker = linelist_to_marker(left_boundary_points, "Left boundary", 0, GREY, 0.1, stamp)
        right_boundary_marker = linelist_to_marker(right_boundary_points, "Right boundary", 0, GREY, 0.1, stamp)
        centerline_marker = triangles_to_marker(centerline_points, "Centerline", 0, CYAN, 1.0, stamp)
        bus_lane_marker = triangles_to_marker(bus_lane_points, "Bus lane", 0, DARK_BLUE, 1.0, stamp)
        bicycle_lane_marker = triangles_to_marker(bicycle_lane_points, "Bicycle lane", 0, LIGHT_BLUE, 1.0, stamp)
        crosswalk_marker = triangles_to_marker(crosswalk_points, "Crosswalk", 0, ORANGE, 1.0, stamp)

        marker_array.markers.append(left_boundary_marker)
        marker_array.markers.append(right_boundary_marker)
        marker_array.markers.append(centerline_marker)
        marker_array.markers.append(bus_lane_marker)
        marker_array.markers.append(bicycle_lane_marker)
        marker_array.markers.append(crosswalk_marker)

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

        points_traffic_light = []
        points_yield_stop = []
        points_yield = []

        for line in linestrings:
                # if has attributes
                if line.attributes:
                    # select stop lines
                    if line.attributes["type"] == "stop_line":
                        # points = [point for point in line]
                        points = convert_geometry_to_line_list(line)
                        if "subtype" in line.attributes:
                            if line.attributes["subtype"]=="traffic_light":
                                points_traffic_light.extend(points)
                            elif line.attributes["subtype"]=="yield_stop":
                                points_yield_stop.extend(points)
                            elif line.attributes["subtype"]=="yield":
                                points_yield.extend(points)

        marker_array.markers.append(linelist_to_marker(points_traffic_light, "Traffic light stop lines", 0, WHITE, 0.5, rospy.Time.now()))
        marker_array.markers.append(linelist_to_marker(points_yield_stop, "Yield stop line", 0, RED if self.enable_auto_stop_checker else GREEN, 0.5, rospy.Time.now()))
        marker_array.markers.append(linelist_to_marker(points_yield, "Yield line", 0, YELLOW, 0.3, rospy.Time.now()))

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

def text_to_marker(text, linestring, namespace, id, color, scale, stamp):
    """
    Creates a Marker from a text
    :param text: text
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
    marker.type = marker.TEXT_VIEW_FACING
    marker.action = marker.ADD
    marker.scale.z = scale
    marker.color = color
    marker.pose.position.x = (linestring[0].x + linestring[-1].x) / 2.0
    marker.pose.position.y = (linestring[0].y + linestring[-1].y) / 2.0
    marker.pose.position.z = (linestring[0].z + linestring[-1].z) / 2.0
    marker.pose.orientation.w = 1.0
    marker.text = text

    return marker

if __name__ == '__main__':
    rospy.init_node('lanelet2_map_visualizer')
    node = Lanelet2MapVisualizer()
    node.run()