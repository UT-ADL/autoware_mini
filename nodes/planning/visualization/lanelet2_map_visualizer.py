#!/usr/bin/env python3

import rospy
import time
from lanelet2.io import Origin, load
from lanelet2.projection import UtmProjector

from autoware_msgs.msg import TrafficLightResultArray
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


# used for traffic lights
RED = ColorRGBA(1.0, 0.0, 0.0, 0.8)
YELLOW = ColorRGBA(1.0, 1.0, 0.0, 0.8)
GREEN = ColorRGBA(0.0, 1.0, 0.0, 0.8)

# colors for other map features
GREY = ColorRGBA(0.4, 0.4, 0.4, 0.6)
ORANGE = ColorRGBA(1.0, 0.5, 0.0, 0.6)
WHITE = ColorRGBA(1.0, 1.0, 1.0, 0.6)
CYAN = ColorRGBA(0.0, 1.0, 1.0, 0.3)
WHITE100 = ColorRGBA(1.0, 1.0, 1.0, 1.0)

LANELET_COLOR_TO_MARKER_COLOR = {
    "RED": RED,
    "RED cam": RED,
    "RED/AMB": RED,
    "AMBER-RED": RED,
    "AMBERRED" : RED,
    "YELLOW": YELLOW,
    "AMBER": YELLOW,
    "AMB FLASH": YELLOW,
    "AMBER FLASH": YELLOW,
    "GREEN": GREEN,
    "GREEN cam": GREEN,
    "GREEN FLASH": GREEN,
    "UNKNOWN": WHITE,
    "UNKNOWN cam": WHITE,
    "OFF": WHITE,
}

class Lanelet2MapVisualizer:

    def __init__(self):
    
        # Parameters
        lanelet2_map_name = rospy.get_param("~lanelet2_map_name")
        
        coordinate_transformer = rospy.get_param("/localization/coordinate_transformer")
        use_custom_origin = rospy.get_param("/localization/use_custom_origin")
        utm_origin_lat = rospy.get_param("/localization/utm_origin_lat")
        utm_origin_lon = rospy.get_param("/localization/utm_origin_lon")

        # Load the map using Lanelet2
        if coordinate_transformer == "utm":
            projector = UtmProjector(Origin(utm_origin_lat, utm_origin_lon), use_custom_origin, False)
        else:
            rospy.logfatal("lanelet2_global_planner - only utm and custom origin currently supported for lanelet2 map loading")
            exit(1)

        self.lanelet2_map = load(lanelet2_map_name, projector)

        # Visualize the Lanelet2 map
        marker_array = visualize_lanelet2_map(self.lanelet2_map)

        # create MarkerArray publisher
        markers_pub = rospy.Publisher('lanelet2_map_markers', MarkerArray, queue_size=1, latch=True)
        markers_pub.publish(marker_array)

        # Special publisher for stop line markers
        self.stop_line_markers_pub = rospy.Publisher('stop_line_markers', MarkerArray, queue_size=1, latch=True)
        rospy.Subscriber("/detection/traffic_light_status", TrafficLightResultArray, self.traffic_light_status_callback)

        rospy.loginfo("lanelet2_map_visualizer - map loaded with %i lanelets and %i regulatory elements from file: %s",
                      len(self.lanelet2_map.laneletLayer), len(self.lanelet2_map.regulatoryElementLayer), lanelet2_map_name)

    def traffic_light_status_callback(self, msg):
        marker_array = MarkerArray()
        states = {}
        for result in msg.results:
            # check if we have already outputted the status of this stopline
            if result.lane_id in states:
                assert states[result.lane_id] == result.recognition_result_str, "Multiple traffic lights with different states on the same stop line"
                continue

            # fetch the stop line data
            stop_line = self.lanelet2_map.lineStringLayer.get(result.lane_id)
            points = [Point(x=p.x, y=p.y, z=p.z + 0.01) for p in stop_line]

            # choose the color of stopline based on the traffic light state
            if result.recognition_result_str in LANELET_COLOR_TO_MARKER_COLOR:
                color = LANELET_COLOR_TO_MARKER_COLOR[result.recognition_result_str]
            else:
                rospy.logwarn("lanelet2_map_visualizer - unrecognized traffic light state: %s", result.recognition_result_str)
                color = WHITE

            # check if string contains "FLASH" string in it
            if "FLASH" in result.recognition_result_str:
                color = ColorRGBA(color.r, color.g, color.b, color.a * get_multiplier())

            # create linestring marker
            stopline_marker = linestring_to_marker(points, "Stop line", stop_line.id, color, 0.5, rospy.Time.now())

            marker_array.markers.append(stopline_marker)

            # create traffic light status marker
            text_marker = text_to_marker(result.recognition_result_str, points, "Status text", stop_line.id, WHITE100, 0.5, rospy.Time.now())
            marker_array.markers.append(text_marker)

            # record the state of this stop line
            states[result.lane_id] = result.recognition_result_str

        self.stop_line_markers_pub.publish(marker_array)

    def run(self):
        rospy.spin()


def get_multiplier():
    if time.time() % 1 < 0.5:
        return 0.5
    else:
        return 1.0


def visualize_lanelet2_map(map):

    # Create a MarkerArray
    marker_array = MarkerArray()

    # Visualize different parts of the map
    lanelet_markers = visualize_laneltLayer(map)
    reg_el_markers = visualize_regulatoryElementLayer(map)
    linestring_markers = visualize_lineStringLayer(map)

    # conactenate the MarkerArrays
    marker_array.markers = lanelet_markers.markers + reg_el_markers.markers + linestring_markers.markers
    return marker_array


def visualize_laneltLayer(map):

    # Create a MarkerArray
    marker_array = MarkerArray()

    for lanelet in map.laneletLayer:

        stamp = rospy.Time.now()

        # TODO bicycle_lane, bus_lane, emergency_lane, parking_lane, pedestrian_lane, sidewalk, special_lane, traffic_island, traffic_lane, traffic_zone, walkway        
        if lanelet.attributes["subtype"] == "road":
        
            # Create markers for the left, right boundary and centerline
            left_boundary_marker = linestring_to_marker(lanelet.leftBound, "Left boundary", lanelet.id, GREY, 0.1, stamp)
            right_boundary_marker = linestring_to_marker(lanelet.rightBound, "Right boundary", lanelet.id, GREY, 0.1, stamp)
            centerline_marker = linestring_to_marker(lanelet.centerline, "Centerline", lanelet.id, CYAN, 1.5, stamp)

            # Add the markers to the MarkerArray
            marker_array.markers.append(left_boundary_marker)
            marker_array.markers.append(right_boundary_marker)
            marker_array.markers.append(centerline_marker)

        elif lanelet.attributes["subtype"] == "crosswalk":

            points = [point for point in lanelet.leftBound]
            points += [point for point in lanelet.rightBound.invert()]
            points.append(lanelet.leftBound[0])

            crosswalk_marker = linestring_to_marker(points, "Crosswalk", lanelet.id, ORANGE, 0.3, stamp)
            marker_array.markers.append(crosswalk_marker)

    return marker_array

def visualize_regulatoryElementLayer(map):
    
    # Create a MarkerArray
    marker_array = MarkerArray()

    # Iterate over all the regulatory elements
    for reg_el in map.regulatoryElementLayer:
        # Check if the regulatory element is a traffic light group
        if reg_el.attributes["subtype"] == "traffic_light":
            stamp = rospy.Time.now()
            # can have several individual traffic lights
            for tfl in reg_el.parameters["light_bulbs"]:
                for bulb in tfl:
                    # Create a marker for the traffic light bulb
                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.header.stamp = stamp
                    marker.ns = "Traffic lights"
                    marker.id = bulb.id
                    marker.type = marker.SPHERE
                    marker.action = marker.ADD
                    marker.scale.x = 0.2
                    marker.scale.y = 0.2
                    marker.scale.z = 0.2
                    # marker.color = map.pointLayer.get(bulb.id).attributes["color"]
                    marker.color = LANELET_COLOR_TO_MARKER_COLOR[map.pointLayer.get(bulb.id).attributes["color"].upper()]
                    marker.pose.position.x = bulb.x
                    marker.pose.position.y = bulb.y
                    marker.pose.position.z = bulb.z
                    marker.pose.orientation.w = 1.0

                    marker_array.markers.append(marker)

        # TODO stop line, yield line, speed limit, etc.

    return marker_array


def visualize_lineStringLayer(map):

    marker_array = MarkerArray()

    for line in map.lineStringLayer:
            # if has attributes
            if line.attributes:
                # select stop lines
                if line.attributes["type"] == "stop_line":
                    points = [point for point in line]
                    stopline_marker = linestring_to_marker(points, "Stop line", line.id, WHITE, 0.5, rospy.Time.now())
                    marker_array.markers.append(stopline_marker)

    return marker_array


def linestring_to_marker(linestring, namespace, id, color, scale, stamp):
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
    marker.type = marker.LINE_STRIP
    marker.action = marker.ADD
    marker.scale.x = scale
    marker.color = color
    marker.pose.orientation.w = 1.0

    # Add the points to the marker
    for point in linestring:
        marker.points.append(point)

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