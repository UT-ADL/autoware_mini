import rospy
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA


# used for traffic lights
RED = ColorRGBA(1.0, 0.0, 0.0, 0.8)
YELLOW = ColorRGBA(1.0, 1.0, 0.0, 0.8)
GREEN = ColorRGBA(0.0, 1.0, 0.0, 0.8)

GREY = ColorRGBA(0.4, 0.4, 0.4, 0.6)
ORANGE = ColorRGBA(1.0, 0.5, 0.0, 0.6)
WHITE = ColorRGBA(1.0, 1.0, 1.0, 0.6)
CYAN = ColorRGBA(0.0, 1.0, 1.0, 0.3)

LANELET_COLOR_TO_MARKER_COLOR = {
    "red": RED,
    "yellow": YELLOW,
    "green": GREEN
}


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
                    marker.color = LANELET_COLOR_TO_MARKER_COLOR[map.pointLayer.get(bulb.id).attributes["color"]]
                    marker.pose.position.x = bulb.x
                    marker.pose.position.y = bulb.y
                    marker.pose.position.z = bulb.z

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

    # Add the points to the marker
    for point in linestring:
        marker.points.append(point)

    return marker

