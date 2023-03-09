import rospy
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA


GREY = ColorRGBA(0.4, 0.4, 0.4, 0.4)
RED = ColorRGBA(1.0, 0.0, 0.0, 0.4)
GREEN = ColorRGBA(0.0, 1.0, 0.0, 0.4)
BLUE = ColorRGBA(0.0, 0.0, 1.0, 0.4)
YELLOW = ColorRGBA(1.0, 1.0, 0.0, 0.4)
ORANGE = ColorRGBA(1.0, 0.5, 0.0, 0.4)



def draw_lanelets(map):
    """
    Draws lanelets on the map
    :param map: Lanelet2 map
    :return:
    """
    
    # Create a MarkerArray message
    marker_array = MarkerArray()

    for lanelet in map.laneletLayer:
        
        stamp = rospy.Time.now()
        
        if lanelet.attributes["subtype"] == "road":
        
            # Create markers for the left, right boundary and centerline
            left_boundary_marker = linestring_to_marker(lanelet.leftBound, "Left boundary", lanelet.id, BLUE, 0.2, stamp)
            right_boundary_marker = linestring_to_marker(lanelet.rightBound, "Right boundary", lanelet.id, GREY, 0.2, stamp)
            centerline_marker = linestring_to_marker(lanelet.centerline, "Centerline", lanelet.id, YELLOW, 0.1, stamp)

            # Add the markers to the MarkerArray
            marker_array.markers.append(left_boundary_marker)
            marker_array.markers.append(right_boundary_marker)
            marker_array.markers.append(centerline_marker)

        elif lanelet.attributes["subtype"] == "crosswalk":
            left_crosswalk = linestring_to_marker(lanelet.leftBound, "Left boundary", lanelet.id, GREEN, 0.3, stamp)
            right_crosswalk = linestring_to_marker(lanelet.rightBound, "Right boundary", lanelet.id, GREEN, 0.3, stamp)

            marker_array.markers.append(left_crosswalk)
            marker_array.markers.append(right_crosswalk)


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

