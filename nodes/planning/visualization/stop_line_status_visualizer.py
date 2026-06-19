#!/usr/bin/env python3

import rospy
from collections import defaultdict
from autoware_mini.lanelet2 import load_lanelet2_map, get_stop_lines
from autoware_mini.msg import StopLineStatus, StopLineStatusArray
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

RED = ColorRGBA(1.0, 0.0, 0.0, 1.0)
GREEN = ColorRGBA(0.0, 1.0, 0.0, 1.0)
UNKNOWN = ColorRGBA(1.0, 1.0, 1.0, 0.5)
WHITE = ColorRGBA(1.0, 1.0, 1.0, 1.0)

STATUS_TYPE_TO_NAMESPACE = {
    StopLineStatusArray.TRAFFIC_LIGHT: "Traffic light",
    StopLineStatusArray.YIELD_STOP: "Stop line",
    StopLineStatusArray.YIELD: "Yield line",
    StopLineStatusArray.YIELD_MANUAL: "Manual yield line",
    StopLineStatusArray.RIGHT_OF_WAY: "Right of way yield line",
}

class StopLineStatusVisualizer:
    def __init__(self):

        # parameters
        lanelet2_map_path = rospy.get_param("~lanelet2_map_path")

        lanelet2_map = load_lanelet2_map(lanelet2_map_path)
        self.stop_lines = get_stop_lines(lanelet2_map)

        # publishers
        self.marker_pub = rospy.Publisher('stop_line_status_markers', MarkerArray, queue_size=5)

        # subscribers
        rospy.Subscriber('stop_line_status', StopLineStatusArray, self.stop_line_status_callback, queue_size=5, tcp_nodelay=True)

        # Store published stop line IDs per type
        self.published_stop_line_ids = defaultdict(set)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def stop_line_status_callback(self, msg):
        markers = MarkerArray()
        current_type = msg.type
        current_stop_line_ids = set()

        # Process new statuses and add markers
        for status in msg.statuses:

            if status.status == StopLineStatus.STATUS_STOP:
                color = RED
                delta_z = 0.1    # Raise the red line slightly
            elif status.status == StopLineStatus.STATUS_GO:
                color = GREEN
                delta_z = 0.05   # keep below red
            else:
                color = UNKNOWN
                delta_z = 0.0

            # Create stop line points
            stop_line = self.stop_lines[status.stop_line_id]
            stop_line_points = [Point(x=x, y=y, z=z + delta_z) for x, y, z in stop_line.coords]

            # create stop line markers
            marker = Marker()
            marker.ns = STATUS_TYPE_TO_NAMESPACE[current_type]
            marker.header.frame_id = "map"
            marker.id = status.stop_line_id
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.points = stop_line_points
            marker.color = color
            marker.scale.x = 0.3
            marker.pose.orientation.w = 1.0
            marker.lifetime = rospy.Duration(0.5)
            markers.markers.append(marker)
            current_stop_line_ids.add(status.stop_line_id)

            # Add label markers for type TRAFFIC_LIGHT
            if current_type == StopLineStatusArray.TRAFFIC_LIGHT:
                marker = Marker()
                marker.ns = "Traffic Light Status"
                marker.header.frame_id = "map"
                marker.id = status.stop_line_id
                marker.type = marker.TEXT_VIEW_FACING
                marker.action = marker.ADD
                marker.scale.z = 0.5
                marker.color = WHITE
                marker.text = status.status_text
                marker.pose.position.x = (stop_line_points[0].x + stop_line_points[-1].x) / 2.0
                marker.pose.position.y = (stop_line_points[0].y + stop_line_points[-1].y) / 2.0
                marker.pose.position.z = (stop_line_points[0].z + stop_line_points[-1].z) / 2.0 + 0.25
                marker.pose.orientation.w = 1.0
                marker.lifetime = rospy.Duration(0.5)
                markers.markers.append(marker)

        # Remove disappeared markers only for current type
        missing_ids = self.published_stop_line_ids[current_type] - current_stop_line_ids
        for id in missing_ids:
            marker = Marker()
            marker.ns = STATUS_TYPE_TO_NAMESPACE[current_type]
            marker.id = id
            marker.action = Marker.DELETE
            markers.markers.append(marker)

            # remove traffic light label marker
            if current_type == StopLineStatusArray.TRAFFIC_LIGHT:
                marker = Marker()
                marker.ns = "Traffic Light Status"
                marker.id = id
                marker.action = Marker.DELETE
                markers.markers.append(marker)

        # Update stored IDs only for current type
        self.published_stop_line_ids[current_type] = current_stop_line_ids
        self.marker_pub.publish(markers)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('stop_line_status_visualizer', log_level=rospy.INFO)
    node = StopLineStatusVisualizer()
    node.run()