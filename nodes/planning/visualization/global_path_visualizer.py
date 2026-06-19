#!/usr/bin/env python3

import rospy
import shapely
import numpy as np

from autoware_mini.msg import Path
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA

from autoware_mini.visualization import triangulate_linestring

GLOBAL_PATH_COLOR = ColorRGBA(0.9, 0.6, 1.0, 0.7)

class GlobalPathVisualizer:
    def __init__(self):

        # Publishers
        self.global_path_markers_pub = rospy.Publisher('global_path_markers', MarkerArray, queue_size=10, latch=True, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber('global_path', Path, self.global_path_callback, queue_size=None, tcp_nodelay=True)

    def global_path_callback(self, path):
        marker_array = MarkerArray()

        if not path.waypoints:
            # create marker_array to delete all visualization markers
            marker = Marker()
            marker.header.frame_id = path.header.frame_id
            marker.action = Marker.DELETEALL
            marker_array.markers.append(marker)

        else:
            linestring_coords = np.array([[p.position.x, p.position.y, p.position.z] for p in path.waypoints])

            # velocity labels
            for i, waypoint in enumerate(path.waypoints):
                marker = Marker()
                marker.header.frame_id = path.header.frame_id
                marker.header.stamp = rospy.Time.now()
                marker.ns = "Velocity label"
                marker.id = i
                marker.type = marker.TEXT_VIEW_FACING
                marker.action = marker.ADD
                marker.pose.position.x = waypoint.position.x
                marker.pose.position.y = waypoint.position.y
                marker.pose.position.z = waypoint.position.z
                marker.pose.orientation.w = 1.0
                marker.scale.z = 0.5
                marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
                marker.text = str(round(waypoint.speed * 3.6, 1))
                marker_array.markers.append(marker)

            # Triangulate global path
            linestring = shapely.linestrings(linestring_coords)
            linestring = linestring.simplify(0.05)
            triangle_points = triangulate_linestring(linestring, 1.5, z_offset=0.1)

            # Create a Marker
            marker = Marker()
            marker.header.frame_id = path.header.frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = "Path"
            marker.id = 0
            marker.type = marker.TRIANGLE_LIST
            marker.action = marker.ADD
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.pose.orientation.w = 1.0
            marker.color = GLOBAL_PATH_COLOR
            marker.colors = [GLOBAL_PATH_COLOR] * len(triangle_points)
            marker.points = triangle_points
            marker_array.markers.append(marker)
    
        self.global_path_markers_pub.publish(marker_array)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('global_path_visualizer', log_level=rospy.INFO)
    node = GlobalPathVisualizer()
    node.run()