#!/usr/bin/env python3

import rospy
import math
import shapely

from autoware_mini.msg import LocalPath
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA

from autoware_mini.path import PathWrapper
from autoware_mini.collision import CollisionPoints
from autoware_mini.visualization import triangulate_linestring
from autoware_mini.transform import get_distance_to_car_front
from autoware_mini.shapely import linesubstring


LOCAL_PATH_COLOR = ColorRGBA(0.1, 1.0, 0.1, 0.6)
LB_SAFETY_BOX_COLOR = ColorRGBA(1.0, 0.4, 0.1, 0.6)

class LocalPathVisualizer:
    def __init__(self):

        # Parameters
        self.use_swerving = rospy.get_param("~use_swerving")
        self.safety_box_width = rospy.get_param("safety_box_width")
        self.wide_safety_box_width = rospy.get_param("wide_safety_box_width")
        self.narrow_safety_box_width = rospy.get_param("narrow_safety_box_width")
        self.stopped_speed_limit = rospy.get_param("stopped_speed_limit")

        self.published_waypoints = 0
        self.current_position = None
        self.distance_to_car_front = get_distance_to_car_front()

        # Publishers
        self.local_path_markers_pub = rospy.Publisher('local_path_markers', MarkerArray, queue_size=1, tcp_nodelay=True)
        self.lane_boundary_safety_box_pub = rospy.Publisher('lane_boundary_safety_box_markers', MarkerArray, queue_size=1, tcp_nodelay=True)
        # Subscribers
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('local_path', LocalPath, self.local_path_callback, queue_size=1, buff_size=2**20, tcp_nodelay=True)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def current_pose_callback(self, msg):
        self.current_position = shapely.Point(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def local_path_callback(self, msg):
        current_position = self.current_position
        if current_position is None:
            return

        stopping_point_distance = max(msg.stopping_point_distance, 0.0)
        collision_point_category = msg.collision_point_category

        marker_array = MarkerArray()
        lb_safety_box_marker_array = None
        if self.lane_boundary_safety_box_pub.get_num_connections() > 0:
            lb_safety_box_marker_array = MarkerArray()

        if len(msg.waypoints) > 1:

            # Triangulate loal path
            linestring = shapely.linestrings([[p.position.x, p.position.y, p.position.z] for p in msg.waypoints])
            linestring = linestring.simplify(0.01)
            if self.use_swerving:
                splitting_distance = linestring.project(current_position) + self.distance_to_car_front
                linestring_narrow = linesubstring(linestring, 0, splitting_distance)
                linestring_wide = linesubstring(linestring, splitting_distance, linestring.length)
                triangle_points = triangulate_linestring(linestring_narrow, self.narrow_safety_box_width, z_offset=0.1) + \
                                 triangulate_linestring(linestring_wide, self.wide_safety_box_width, z_offset=0.1)
            else:
                triangle_points = triangulate_linestring(linestring, self.safety_box_width, z_offset=0.1)

            marker = Marker(header=msg.header)
            marker.ns = "Stopping lateral distance"
            marker.type = marker.TRIANGLE_LIST
            marker.action = marker.ADD
            marker.id = 0
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.pose.orientation.w = 1.0
            marker.color = LOCAL_PATH_COLOR
            marker.lifetime = rospy.Duration(0.3)
            marker.colors = [LOCAL_PATH_COLOR] * len(triangle_points)
            marker.points = triangle_points
            marker_array.markers.append(marker)

            # velocity labels
            current_waypoints = 0
            for i, waypoint in enumerate(msg.waypoints):
                marker = Marker(header=msg.header)
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
                marker.lifetime = rospy.Duration(0.3)
                marker.text = str(round(waypoint.speed * 3.6, 1))
                marker_array.markers.append(marker)

                current_waypoints = i
                # add only up to a first 0.0 velocity label
                if math.isclose(waypoint.speed, 0.0):
                    break

            # delete all markers if local path length decreased
            if self.published_waypoints > current_waypoints:
                for j in range(current_waypoints + 1, self.published_waypoints + 1):
                    marker = Marker(header=msg.header)
                    marker.ns = "Velocity label"
                    marker.id = j
                    marker.action = marker.DELETE
                    marker_array.markers.append(marker)

            self.published_waypoints = current_waypoints

            if msg.is_blocked:

                path = PathWrapper(msg.waypoints)
                pose = path.get_pose_at_distance(stopping_point_distance)

                if collision_point_category == CollisionPoints.GOAL_POINT:
                    color = ColorRGBA(0.9, 0.9, 0.9, 0.2)       # white - goal point
                elif msg.target_object_speed < self.stopped_speed_limit:
                    color = ColorRGBA(1.0, 0.0, 0.0, 0.5)       # red - obstacle in front and very slow
                else:
                    color = ColorRGBA(1.0, 1.0, 0.0, 0.5)       # yellow - follow obstacle

                # "Stopping point" - obstacle that currently causes the smallest target velocity
                marker = Marker(header=msg.header)
                marker.ns = "Stopping point"
                marker.id = 0
                marker.type = marker.CUBE
                marker.action = marker.ADD
                marker.pose = pose
                marker.pose.position.z += 1.0
                marker.scale.x = 0.3
                marker.scale.y = 5.0
                marker.scale.z = 2.5
                marker.color = color
                marker.lifetime = rospy.Duration(0.3)
                marker_array.markers.append(marker)
            else:
                marker = Marker(header=msg.header)
                marker.ns = "Stopping point"
                marker.id = 0
                marker.action = marker.DELETE
                marker_array.markers.append(marker)

            if lb_safety_box_marker_array is not None and self.lane_boundary_safety_box_pub.get_num_connections() > 0:
                narrow_safety_box_triangle_points = triangulate_linestring(linestring, self.narrow_safety_box_width, z_offset=0.1)

                marker = Marker(header=msg.header)
                marker.ns = "Lane boundary safety box"
                marker.type = marker.TRIANGLE_LIST
                marker.action = marker.ADD
                marker.id = 0
                marker.scale.x = 1.0
                marker.scale.y = 1.0
                marker.scale.z = 1.0
                marker.pose.orientation.w = 1.0
                marker.color = LB_SAFETY_BOX_COLOR
                marker.lifetime = rospy.Duration(0.3)
                marker.colors = [LB_SAFETY_BOX_COLOR] * len(narrow_safety_box_triangle_points)
                marker.points = narrow_safety_box_triangle_points
                lb_safety_box_marker_array.markers.append(marker)

        # delete markers if local path not created
        else:

            marker = Marker(header=msg.header)
            marker.ns = "Stopping lateral distance"
            marker.id = 0
            marker.action = marker.DELETE
            marker_array.markers.append(marker)

            marker = Marker(header=msg.header)
            marker.ns = "Stopping point"
            marker.id = 0
            marker.action = marker.DELETE
            marker_array.markers.append(marker)

            if self.published_waypoints > 0:
                marker = Marker(header=msg.header)
                marker.ns = "Velocity label"
                marker.id = 0
                marker.action = marker.DELETEALL
                marker_array.markers.append(marker)

            self.published_waypoints = 0

            if lb_safety_box_marker_array is not None and self.lane_boundary_safety_box_pub.get_num_connections() > 0:
                marker = Marker(header=msg.header)
                marker.ns = "Lane boundary safety box"
                marker.id = 0
                marker.action = marker.DELETE
                lb_safety_box_marker_array.markers.append(marker)

        self.local_path_markers_pub.publish(marker_array)
        
        if lb_safety_box_marker_array is not None and self.lane_boundary_safety_box_pub.get_num_connections() > 0:
            self.lane_boundary_safety_box_pub.publish(lb_safety_box_marker_array)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('local_path_visualizer', log_level=rospy.INFO)
    node = LocalPathVisualizer()
    node.run()