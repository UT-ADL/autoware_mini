#!/usr/bin/env python3

import rospy
import math
import shapely

from autoware_mini.msg import Path, Log
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
from jsk_rviz_plugins.msg import OverlayText

from autoware_mini.path import PathWrapper
from autoware_mini.collision import CollisionPoints
from autoware_mini.geometry import get_orientation_from_heading
from autoware_mini.visualization import triangulate_linestring


COLLISION_POINT_CATEGORY_COLOR = {
    CollisionPoints.NO_OBSTACLES:                       "PaleGreen",
    CollisionPoints.GOAL_POINT:                         "PaleGreen",
    CollisionPoints.TRAFFIC_LIGHT_STOPLINE:             "LightCoral",
    CollisionPoints.STOPPED_OBSTACLE_ON_PATH:           "LightCoral",
    CollisionPoints.MOVING_OBSTACLE_ON_PATH:            "Khaki",
    CollisionPoints.COLLIDING_TRAJECTORY:               "LightCoral",
    CollisionPoints.MERGING_TRAJECTORY:                 "Khaki",
    CollisionPoints.OBJECT_ON_CROSSWALK:                "LightCoral",
    CollisionPoints.TRAJECTORY_ON_CROSSWALK:            "Khaki",
    CollisionPoints.YIELDING_TRAJECTORY:                "Khaki",
    CollisionPoints.STOP_LINE_FORCED_STOP:              "LightCoral"
}

LOCAL_PATH_COLOR = ColorRGBA(0.1, 1.0, 0.1, 0.6)

class LocalPathVisualizer:
    def __init__(self):

        # Parameters
        self.safety_box_width = rospy.get_param("safety_box_width")
        self.stopped_speed_limit = rospy.get_param("stopped_speed_limit")

        self.published_waypoints = 0
        self.planner_status_last_timestamp = None
        self.planner_status_last_category = None

        # Publishers
        self.local_path_markers_pub = rospy.Publisher('local_path_markers', MarkerArray, queue_size=1, tcp_nodelay=True)
        self.planner_status_pub = rospy.Publisher('/dashboard/planner_status', OverlayText, queue_size=1, tcp_nodelay=True, latch=True)
        self.planner_status_pub.publish(OverlayText(text=""))  # Initialize with empty text
        self.log_message_pub = rospy.Publisher('/dashboard/log_message', Log, queue_size=1, tcp_nodelay=True, latch=True)

        # Subscribers
        rospy.Subscriber('local_path', Path, self.local_path_callback, queue_size=1, buff_size=2**20, tcp_nodelay=True)

    def local_path_callback(self, msg):

        stopping_point_distance = max(msg.stopping_point_distance, 0.0)
        collision_point_category = msg.collision_point_category

        if self.planner_status_last_timestamp is None:
            self.planner_status_last_timestamp = msg.header.stamp

        marker_array = MarkerArray()

        if len(msg.waypoints) > 1:

            planner_status_text = f"<div style='text-align: center; color: {COLLISION_POINT_CATEGORY_COLOR[collision_point_category]};'>{CollisionPoints.COLLISION_POINT_CATEGORY_CAPTION[collision_point_category]}</div>"

            # Triangulate loal path
            linestring = shapely.linestrings([[p.position.x, p.position.y, p.position.z] for p in msg.waypoints])
            linestring = linestring.simplify(0.01)
            trangle_points = triangulate_linestring(linestring, self.safety_box_width, z_offset=0.1)

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
            marker.colors = [LOCAL_PATH_COLOR] * len(trangle_points)
            marker.points = trangle_points
            marker_array.markers.append(marker)

            # velocity labels
            current_waypoints = 0
            for i, waypoint in enumerate(msg.waypoints):
                marker = Marker(header=msg.header)
                marker.ns = "Velocity label"
                marker.id = i
                marker.type = marker.TEXT_VIEW_FACING
                marker.action = marker.ADD
                marker.pose.position = waypoint.position
                marker.pose.orientation = get_orientation_from_heading(waypoint.heading)
                marker.scale.z = 0.5
                marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
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
                elif msg.closest_object_velocity < self.stopped_speed_limit:
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
                marker_array.markers.append(marker)
            else:
                marker = Marker(header=msg.header)
                marker.ns = "Stopping point"
                marker.id = 0
                marker.action = marker.DELETE
                marker_array.markers.append(marker)

        # delete markers if local path not created
        else:

            planner_status_text = "<div style='text-align: center; color: gray;'>Waiting for path</div>"

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

            marker = Marker(header=msg.header)
            marker.ns = "Velocity label"
            marker.id = 0
            marker.action = marker.DELETEALL
            marker_array.markers.append(marker)

            self.published_waypoints = 0

        planner_status = OverlayText()
        planner_status.text = planner_status_text
        self.planner_status_pub.publish(planner_status)
        self.local_path_markers_pub.publish(marker_array)

        if self.planner_status_last_category != collision_point_category:
            if self.planner_status_last_category != None:
                log = Log()
                log.message = CollisionPoints.COLLISION_POINT_CATEGORY_CAPTION[self.planner_status_last_category]
                log.color = COLLISION_POINT_CATEGORY_COLOR[self.planner_status_last_category]
                log.duration = (msg.header.stamp - self.planner_status_last_timestamp).to_sec()
                self.log_message_pub.publish(log)

            # update last timestamp and category
            self.planner_status_last_timestamp = msg.header.stamp
            self.planner_status_last_category = collision_point_category

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('local_path_visualizer', log_level=rospy.INFO)
    node = LocalPathVisualizer()
    node.run()