#!/usr/bin/env python3

import rospy
import math
from autoware_msgs.msg import Lane
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
from helpers.waypoints import get_point_and_orientation_on_path_within_distance
from helpers.geometry import get_distance_between_two_points_2d, get_closest_point_on_line
from geometry_msgs.msg import Point, PoseStamped

class LocalPathVisualizer:
    def __init__(self):

        # Parameters
        self.stopping_lateral_distance = rospy.get_param("stopping_lateral_distance")
        self.slowdown_lateral_distance = rospy.get_param("slowdown_lateral_distance")
        self.current_pose_to_car_front = rospy.get_param("current_pose_to_car_front")
        self.braking_safety_distance = rospy.get_param("braking_safety_distance")

        # Publishers
        self.local_path_markers_pub = rospy.Publisher('local_path_markers', MarkerArray, queue_size=1, latch=True)

        # Subscribers
        rospy.Subscriber('local_path', Lane, self.local_path_callback, queue_size=1)
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1)

        self.current_pose = None

    def current_pose_callback(self, pose):
        self.current_pose = pose.pose


    def local_path_callback(self, lane):
        marker_array = MarkerArray()

        # create marker_array to delete previous visualization
        marker = Marker()
        marker.header.frame_id = lane.header.frame_id
        marker.action = Marker.DELETEALL
        marker_array.markers.append(marker)

        stamp = rospy.Time.now()

        points = []
        for waypoint in lane.waypoints:
            points.append(waypoint.pose.pose.position)
        
        distance_correction = 0.0
        if self.current_pose is not None and len(lane.waypoints) > 1:
            current_pose_on_path = get_closest_point_on_line(self.current_pose.position, points[0], points[1])
            distance_correction = get_distance_between_two_points_2d(current_pose_on_path, points[0])

        color = ColorRGBA(0.0, 1.0, 0.0, 0.3)

        # color RED if lane is blocked
        if lane.is_blocked:
            color = ColorRGBA(1.0, 0.0, 0.0, 0.3)

        # local path with stopping_lateral_distance
        marker = Marker()
        marker.header.frame_id = lane.header.frame_id
        marker.header.stamp = stamp
        marker.ns = "Stopping lateral distance"
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.id = 0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 2*self.stopping_lateral_distance
        marker.color = color
        marker.points = points
        marker_array.markers.append(marker)

        # color yellow if lane is blocked or obs in slowdown area
        if lane.is_blocked or lane.cost != 0.0:
            color = ColorRGBA(1.0, 1.0, 0.0, 0.3)

        # local path with slowdown_lateral_distance
        marker = Marker()
        marker.header.frame_id = lane.header.frame_id
        marker.header.stamp = stamp
        marker.ns = "Slowdown lateral distance"
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.id = 1
        marker.pose.orientation.w = 1.0
        marker.scale.x = 2*self.slowdown_lateral_distance
        marker.color = color
        marker.points = points
        marker_array.markers.append(marker)

        # velocity labels
        for i, waypoint in enumerate(lane.waypoints):
            marker = Marker()
            marker.header.frame_id = lane.header.frame_id
            marker.header.stamp = stamp
            marker.ns = "Velocity label"
            marker.id = i
            marker.type = marker.TEXT_VIEW_FACING
            marker.action = marker.ADD
            marker.pose = waypoint.pose.pose
            marker.scale.z = 0.5
            marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
            marker.text = str(round(waypoint.twist.twist.linear.x * 3.6, 1))
            marker_array.markers.append(marker)

        # stop position visualization
        if len(lane.waypoints) > 1 and (lane.is_blocked or lane.cost != 0.0):

            stop_position, stop_orientation = get_point_and_orientation_on_path_within_distance(lane.waypoints, 0, lane.waypoints[0].pose.pose.position, lane.closest_object_distance + self.current_pose_to_car_front - self.braking_safety_distance + distance_correction)

            color = ColorRGBA(0.0, 1.0, 0.0, 0.5)
            if lane.is_blocked:
                color = ColorRGBA(1.0, 1.0, 0.0, 0.5)
                if lane.closest_object_velocity < 1.0:
                    color = ColorRGBA(1.0, 0.0, 0.0, 0.5)

            marker = Marker()
            marker.header.frame_id = lane.header.frame_id
            marker.header.stamp = stamp
            marker.ns = "Stopping point"
            marker.id = 0
            marker.type = marker.CUBE
            marker.action = marker.ADD
            marker.pose.position.x = stop_position.x
            marker.pose.position.y = stop_position.y
            marker.pose.position.z = stop_position.z + 1.0
            marker.pose.orientation = stop_orientation
            marker.scale.x = 0.3
            marker.scale.y = 5.0
            marker.scale.z = 2.5
            marker.color = color
            marker_array.markers.append(marker)

        self.local_path_markers_pub.publish(marker_array)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('local_path_visualizer', log_level=rospy.INFO)
    node = LocalPathVisualizer()
    node.run()