#!/usr/bin/env python3

import rospy
from autoware_msgs.msg import Lane
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
from helpers.waypoints import get_point_and_orientation_on_path_within_distance

class LocalPathVisualizer:
    def __init__(self):

        # Parameters
        self.car_safety_radius = rospy.get_param("car_safety_radius")
        self.current_pose_to_car_front = rospy.get_param("current_pose_to_car_front")
        self.braking_safety_distance = rospy.get_param("braking_safety_distance")

        # Publishers
        self.local_path_markers_pub = rospy.Publisher('local_path_markers', MarkerArray, queue_size=1, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber('local_path', Lane, self.local_path_callback, queue_size=1, buff_size=2**20, tcp_nodelay=True)

    def local_path_callback(self, lane):
        marker_array = MarkerArray()

        # create marker_array to delete previous visualization
        marker = Marker()
        marker.header.frame_id = lane.header.frame_id
        marker.action = Marker.DELETEALL
        marker_array.markers.append(marker)

        for i, waypoint in enumerate(lane.waypoints):
            marker = Marker()
            marker.header.frame_id = lane.header.frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = "Waypoints"
            marker.id = i
            marker.type = marker.CYLINDER
            marker.action = marker.ADD
            marker.pose = waypoint.pose.pose
            marker.scale.x = 2 * self.car_safety_radius
            marker.scale.y = 2 * self.car_safety_radius
            marker.scale.z = 0.3
            marker.color = ColorRGBA(waypoint.cost, 1.0 - waypoint.cost, 0.0, 0.2)
            marker_array.markers.append(marker)

        # velocity labels
        for i, waypoint in enumerate(lane.waypoints):
            marker = Marker()
            marker.header.frame_id = lane.header.frame_id
            marker.header.stamp = rospy.Time.now()
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
        if len(lane.waypoints) > 1 and lane.closest_object_distance > 0:

            stop_position, stop_orientation = get_point_and_orientation_on_path_within_distance(lane.waypoints, 0, lane.waypoints[0].pose.pose.position, lane.closest_object_distance + self.current_pose_to_car_front - self.braking_safety_distance)

            color = ColorRGBA(0.0, 1.0, 0.0, 0.5)
            if lane.is_blocked:
                color = ColorRGBA(1.0, 1.0, 0.0, 0.5)
                if lane.closest_object_velocity < 1.0:
                    color = ColorRGBA(1.0, 0.0, 0.0, 0.5)

            marker = Marker()
            marker.header.frame_id = lane.header.frame_id
            marker.header.stamp = rospy.Time.now()
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