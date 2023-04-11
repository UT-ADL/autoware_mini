#!/usr/bin/env python3

import rospy
from autoware_msgs.msg import Lane, WaypointState
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA

class WaypointVisualizer:
    def __init__(self):

        # Parameters
        self.car_safety_width = rospy.get_param("/planning/local_planner/car_safety_width", 1.3)

        # will be taken from the received path message: lane.header.frame_id
        self.output_frame = None

        # Publishers
        self.global_path_markers_pub = rospy.Publisher('global_path_markers', MarkerArray, queue_size=1, latch=True)
        self.smoothed_path_markers_pub = rospy.Publisher('smoothed_path_markers', MarkerArray, queue_size=1, latch=True)
        self.local_path_markers_pub = rospy.Publisher('local_path_markers', MarkerArray, queue_size=1, latch=True)

        # Subscribers
        self.global_path_sub = rospy.Subscriber('global_path', Lane, self.global_path_callback, queue_size=1)
        self.smoothed_path_sub = rospy.Subscriber('smoothed_path', Lane, self.smoothed_path_callback, queue_size=1)
        self.local_path_sub = rospy.Subscriber('local_path', Lane, self.local_path_callback, queue_size=1)

    def global_path_callback(self, lane):
        self.output_frame = lane.header.frame_id
        self.publish_global_path_markers(lane.waypoints)

    def smoothed_path_callback(self, lane):
        self.output_frame = lane.header.frame_id
        self.publish_smoothed_path_markers(lane.waypoints)

    def local_path_callback(self, lane):
        self.output_frame = lane.header.frame_id
        self.publish_local_path_markers(lane.waypoints)

    def publish_global_path_markers(self, waypoints):
        marker_array = self.create_path_markers(waypoints)
        self.global_path_markers_pub.publish(marker_array)

    def publish_smoothed_path_markers(self, waypoints):
        marker_array = self.create_path_markers(waypoints)
        self.smoothed_path_markers_pub.publish(marker_array)

    def publish_local_path_markers(self, waypoints):
        marker_array = self.create_local_path_markers(waypoints)
        self.local_path_markers_pub.publish(marker_array)

    # Create standard path markers visualization for RVIZ
    def create_path_markers(self, waypoints):
        marker_array = MarkerArray()

        if len(waypoints) == 0:
            # create marker_array to delete all visualization markers
            marker = Marker()
            marker.header.frame_id = self.output_frame
            marker.action = Marker.DELETEALL
            marker_array.markers.append(marker)

        else:
            # Pose arrows
            for i, waypoint in enumerate(waypoints):

                # color the arrows based on the waypoint steering_flag (blinker)
                if waypoint.wpstate.steering_state == WaypointState.STR_LEFT:
                    color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
                elif waypoint.wpstate.steering_state == WaypointState.STR_RIGHT:
                    color = ColorRGBA(0.0, 0.0, 1.0, 1.0)
                else:
                    color = ColorRGBA(0.0, 1.0, 0.0, 1.0)

                marker = Marker()
                marker.header.frame_id = self.output_frame
                marker.header.stamp = rospy.Time.now()
                marker.ns = "Waypoint pose"
                marker.id = i
                marker.type = marker.ARROW
                marker.action = marker.ADD
                marker.pose = waypoint.pose.pose
                marker.scale.x = 0.4
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.color = color
                marker_array.markers.append(marker)

            # velocity labels
            for i, waypoint in enumerate(waypoints):
                marker = Marker()
                marker.header.frame_id = self.output_frame
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

            # line strips
            marker = Marker()
            marker.header.frame_id = self.output_frame
            marker.header.stamp = rospy.Time.now()
            marker.ns = "Path"
            marker.type = marker.LINE_STRIP
            marker.action = marker.ADD
            marker.id = 0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 1.5
            marker.color = ColorRGBA(0.9, 0.6, 1.0, 0.6)
            for waypoint in waypoints:
                marker.points.append(waypoint.pose.pose.position)
            marker_array.markers.append(marker)

        return marker_array

    def create_local_path_markers(self, waypoints):
        marker_array = MarkerArray()

        if len(waypoints) == 0:
            # create marker_array to delete all visualization markers
            marker = Marker()
            marker.header.frame_id = self.output_frame
            marker.action = Marker.DELETEALL
            marker_array.markers.append(marker)

        else:
            for i, waypoint in enumerate(waypoints):
                marker = Marker()
                marker.header.frame_id = self.output_frame
                marker.header.stamp = rospy.Time.now()
                marker.ns = "Waypoints"
                marker.id = i
                marker.type = marker.CYLINDER
                marker.action = marker.ADD
                marker.pose = waypoint.pose.pose
                marker.scale.x = 2 * self.car_safety_width
                marker.scale.y = 2 * self.car_safety_width
                marker.scale.z = 0.3
                marker.color = ColorRGBA(waypoint.cost, 1.0 - waypoint.cost, 0.0, 0.2)
                marker_array.markers.append(marker)

            # velocity labels
            for i, waypoint in enumerate(waypoints):
                marker = Marker()
                marker.header.frame_id = self.output_frame
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

        return marker_array


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('waypoint_visualizer', log_level=rospy.INFO)
    node = WaypointVisualizer()
    node.run()