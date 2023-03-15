#!/usr/bin/env python

import rospy
from autoware_msgs.msg import Lane, WaypointState
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA

class WaypointVisualizer:
    def __init__(self):

        # will be taken from the received path message: lane.header.frame_id
        self.output_frame = None

        # Subscribers
        self.path_sub = rospy.Subscriber('path', Lane, self.waypoints_callback, queue_size=1)

        # Publishers
        self.waypoints_markers_pub = rospy.Publisher('path_markers', MarkerArray, queue_size=1, latch=True)

    def waypoints_callback(self, lane):
        self.output_frame = lane.header.frame_id
        self.publish_waypoints_markers(lane.waypoints)


    # Waypoints visualization in RVIZ
    def publish_waypoints_markers(self, waypoints):
        marker_array = MarkerArray()

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
        marker.scale.x = 1.5
        marker.color = ColorRGBA(0.1, 10, 1.0, 0.4)
        for waypoint in waypoints:
            marker.points.append(waypoint.pose.pose.position)
        marker_array.markers.append(marker)

        # publish markers
        self.waypoints_markers_pub.publish(marker_array)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('waypoint_visualizer', log_level=rospy.INFO)
    node = WaypointVisualizer()
    node.run()