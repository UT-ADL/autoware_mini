#!/usr/bin/env python3

import rospy
from autoware_msgs.msg import Lane, WaypointState
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA

class GlobalPathVisualizer:
    def __init__(self):

        # Publishers
        self.global_path_markers_pub = rospy.Publisher('global_path_markers', MarkerArray, queue_size=10, latch=True, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber('global_path', Lane, self.global_path_callback, queue_size=None, tcp_nodelay=True)

    def global_path_callback(self, lane):
        marker_array = MarkerArray()

        if len(lane.waypoints) == 0:
            # create marker_array to delete all visualization markers
            marker = Marker()
            marker.header.frame_id = lane.header.frame_id
            marker.action = Marker.DELETEALL
            marker_array.markers.append(marker)

        else:
            # Pose arrows
            for i, waypoint in enumerate(lane.waypoints):

                # color the arrows based on the waypoint steering_flag (blinker)
                if waypoint.wpstate.steering_state == WaypointState.STR_LEFT:
                    color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
                elif waypoint.wpstate.steering_state == WaypointState.STR_RIGHT:
                    color = ColorRGBA(0.0, 0.0, 1.0, 1.0)
                else:
                    color = ColorRGBA(0.0, 1.0, 0.0, 1.0)

                marker = Marker()
                marker.header.frame_id = lane.header.frame_id
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

            # line strips
            marker = Marker()
            marker.header.frame_id = lane.header.frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = "Path"
            marker.type = marker.LINE_STRIP
            marker.action = marker.ADD
            marker.id = 0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 1.5
            marker.color = ColorRGBA(0.9, 0.6, 1.0, 0.6)
            for waypoint in lane.waypoints:
                marker.points.append(waypoint.pose.pose.position)
            marker_array.markers.append(marker)

        self.global_path_markers_pub.publish(marker_array)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('global_path_visualizer', log_level=rospy.INFO)
    node = GlobalPathVisualizer()
    node.run()