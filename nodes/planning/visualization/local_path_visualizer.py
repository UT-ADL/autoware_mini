#!/usr/bin/env python3

import rospy
from autoware_msgs.msg import Lane
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA

class LocalPathVisualizer:
    def __init__(self):

        # Parameters
        self.car_safety_radius = rospy.get_param("/planning/local_planner/car_safety_radius", 1.3)

        # Publishers
        self.local_path_markers_pub = rospy.Publisher('local_path_markers', MarkerArray, queue_size=1, latch=True)

        # Subscribers
        rospy.Subscriber('local_path', Lane, self.local_path_callback, queue_size=1)

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

        self.local_path_markers_pub.publish(marker_array)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('local_path_visualizer', log_level=rospy.INFO)
    node = LocalPathVisualizer()
    node.run()