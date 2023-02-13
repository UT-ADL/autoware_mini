#!/usr/bin/env python

import os
import rospy
import csv
import tf
import math

from autoware_msgs.msg import Lane, Waypoint
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA

class WaypointLoader:
    def __init__(self):

        # Parameters
        self.waypoints_file = rospy.get_param("~waypoints_file")
        self.output_frame = rospy.get_param("~output_frame", "map")
        self.publish_markers = rospy.get_param("~publish_markers", True)

        # Publishers
        self.waypoints_pub = rospy.Publisher('/waypoints', Lane, queue_size=1, latch=True)
        self.waypoints_markers_pub = rospy.Publisher('/waypoint_markers', MarkerArray, queue_size=1, latch=True)

        # loginfo and processing
        rospy.loginfo("waypoint_loader - loading waypoints from file: %s ", self.waypoints_file)
        self.waypoints = self.load_waypoints(self.waypoints_file)
        self.publish_waypoints(self.waypoints)

        if self.publish_markers:
            self.publish_waypoints_markers(self.waypoints)

        rospy.loginfo("waypoint_loader - waypoints are published ")


    def load_waypoints(self, waypoints_file):
        
        # load waypoints from file
        with open(waypoints_file, 'r') as f:
            reader = csv.reader(f)
            # skip header
            next(reader)
            waypoints = []

            for row in reader:
                # create waypoint
                waypoint = Waypoint()
                # 0      1  2  3  4    5         6          
                # wp_id, x, y, z, yaw, velocity, change_flag
                # set waypoint values
                waypoint.gid = int(row[0])
                waypoint.pose.pose.position.x = float(row[1])
                waypoint.pose.pose.position.y = float(row[2])
                waypoint.pose.pose.position.z = float(row[3])
                waypoint.twist.twist.linear.x = float(row[5]) / 3.6 # convert to m/s

                # convert yaw (contains heading in waypoints file) to quaternion
                q = self.get_quaternion_from_heading(float(row[4]))
                waypoint.pose.pose.orientation.x = q[0]
                waypoint.pose.pose.orientation.y = q[1]
                waypoint.pose.pose.orientation.z = q[2]
                waypoint.pose.pose.orientation.w = q[3]

                # set waypoint flags
                waypoint.change_flag = int(row[6])

                waypoints.append(waypoint)

        return waypoints

    def get_quaternion_from_heading(self, heading):
        # transform heading from (north y-axis: cw in deg up to 360) to (x-axis: ccw up to 180, cw down to -180 degrees)
        # will correct for cw down to -180 degrees
        yaw = 90 - heading
        # correct for 2nd ccw sector
        if yaw < -180:
            yaw = -(yaw + 360)

        quaternion = tf.transformations.quaternion_from_euler(0, 0, math.radians(yaw))

        return quaternion

    def publish_waypoints(self, waypoints):
        lane = Lane()
        lane.header.frame_id = self.output_frame
        lane.header.stamp = rospy.Time.now()
        lane.waypoints = waypoints
        self.waypoints_pub.publish(lane)

    # Waypoints visualization in RVIZ
    def publish_waypoints_markers(self, waypoints):
        marker_array = MarkerArray()

        # Pose arrows
        for i, waypoint in enumerate(waypoints):
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
            marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)
            marker_array.markers.append(marker)

        # velocity labels
        for i, waypoint in enumerate(waypoints):
            marker = Marker()
            marker.header.frame_id = self.output_frame
            marker.header.stamp = rospy.Time.now()
            marker.ns = "Velocity lables"
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
        marker.scale.x = 0.5
        marker.color = ColorRGBA(0.4, 10, 1.0, 0.4)
        for i, waypoint in enumerate(waypoints):
            marker.points.append(waypoint.pose.pose.position)
        marker_array.markers.append(marker)

        # publish markers
        self.waypoints_markers_pub.publish(marker_array)


    def run(self):
        rospy.spin()
        

if __name__ == '__main__':
    rospy.init_node('waypoint_loader', log_level=rospy.INFO)
    node = WaypointLoader()
    node.run()