#!/usr/bin/env python

import numpy as np
import csv

import rospy
import message_filters

from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3
from visualization_msgs.msg import Marker


class WaypointSaver:
    def __init__(self):

        # Parameters
        self.interval = rospy.get_param("~interval", 1.0)
        self.file_name = rospy.get_param("~file_name", "waypoints.csv")

        # Internal params
        self.written_x = 0  # last x coordinate written into text file, kept to calculate distance interval
        self.written_y = 0  # last y coordinate written into text file, kept to calculate distance interval
        self.wp_id = 0      # waypoint id - incremented when written into text file

        # write column names to waypoints file (append mode)
        with open('/tmp/' + self.file_name, 'a') as f:
            f.write('wp_id, x, y, z, yaw, velocity, change_flag, steering_flag, accel_flag, stop_flag, event_flag\n')

        # Subscribers
        self.current_pose_sub = message_filters.Subscriber('/current_pose', PoseStamped)
        self.current_velocity_sub = message_filters.Subscriber('/current_velocity', TwistStamped)

        # Sync 2 source topics in callback
        ts = message_filters.ApproximateTimeSynchronizer([self.current_pose_sub, self.current_velocity_sub], queue_size=10, slop=0.05)
        ts.registerCallback(self.data_callback)

        # Publishers
        self.waypoint_marker_pub = rospy.Publisher('waypoint_saver_markers', Marker, queue_size=10)

        # loginfo
        rospy.loginfo("WaypointSaver - interval: %i ", self.interval)
        rospy.loginfo("WaypointSaver - save to /tmp/%s ", str(self.file_name))


    def data_callback(self, current_pose, current_velocity):
        
        x = current_pose.pose.position.x
        y = current_pose.pose.position.y

        # distance between current and last written waypoint coordinates
        distance = np.sqrt(np.power(self.written_x - x, 2) + np.power(self.written_y - y, 2))

        if distance > self.interval:
            # write data to waypoints.csv file
            self.write_to_waypoint_file(x, y, current_pose.pose.position.z, current_velocity.twist.linear.x)
            # create and publish a marker of the waypoint
            self.publish_wp_marker(current_pose, current_velocity.twist.linear.x)

            # update stored values
            self.written_x = x
            self.written_y = y
            self.wp_id += 1

    def write_to_waypoint_file(self, x, y, z, v):
        
        # wp_id, x, y, z, yaw, velocity, change_flag, steering_flag, accel_flag, stop_flag, event_flag
        row = [self.wp_id, x, y, z, 0, v, 0, 0, 0, 0, 0]

        with open('/tmp/' + self.file_name, 'a') as f:
            writer = csv.writer(f)
            writer.writerow(row)

    def publish_wp_marker(self, current_pose, v):
        # print("publih", pose, v)
        marker = Marker()
        marker.id = self.wp_id
        marker.header.frame_id = "map"
        marker.frame_locked = True
        marker.header.stamp = current_pose.header.stamp

        marker.scale = Vector3(x=0.5, y=0.1, z=0.1)
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose = current_pose.pose
        marker.type = marker.ARROW
        marker.action = Marker.ADD

        # publish only one marker - currently not collected into array!
        self.waypoint_marker_pub.publish(marker)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('waypoint_saver', anonymous=True, log_level=rospy.INFO)
    node = WaypointSaver()
    node.run()