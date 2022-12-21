#!/usr/bin/env python3

import numpy as np
import csv

import rospy
import message_filters

from geometry_msgs.msg import PoseStamped, TwistStamped


class WaypointSaver:
    def __init__(self):

        # Parameters
        self.interval = rospy.get_param("~interval", 1.0)
        self.file_name = rospy.get_param("~file_name", "waypoints.csv")

        # Internal params
        self.written_x = 0  # last x coordinate written into text file, kept to calculate distance interval
        self.written_y = 0  # last y coordinate written into text file, kept to calculate distance interval
        self.wp_id = 0      # waypoint id - incremented when written into text file

        rospy.init_node('waypoint_saver', anonymous=True)

        # write column names to waypoints file (append mode)
        with open('/tmp/' + self.file_name, 'a') as f:
            f.write('wp_id, x, y, z, yaw, velocity, change_flag\n')

        # Subscribers
        self.current_pose_sub = message_filters.Subscriber('/current_pose', PoseStamped)
        self.current_velocity_sub = message_filters.Subscriber('/current_velocity', TwistStamped)

        # Sync 2 source topics in callback
        ts = message_filters.ApproximateTimeSynchronizer([self.current_pose_sub, self.current_velocity_sub], queue_size=10, slop=0.05)
        ts.registerCallback(self.data_callback)


    def data_callback(self, current_pose, current_velocity):
        
        x = current_pose.pose.position.x
        y = current_pose.pose.position.y
        z = current_pose.pose.position.z
        v = current_velocity.twist.linear.x

        self.write_to_waypoint_file(x, y, z, v)


    def write_to_waypoint_file(self, x, y, z, v):
        
        # distance between current and lastly written coordinates
        distance = np.sqrt(np.power(self.written_x - x, 2) + np.power(self.written_y - y, 2))

        if distance > self.interval:

            # wp_id, x, y, z, yaw, velocity, change_flag
            row = [self.wp_id, x, y, z, 0, v, 0]

            with open('/tmp/' + self.file_name, 'a') as f:
                writer = csv.writer(f)
                writer.writerow(row)

            # TODO - create markers for rviz
            # location written_x,y,z and pose is towards the current wp / or use yaw from current pose - needs to be calculated! 

            self.written_x = x
            self.written_y = y
            self.wp_id += 1

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = WaypointSaver()
    node.run()