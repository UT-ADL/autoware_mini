#!/usr/bin/env python

import math
import csv
import rospy
import message_filters
import tf
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

        # open file in append mode and write the header row
        self.waypoint_file = open(self.file_name, 'a')
        self.writer = csv.writer(self.waypoint_file)
        self.writer.writerow(['wp_id', 'x', 'y', 'z', 'yaw', 'velocity', 'change_flag'])

        # Subscribers
        self.current_pose_sub = message_filters.Subscriber('/current_pose', PoseStamped)
        self.current_velocity_sub = message_filters.Subscriber('/current_velocity', TwistStamped)

        # Sync 2 source topics in callback
        ts = message_filters.TimeSynchronizer([self.current_pose_sub, self.current_velocity_sub], queue_size=10)
        ts.registerCallback(self.data_callback)

        # Publishers
        self.waypoint_marker_pub = rospy.Publisher('waypoint_saver_markers', Marker, queue_size=10)

        # loginfo
        rospy.loginfo("waypoint_saver - interval: %i m", self.interval)
        rospy.loginfo("waypoint_saver - save to %s ", str(self.file_name))


    def data_callback(self, current_pose, current_velocity):
        
        x = current_pose.pose.position.x
        y = current_pose.pose.position.y

        # distance between current and last written waypoint coordinates
        distance = math.sqrt((self.written_x - x) ** 2 + (self.written_y - y) ** 2)

        if distance > self.interval:
            # calculate current_heading
            current_heading = self.get_current_heading(current_pose.pose.orientation)
            # write data to waypoints.csv file
            self.write_to_waypoint_file(x, y, current_pose.pose.position.z, current_heading, current_velocity.twist.linear.x)
            # create and publish a marker of the waypoint
            self.publish_wp_marker(current_pose, current_velocity.twist.linear.x)

            # update stored values
            self.written_x = x
            self.written_y = y
            self.wp_id += 1

    def get_current_heading(self, orientation):
        # convert quaternion to euler angles
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = math.degrees(euler[2])  # from x axis: ccw up to 180, cw down to -180 degrees

        # convert yaw to heading from y axis (north) and cw from 0 up to 360
        if yaw < 0:
            heading = abs(yaw) + 90
        else:
            heading = 90 - yaw
            if heading < 0:
                heading += 360

        return heading

    def write_to_waypoint_file(self, x, y, z, yaw, v):
        
        # wp_id, x, y, z, yaw, velocity, change_flag
        row = [self.wp_id, x, y, z, yaw, v, 0]
        self.writer.writerow(row)

    def publish_wp_marker(self, current_pose, v):
        # print("publih", pose, v)
        marker = Marker()
        marker.id = self.wp_id
        marker.header.frame_id = current_pose.header.frame_id
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
    rospy.init_node('waypoint_saver', log_level=rospy.INFO)
    node = WaypointSaver()
    node.run()