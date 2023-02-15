#!/usr/bin/env python

import math
import csv
import rospy
import message_filters
import tf
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3
from visualization_msgs.msg import Marker
from pacmod_msgs.msg import SystemRptInt
from std_msgs.msg import ColorRGBA

class WaypointSaver:
    def __init__(self):

        # Parameters
        self.interval = rospy.get_param("~interval", 1.0)
        self.file_name = rospy.get_param("~file_name", "/tmp/waypoints.csv")

        # Internal params
        self.written_x = 0  # last x coordinate written into text file, kept to calculate distance interval
        self.written_y = 0  # last y coordinate written into text file, kept to calculate distance interval
        self.wp_id = 0  # waypoint index
        self.turn_signal = 0

        # open file in append mode and write the header row
        self.waypoint_file = open(self.file_name, 'a')
        self.writer = csv.writer(self.waypoint_file)
        self.writer.writerow(['x', 'y', 'z', 'yaw', 'velocity', 'change_flag', 'steering_flag', 'accel_flag', 'stop_flag', 'event_flag'])

        # Subscribers
        self.current_pose_sub = message_filters.Subscriber('/current_pose', PoseStamped)
        self.current_velocity_sub = message_filters.Subscriber('/current_velocity', TwistStamped)
        self.turn_rpt_sub = rospy.Subscriber('/pacmod/parsed_tx/turn_rpt', SystemRptInt, self.turn_rpt_callback)

        # Sync 2 source topics in callback
        ts = message_filters.TimeSynchronizer([self.current_pose_sub, self.current_velocity_sub], queue_size=10)
        ts.registerCallback(self.data_callback)

        # Publishers
        self.waypoint_marker_pub = rospy.Publisher('waypoint_saver_markers', Marker, queue_size=10)

        # loginfo
        rospy.loginfo("waypoint_saver - interval: %i m", self.interval)
        rospy.loginfo("waypoint_saver - save to %s ", str(self.file_name))


    def turn_rpt_callback(self, turn_rpt_msg):
        
        # turn_rpt message    convert to wpstate/steering_state
        # 0 - right turn      STR_RIGHT=2
        # 1 - straight        STR_STRAIGHT=3
        # 2 - left turn       STR_LEFT=1

        # convert from turn_rpt to wpstate/steering_state
        turn_rpt_to_wpstate = {0: 2, 1: 3, 2: 1}
        self.turn_signal = turn_rpt_to_wpstate[turn_rpt_msg.output]

    def data_callback(self, current_pose, current_velocity):
        
        x = current_pose.pose.position.x
        y = current_pose.pose.position.y

        # distance between current and last written waypoint coordinates
        distance = math.sqrt((self.written_x - x) ** 2 + (self.written_y - y) ** 2)

        if distance > self.interval:
            # calculate current_heading
            current_heading = self.get_current_heading(current_pose.pose.orientation)
            blinker = self.turn_signal
            # write data to waypoints.csv file
            self.write_to_waypoint_file(x, y, current_pose.pose.position.z, current_heading, current_velocity.twist.linear.x, blinker)
            # create and publish a marker of the waypoint
            self.publish_wp_marker(current_pose, current_velocity.twist.linear.x)

            # update stored values
            self.written_x = x
            self.written_y = y

            # increment wp_id
            self.wp_id += 1

    def get_current_heading(self, orientation):
        # convert quaternion to euler angles
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)

        return math.degrees(yaw)

    def write_to_waypoint_file(self, x, y, z, yaw, v, blinker):
        
        # x, y, z, yaw, velocity, change_flag, steering_flag, accel_flag, stop_flag, event_flag
        row = [x, y, z, yaw, v, 0, blinker, 0, 0, 0]
        self.writer.writerow(row)

    def publish_wp_marker(self, current_pose, v):

        if self.turn_signal == 1:
            color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
        elif self.turn_signal == 2:
            color = ColorRGBA(0.0, 0.0, 1.0, 1.0)
        else:
            color = ColorRGBA(0.0, 1.0, 0.0, 1.0)

        marker = Marker()
        marker.id = self.wp_id
        marker.header.frame_id = current_pose.header.frame_id
        marker.frame_locked = True
        marker.header.stamp = current_pose.header.stamp

        marker.scale = Vector3(x=0.5, y=0.1, z=0.1)
        marker.color = color
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