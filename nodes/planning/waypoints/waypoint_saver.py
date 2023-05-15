#!/usr/bin/env python3

import math
import csv
import rospy
import message_filters
import tf

from autoware_msgs.msg import WaypointState, VehicleStatus
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

VEHICLE_STATUS_LAMP_TO_WAYPOINT_STATE_MAP = {
    0: WaypointState.STR_STRAIGHT,
    VehicleStatus.LAMP_LEFT: WaypointState.STR_LEFT,
    VehicleStatus.LAMP_RIGHT: WaypointState.STR_RIGHT,
    VehicleStatus.LAMP_HAZARD: 5
}

class WaypointSaver:
    def __init__(self):

        # Parameters
        self.interval = rospy.get_param("~interval")
        self.waypoints_file = rospy.get_param("~waypoints_file")

        # Internal params
        self.written_x = 0  # last x coordinate written into text file, kept to calculate distance interval
        self.written_y = 0  # last y coordinate written into text file, kept to calculate distance interval
        self.wp_id = 0  # waypoint index
        self.turn_signal = 0
        self.marker_array = MarkerArray()

        # open file in append mode and write the header row
        self.file = open(self.waypoints_file, 'w')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['x', 'y', 'z', 'yaw', 'velocity', 'change_flag', 'steering_flag', 'accel_flag', 'stop_flag', 'event_flag'])

        # Publishers
        self.waypoint_marker_pub = rospy.Publisher('path_markers', MarkerArray, queue_size=1)

        # Subscribers
        self.current_pose_sub = message_filters.Subscriber('/localization/current_pose', PoseStamped, queue_size=1)
        self.current_velocity_sub = message_filters.Subscriber('/localization/current_velocity', TwistStamped, queue_size=1)
        self.turn_rpt_sub = rospy.Subscriber('/vehicle/vehicle_status', VehicleStatus, self.vehicle_status_callback, queue_size=1)

        # Sync 2 source topics in callback
        ts = message_filters.ApproximateTimeSynchronizer([self.current_pose_sub, self.current_velocity_sub], queue_size=2, slop=0.02)
        ts.registerCallback(self.data_callback)

        # loginfo
        rospy.loginfo("%s - interval: %i m", rospy.get_name(), self.interval)
        rospy.loginfo("%s - save to %s ", rospy.get_name(), self.waypoints_file)


    def vehicle_status_callback(self, vehicle_status_msg):
        self.turn_signal = VEHICLE_STATUS_LAMP_TO_WAYPOINT_STATE_MAP[vehicle_status_msg.lamp]

    def data_callback(self, current_pose, current_velocity):
        
        x = current_pose.pose.position.x
        y = current_pose.pose.position.y

        # distance between current and last written waypoint coordinates
        distance = math.sqrt((self.written_x - x) ** 2 + (self.written_y - y) ** 2)

        if distance >= self.interval:
            # calculate current_heading
            current_heading = get_current_heading_degrees(current_pose.pose.orientation)
            # write data to waypoints.csv file
            self.write_to_waypoints_file(x, y, current_pose.pose.position.z, current_heading, current_velocity.twist.linear.x, self.turn_signal)
            # create and publish a marker of the waypoint
            self.publish_wp_marker(current_pose, current_velocity.twist.linear.x)

            # update stored values
            self.written_x = x
            self.written_y = y

            # increment wp_id
            self.wp_id += 1

    def write_to_waypoints_file(self, x, y, z, yaw, v, steering_flag):
        
        # x, y, z, yaw, velocity, change_flag, steering_flag, accel_flag, stop_flag, event_flag
        row = [x, y, z, yaw, v, 0, steering_flag, 0, 0, 0]
        self.writer.writerow(row)

    def publish_wp_marker(self, current_pose, v):

        if self.turn_signal == WaypointState.STR_LEFT:
            color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
        elif self.turn_signal == WaypointState.STR_RIGHT:
            color = ColorRGBA(0.0, 0.0, 1.0, 1.0)
        else:
            color = ColorRGBA(0.0, 1.0, 0.0, 1.0)

        # marker representing pose
        marker = Marker()
        marker.id = self.wp_id
        marker.header.frame_id = current_pose.header.frame_id
        marker.frame_locked = True
        marker.header.stamp = current_pose.header.stamp
        marker.ns = "Waypoint pose"
        marker.scale = Vector3(x=0.5, y=0.1, z=0.1)
        marker.color = color
        marker.pose = current_pose.pose
        marker.type = marker.ARROW
        marker.action = marker.ADD
        self.marker_array.markers.append(marker)

        # marker representing velocity label
        marker_label = Marker()
        marker_label.header.frame_id = current_pose.header.frame_id
        marker_label.header.stamp = current_pose.header.stamp
        marker_label.ns = "Velocity label"
        marker_label.id = self.wp_id
        marker_label.type = marker_label.TEXT_VIEW_FACING
        marker_label.action = marker_label.ADD
        marker_label.pose = current_pose.pose
        marker_label.scale.z = 0.5
        marker_label.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
        marker_label.text = str(round(v * 3.6, 1))
        self.marker_array.markers.append(marker_label)

        self.waypoint_marker_pub.publish(self.marker_array)


    def run(self):
        rospy.spin()

def get_current_heading_degrees(orientation):
    # convert quaternion to euler angles
    quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
    _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)

    return math.degrees(yaw)


if __name__ == '__main__':
    rospy.init_node('waypoint_saver', log_level=rospy.INFO)
    node = WaypointSaver()
    node.run()