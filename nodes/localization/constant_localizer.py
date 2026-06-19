#!/usr/bin/env python3

import math
import traceback
import rospy
import tf2_ros

from geometry_msgs.msg import Pose, PoseStamped, TwistStamped, TransformStamped, PoseWithCovarianceStamped

from autoware_mini.geometry import get_heading_from_orientation
from autoware_mini.lanelet2 import load_lanelet2_map, get_height_at_position


class ConstantLocalizer:
    def __init__(self):

        # Parameters
        self.publish_rate = rospy.get_param("~publish_rate")
        self.lanelet2_map = load_lanelet2_map(rospy.get_param("~lanelet2_map_path"))

        # State - start at origin facing east
        self.pose = Pose()
        self.pose.orientation.w = 1.0

        # TF broadcaster
        self.br = tf2_ros.TransformBroadcaster()

        # Publishers
        self.current_pose_pub = rospy.Publisher('current_pose', PoseStamped, queue_size=1, tcp_nodelay=True)
        self.current_velocity_pub = rospy.Publisher('current_velocity', TwistStamped, queue_size=1, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.initialpose_callback, queue_size=1, tcp_nodelay=True)

        # Timer
        rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.timer_callback)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def initialpose_callback(self, msg):
        pose = msg.pose.pose
        pose.position.z = get_height_at_position(self.lanelet2_map, pose.position.x, pose.position.y, pose.position.z)
        self.pose = pose

        heading = get_heading_from_orientation(self.pose.orientation)
        rospy.loginfo("%s - new pose: (%.1f, %.1f, %.1f) heading: %.1f deg", rospy.get_name(),
                      self.pose.position.x, self.pose.position.y, self.pose.position.z, math.degrees(heading))

    def timer_callback(self, event):
        try:
            stamp = rospy.Time.now()

            # Publish current pose
            pose_msg = PoseStamped()
            pose_msg.header.stamp = stamp
            pose_msg.header.frame_id = "map"
            pose_msg.pose = self.pose
            self.current_pose_pub.publish(pose_msg)

            # Publish zero velocity
            vel_msg = TwistStamped()
            vel_msg.header.stamp = stamp
            vel_msg.header.frame_id = "base_link"
            self.current_velocity_pub.publish(vel_msg)

            # Broadcast map -> base_link transform
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = "map"
            t.child_frame_id = "base_link"
            t.transform.translation.x = self.pose.position.x
            t.transform.translation.y = self.pose.position.y
            t.transform.translation.z = self.pose.position.z
            t.transform.rotation = self.pose.orientation
            self.br.sendTransform(t)
        except Exception:
            rospy.logerr_throttle(10, "%s - Exception in callback: %s", rospy.get_name(), traceback.format_exc())

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('constant_localizer', log_level=rospy.INFO)
    node = ConstantLocalizer()
    node.run()
