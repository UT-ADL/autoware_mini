#!/usr/bin/env python3
#
# Copyright (c) 2023 Autonomous Driving Lab (ADL), University of Tartu.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
ground truth localization. Publishes the following topics:
    current_velocty (geometry_msgs::TwistStamped)
    current_pose    (geometry_msgs::PoseStamped)
"""
import rospy
import math
import tf

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from localization.SimulationToUTMTransformer import SimulationToUTMTransformer

class CarlaLocalizer:

    def __init__(self):

        # Node parameters
        self.use_offset = rospy.get_param("/carla/use_offset")
        use_custom_origin = rospy.get_param("/localization/use_custom_origin")
        utm_origin_lat = rospy.get_param("/localization/utm_origin_lat")
        utm_origin_lon = rospy.get_param("/localization/utm_origin_lon")

        # Internal parameters
        self.sim2utm_transformer = SimulationToUTMTransformer(use_custom_origin=use_custom_origin,
                                                              origin_lat=utm_origin_lat,
                                                              origin_lon=utm_origin_lon)
        # Publishers
        self.pose_pub = rospy.Publisher('current_pose', PoseStamped, queue_size=1, tcp_nodelay=True)
        self.twist_pub = rospy.Publisher('current_velocity', TwistStamped, queue_size=1, tcp_nodelay=True)
        self.odom_pub = rospy.Publisher('odometry', Odometry, queue_size=1, tcp_nodelay=True)
        self.br = tf.TransformBroadcaster()

        # Subscribers
        rospy.Subscriber('/carla/odometry', Odometry, self.odometry_callback, queue_size=2, tcp_nodelay=True)


    def odometry_callback(self, msg):
        """
        callback odometry
        """

        if self.use_offset:
            new_pose = self.sim2utm_transformer.transform_pose(msg.pose.pose)
        else:
            new_pose = msg.pose.pose

        # Publish Transform
        self.br.sendTransform((new_pose.position.x, new_pose.position.y, new_pose.position.z),
                              (new_pose.orientation.x, new_pose.orientation.y, new_pose.orientation.z, new_pose.orientation.w),
                               msg.header.stamp, 'ego_vehicle', 'map')

        # Publish current pose
        current_pose = PoseStamped()
        current_pose.header.frame_id = "map"
        current_pose.header.stamp = msg.header.stamp
        current_pose.pose = new_pose
        self.pose_pub.publish(current_pose)

        # Publish current velocity
        current_velocity = TwistStamped()
        current_velocity.header.frame_id = "ego_vehicle"
        current_velocity.header.stamp = msg.header.stamp
        current_velocity.twist = msg.twist.twist
        self.twist_pub.publish(current_velocity)

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = current_pose.header.frame_id
        odom.child_frame_id = current_velocity.header.frame_id
        odom.pose.pose = current_pose.pose
        odom.twist.twist = current_velocity.twist
        self.odom_pub.publish(odom)


    def run(self):
        rospy.spin()


if __name__ == '__main__':
    # log_level set to errors only
    rospy.init_node('carla_localizer', log_level=rospy.INFO)
    node = CarlaLocalizer()
    node.run()
