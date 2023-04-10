#!/usr/bin/env python
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
        self.use_offset = rospy.get_param("~use_offset", default=True)
        use_custom_origin = rospy.get_param("/localization/use_custom_origin", True)
        utm_origin_lat = rospy.get_param("/localization/utm_origin_lat")
        utm_origin_lon = rospy.get_param("/localization/utm_origin_lon")

        # Internal parameters
        self.sim2utm_transformer = SimulationToUTMTransformer(use_custom_origin=use_custom_origin,
                                                              origin_lat=utm_origin_lat,
                                                              origin_lon=utm_origin_lon)
        # Publishers
        self.pose_pub = rospy.Publisher('current_pose', PoseStamped, queue_size=1)
        self.twist_pub = rospy.Publisher('current_velocity', TwistStamped, queue_size=1)
        self.odom_pub = rospy.Publisher('odometry', Odometry, queue_size=1)
        self.br = tf.TransformBroadcaster()

        # Subscribers
        rospy.Subscriber('/carla/odometry', Odometry, self.odometry_callback, queue_size=1)


    def odometry_callback(self, msg):
        """
        callback odometry
        """
        # Odometry Pose
        current_pose = PoseStamped()
        current_pose.header.frame_id = "map"
        current_pose.header.stamp = msg.header.stamp

        if self.use_offset:
            current_pose.pose = self.sim2utm_transformer.transform_pose(msg.pose.pose)
        else:
            current_pose.pose = msg.pose

        # Publish current pose
        self.pose_pub.publish(current_pose)

        # Publish current velocity
        current_velocity = TwistStamped()
        current_velocity.header.stamp = msg.header.stamp
        current_velocity.header.frame_id = "ego_vehicle"
        speed = msg.twist.twist.linear
        current_velocity.twist.linear.x = math.sqrt(speed.x**2 + speed.y**2 + speed.z**2)
        current_velocity.twist.angular = msg.twist.twist.angular
        self.twist_pub.publish(current_velocity)

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = current_pose.header.frame_id
        odom.child_frame_id = current_velocity.header.frame_id
        odom.pose.pose = current_pose.pose
        odom.twist.twist = current_velocity.twist
        self.odom_pub.publish(odom)

        # Publish Transform
        self.br.sendTransform((current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z),
                              (current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w),
                               current_pose.header.stamp, 'ego_vehicle', 'map')

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    # log_level set to errors only
    rospy.init_node('carla_localizer', log_level=rospy.INFO)
    node = CarlaLocalizer()
    node.run()
