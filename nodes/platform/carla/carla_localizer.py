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
import numpy as np

from ros_numpy import numpify, msgify
from tf2_ros import TransformBroadcaster, TransformListener, Buffer, TransformException

from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped, Pose
from nav_msgs.msg import Odometry
from localization.SimulationToUTMTransformer import SimulationToUTMTransformer

class CarlaLocalizer:

    def __init__(self):

        # Node parameters
        self.use_transformer = rospy.get_param("/carla_localization/use_transformer")
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

        # TF Broadcaster and Listener
        self.tf_broadcaster = TransformBroadcaster()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        # Wait for the static transform between base_link and ego_vehicle
        base_to_ego_static_transform = self.tf_buffer.lookup_transform("ego_vehicle", "base_link", rospy.Time(0), rospy.Duration(100))
        self.base_link_to_ego_matrix = numpify(base_to_ego_static_transform.transform)

        # Subscribers
        rospy.Subscriber('/carla/ego_vehicle/odometry', Odometry, self.odometry_callback, queue_size=2, tcp_nodelay=True)


    def odometry_callback(self, msg):
        """
        callback odometry
        """

        if self.use_transformer:
            new_pose = self.sim2utm_transformer.transform_pose(msg.pose.pose)
        else:
            new_pose = msg.pose.pose

        map_transform = TransformStamped()
        map_transform.header.stamp = msg.header.stamp
        map_transform.header.frame_id = "map"
        map_transform.child_frame_id = "ego_vehicle"
        map_transform.transform.translation = new_pose.position
        map_transform.transform.rotation = new_pose.orientation
        self.tf_broadcaster.sendTransform(map_transform)

        # Publish current velocity
        current_velocity = TwistStamped()
        current_velocity.header.frame_id = "base_link"
        current_velocity.header.stamp = msg.header.stamp
        current_velocity.twist = msg.twist.twist
        self.twist_pub.publish(current_velocity)

        # Make it pose of base_link instead of ego_vehicle
        new_pose_matrix = numpify(new_pose)
        pose_matrix = np.dot(new_pose_matrix, self.base_link_to_ego_matrix)
        pose = msgify(Pose, pose_matrix)

        # Publish current pose
        current_pose = PoseStamped()
        current_pose.header.frame_id = "map"
        current_pose.header.stamp = msg.header.stamp
        current_pose.pose = pose
        self.pose_pub.publish(current_pose)

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = current_pose.header.frame_id
        odom.child_frame_id = current_velocity.header.frame_id
        odom.pose.pose = pose
        odom.twist.twist = current_velocity.twist
        self.odom_pub.publish(odom)
     

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    # log_level set to errors only
    rospy.init_node('carla_localizer', log_level=rospy.INFO)
    node = CarlaLocalizer()
    node.run()
