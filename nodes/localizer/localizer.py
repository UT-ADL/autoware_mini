#!/usr/bin/env python3

import rospy
import numpy as np
from pyproj import CRS, Transformer

from novatel_oem7_msgs.msg import INSPVA
from geometry_msgs.msg import PoseStamped, TwistStamped


class Localizer:
    def __init__(self):

        # Parameters
        # Options: lest97, mgrs
        self.coordinat_system = rospy.get_param("~coordinat_system", 'lest97')

        # Subscribers
        self.inspva_sub = rospy.Subscriber('/novatel/oem7/inspva', INSPVA, self.inspva_callback)

        # Publishers
        self.current_pose_pub = rospy.Publisher('/current_pose', PoseStamped, queue_size=10)
        self.current_velocity_pub = rospy.Publisher('/current_velocity', TwistStamped, queue_size=10)


        # initialize coordinate_transformer
        if self.coordinat_system == 'lest97':
            # Create transformer from gnss to lest97 coordinate system
            self.coord_tranformer = Transformer.from_crs(CRS.from_epsg(4326), CRS.from_epsg(3301))



    def inspva_callback(self, msg):

        stamp = msg.header.stamp
        # TODO - get undulation from BESTPOS to calculate msl height
        height = msg.height
        # transform lat lon coordinates according to created transformer
        coords = self.coord_tranformer.transform(msg.latitude, msg.longitude)
        velocity = np.sqrt(msg.north_velocity * msg.north_velocity + msg.east_velocity + msg.east_velocity)
        # get IMU angular speeds for Current velocity!
        
        # Publish 
        self.publish_current_pose(stamp, coords, height)
        self.publish_current_velocity(stamp, velocity)


    def publish_current_pose(self, stamp, coords, height):

        # Create and fill current_pose message
        pose_msg = PoseStamped()
        
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = "map"

        # TODO might need to swap x and y - or in callback (data)
        pose_msg.pose.position.x = coords[0]
        pose_msg.pose.position.y = coords[1]
        pose_msg.pose.position.z = height

        # pose_msg.pose.orientation.x = orient_x
        # pose_msg.pose.orientation.y = orient_y
        # pose_msg.pose.orientation.z = orient_z
        # pose_msg.pose.orientation.w = orient_w

        self.current_pose_pub.publish(pose_msg)


    def publish_current_velocity(self, stamp, velocity):
        
        vel_msg = TwistStamped()

        vel_msg.header.stamp = stamp
        vel_msg.header.frame_id = "base_link"

        vel_msg.twist.linear.x = velocity;
        vel_msg.twist.linear.y = 0.0;
        vel_msg.twist.linear.z = 0.0;

        # vel_msg.twist.angular.x = imu_msg->angular_velocity.x;
        # vel_msg.twist.angular.y = imu_msg->angular_velocity.y;
        # vel_msg.twist.angular.z = imu_msg->angular_velocity.z;

        self.current_velocity_pub.publish(vel_msg)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = Localizer()
    rospy.init_node('gnss_localizer', anonymous=True)
    node.run()
