#!/usr/bin/env python3

import numpy as np
import math
from pyproj import CRS, Transformer

import rospy
import message_filters

from novatel_oem7_msgs.msg import INSPVA, BESTPOS
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion



class Localizer:
    def __init__(self):

        # Parameters
        # Options: lest97, mgrs
        self.coordinate_system = rospy.get_param("~coordinate_system", 'lest97')
        self.use_msl_height = rospy.get_param("~use_msl_height", True)

        rospy.init_node('gnss_localizer', anonymous=True)

        # store undulation value from bestpos message
        self.undulation = 0.0

        # initialize coordinate_transformer
        if self.coordinate_system == 'lest97':
            # Create transformer from gnss to lest97 coordinate system
            crs_wgs84 = CRS.from_epsg(4326)
            crs_lest97 = CRS.from_epsg(3301)
            self.coord_tranformer = Transformer.from_crs(crs_wgs84, crs_lest97)

            # Stuff necessary to calc meridian convergence

            # flattening of GRS-80  ellipsoid
            f = 1 / 298.257222101
            er = (2.0 * f) - (f * f)
            e = math.sqrt(er)

            # 2 standard parallels of lest_97
            B1 = 58.0 * (math.pi / 180)
            B2 = (59.0 + 20.0 / 60.0) * (math.pi / 180)

            # calculate constant
            m1 = math.cos(B1) / math.sqrt(1.0 - er * math.pow(math.sin(B1), 2))
            m2 = math.cos(B2) / math.sqrt(1.0 - er * math.pow(math.sin(B2), 2))
            
            t1 = math.sqrt(((1.0 - math.sin(B1)) / (1.0 + math.sin(B1))) * math.pow((1.0 + e * math.sin(B1)) / (1.0 - e * math.sin(B1)), e))
            t2 = math.sqrt(((1.0 - math.sin(B2)) / (1.0 + math.sin(B2))) * math.pow((1.0 + e * math.sin(B2)) / (1.0 - e * math.sin(B2)), e))

            self.convergence_const = (math.log(m1) - math.log(m2)) / (math.log(t1) - math.log(t2))
            self.refernece_meridian = 24.0

        # Subscribers
        if self.use_msl_height == True:
            self.bestpos_sub = rospy.Subscriber('/novatel/oem7/bestpos', BESTPOS, self.bestpos_callback)

        self.inspva_sub = message_filters.Subscriber('/novatel/oem7/inspva', INSPVA)
        self.imu_sub = message_filters.Subscriber('/gps/imu', Imu)
        
        # Sync 2 main source topics in callback
        ts = message_filters.ApproximateTimeSynchronizer([self.inspva_sub, self.imu_sub], queue_size=10, slop=0.005)
        ts.registerCallback(self.data_callback)

        # Publishers
        self.current_pose_pub = rospy.Publisher('/current_pose', PoseStamped, queue_size=10)
        self.current_velocity_pub = rospy.Publisher('/current_velocity', TwistStamped, queue_size=10)


    def data_callback(self, inspva_msg, imu_msg):

        stamp = inspva_msg.header.stamp

        # inspva_msg contains ellipsoid height if msl hight is wanted then undulation is subtracted
        height = inspva_msg.height
        if self.use_msl_height == True:
            height -= self.undulation

        # transform lat lon coordinates according to created transformer
        coords = self.coord_tranformer.transform(inspva_msg.latitude, inspva_msg.longitude)
        # TODO - Remove tartu specific adjustments
        # swap x and y coordinates
        pos_x = coords[1] - 650000
        pos_y = coords[0] - 6465000
        # calculate velocity
        velocity = calculate_velocity(inspva_msg.east_velocity, inspva_msg.north_velocity)
        azimuth = inspva_msg.azimuth

        # TODO - correct the azimuth divergence. Raw azimuth from WGS84 - pointing along meridian
        # That is not correct azimuth for Lest97 system - needs to be corrected with respect to standard meridian and calculated divergence!
        # https://github.com/kimollivier/convergence
        if self.coordinate_system == "lest97":
            azimuth -= self.convergence_const * (inspva_msg.longitude - self.refernece_meridian)

        # angles from GNSS (degrees) need to be converted to orientation (quaternion) in map frame
        orientation = convert_angles_to_orientation(inspva_msg.roll, inspva_msg.pitch, azimuth)

        # get IMU angular speeds for Current velocity!
        ang_vel_x = imu_msg.angular_velocity.x
        ang_vel_y = imu_msg.angular_velocity.y
        ang_vel_z = imu_msg.angular_velocity.z

        # Publish 
        self.publish_current_pose(stamp, pos_x, pos_y, height, orientation)
        self.publish_current_velocity(stamp, velocity, ang_vel_x, ang_vel_y, ang_vel_z)

    def bestpos_callback(self, bestpos_msg):
        self.undulation = bestpos_msg.undulation
        print(self.undulation)


    def publish_current_pose(self, stamp, pos_x, pos_y, height, orientation):

        pose_msg = PoseStamped()
        
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = "map"

        pose_msg.pose.position.x = pos_x
        pose_msg.pose.position.y = pos_y
        pose_msg.pose.position.z = height
        pose_msg.pose.orientation = orientation

        self.current_pose_pub.publish(pose_msg)

    def publish_current_velocity(self, stamp, velocity, ang_vel_x, ang_vel_y, ang_vel_z):
        
        vel_msg = TwistStamped()

        vel_msg.header.stamp = stamp
        vel_msg.header.frame_id = "base_link"

        vel_msg.twist.linear.x = velocity;
        vel_msg.twist.linear.y = 0.0;
        vel_msg.twist.linear.z = 0.0;

        vel_msg.twist.angular.x = ang_vel_x
        vel_msg.twist.angular.y = ang_vel_y
        vel_msg.twist.angular.z = ang_vel_z

        self.current_velocity_pub.publish(vel_msg)


    def run(self):
        rospy.spin()


def calculate_velocity(x_vel, y_vel):

    return np.sqrt(y_vel * y_vel + x_vel * x_vel)


def convert_angles_to_orientation(roll, pitch, yaw):
    
    # convert angles to radians
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)

    roll, pitch, yaw = convertAzimuthToENU(roll, pitch, yaw)
    orientation = get_quaternion_from_euler(roll, pitch, yaw)

    return orientation


def convertAzimuthToENU(roll, pitch, yaw):

    # These transforms are taken from gpsins_localizer_nodelet.cpp

    # Convert from Azimuth (CW from North) to ENU (CCW from East)
    yaw = -yaw + math.pi/2

    # Clamp within 0 to 2 pi
    if yaw > 2 * math.pi:
        yaw = yaw - 2 * math.pi
    elif yaw < 0:
        yaw += 2 * math.pi
    
    # Novatel GPS uses different vehicle body frame (y forward, x right, z up)
    pitch = -pitch

    return roll, pitch, yaw


def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.

    Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    
    orientation = Quaternion(qx, qy, qz, qw)

    return orientation

if __name__ == '__main__':
    node = Localizer()
    node.run()
