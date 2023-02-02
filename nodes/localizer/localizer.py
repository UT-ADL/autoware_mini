#!/usr/bin/env python

import numpy as np
from pyproj import CRS, Transformer

import rospy
import message_filters
import tf
from tf2_ros import TransformBroadcaster

from novatel_oem7_msgs.msg import INSPVA, BESTPOS
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion, TransformStamped


class Localizer:
    def __init__(self):

        # Parameters
        # Options: lest97, mgrs
        self.coordinate_system = rospy.get_param("~coordinate_system", 'lest97')
        self.use_msl_height = rospy.get_param("~use_msl_height", True)
        self.use_custom_origin = rospy.get_param("~use_custom_origin", False)

        # store undulation value from bestpos message
        self.undulation = 0.0

        # initialize coordinate_transformer
        if self.coordinate_system == 'lest97':
            self.coord_transformer = WGS84ToLest97Transformer(self.use_custom_origin)
        elif self.coordinate_system == 'utm':
            self.coord_transformer = WGS84ToUTMTransformer(self.use_custom_origin)

        # Subscribers
        if self.use_msl_height:
            self.bestpos_sub = rospy.Subscriber('/novatel/oem7/bestpos', BESTPOS, self.bestpos_callback)
        self.inspva_sub = message_filters.Subscriber('/novatel/oem7/inspva', INSPVA)
        self.imu_sub = message_filters.Subscriber('/gps/imu', Imu)
        
        # Sync 2 main source topics in callback
        ts = message_filters.ApproximateTimeSynchronizer([self.inspva_sub, self.imu_sub], queue_size=10, slop=0.005)
        ts.registerCallback(self.data_callback)

        # Publishers
        self.current_pose_pub = rospy.Publisher('/current_pose', PoseStamped, queue_size=10)
        self.current_velocity_pub = rospy.Publisher('/current_velocity', TwistStamped, queue_size=10)

        # output localizer settings to console
        rospy.loginfo("Localizer - coordinate_system: %s ", self.coordinate_system)
        rospy.loginfo("Localizer - use_msl_height: %s ", str(self.use_msl_height))
        rospy.loginfo("Localizer - use_custom_origin: %s ", str(self.use_custom_origin))


    def data_callback(self, inspva_msg, imu_msg):

        stamp = inspva_msg.header.stamp

        # inspva_msg contains ellipsoid height if msl (mean sea level) height is wanted then undulation is subtracted
        height = inspva_msg.height
        if self.use_msl_height == True:
            height -= self.undulation

        # transform lat lon coordinates according to created transformer
        pos_x, pos_y = self.coord_transformer.transform(inspva_msg.latitude, inspva_msg.longitude)
        azimuth = self.coord_transformer.azimuth_correction(inspva_msg.longitude, inspva_msg.azimuth)

        velocity = calculate_velocity(inspva_msg.east_velocity, inspva_msg.north_velocity)

        # angles from GNSS (degrees) need to be converted to orientation (quaternion) in map frame
        orientation = convert_angles_to_orientation(inspva_msg.roll, inspva_msg.pitch, azimuth)

        # get IMU angular speeds for Current velocity!
        ang_vel_x = imu_msg.angular_velocity.x
        ang_vel_y = imu_msg.angular_velocity.y
        ang_vel_z = imu_msg.angular_velocity.z

        # Publish 
        self.publish_current_pose(stamp, pos_x, pos_y, height, orientation)
        self.publish_current_velocity(stamp, velocity, ang_vel_x, ang_vel_y, ang_vel_z)
        self.publish_map_to_baselink_tf(stamp, pos_x, pos_y, height, orientation)

    def bestpos_callback(self, bestpos_msg):
        self.undulation = bestpos_msg.undulation


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

        vel_msg.twist.linear.x = velocity
        vel_msg.twist.linear.y = 0.0
        vel_msg.twist.linear.z = 0.0

        vel_msg.twist.angular.x = ang_vel_x
        vel_msg.twist.angular.y = ang_vel_y
        vel_msg.twist.angular.z = ang_vel_z

        self.current_velocity_pub.publish(vel_msg)


    def publish_map_to_baselink_tf(self, stamp, pos_x, pos_y, height, orientation):
            
        br = TransformBroadcaster()
        t = TransformStamped()

        t.header.stamp = stamp
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"

        t.transform.translation.x = pos_x
        t.transform.translation.y = pos_y
        t.transform.translation.z = height
        t.transform.rotation = orientation

        br.sendTransform(t)


    def run(self):
        rospy.spin()


class WGS84ToLest97Transformer:
    
    # https://epsg.io/3301
    # axes: northing, easting

    def __init__(self, use_custom_origin, origin_x=6465000, origin_y=650000):
        
        self.use_custom_origin = use_custom_origin
        self.origin_x = origin_x
        self.origin_y = origin_y

        self.crs_wgs84 = CRS.from_epsg(4326)
        self.crs_lest97 = CRS.from_epsg(3301)
        self.transformer = Transformer.from_crs(self.crs_wgs84, self.crs_lest97)


        # TODO - replace with more general solution, like pyproj get_factors.meridian_convergence
        # Stuff necessary to calc meridian convergence
        # flattening of GRS-80  ellipsoid
        f = 1 / 298.257222101
        er = (2.0 * f) - (f * f)
        e = np.sqrt(er)

        # 2 standard parallels of lest_97
        B1 = 58.0 * (np.pi / 180)
        B2 = (59.0 + 20.0 / 60.0) * (np.pi / 180)

        # calculate constant
        m1 = np.cos(B1) / np.sqrt(1.0 - er * np.power(np.sin(B1), 2))
        m2 = np.cos(B2) / np.sqrt(1.0 - er * np.power(np.sin(B2), 2))
        
        t1 = np.sqrt(((1.0 - np.sin(B1)) / (1.0 + np.sin(B1))) * np.power((1.0 + e * np.sin(B1)) / (1.0 - e * np.sin(B1)), e))
        t2 = np.sqrt(((1.0 - np.sin(B2)) / (1.0 + np.sin(B2))) * np.power((1.0 + e * np.sin(B2)) / (1.0 - e * np.sin(B2)), e))

        self.convergence_const = (np.log(m1) - np.log(m2)) / (np.log(t1) - np.log(t2))
        self.refernece_meridian = 24.0

    def transform(self, lat, lon):
        coords = self.transformer.transform(lat, lon)
        
        # axes: northing, easting
        if self.use_custom_origin:
            northing = coords[0] - self.origin_x
            easting = coords[1] - self.origin_y
        else:
            northing = coords[0]
            easting = coords[1]

        # return fist easting, then northing to match x and y in ROS map frame
        return easting, northing

    def azimuth_correction(self, lon, azimuth):
        #print("correction  : ", (self.convergence_const * (lon - self.refernece_meridian)))
        return azimuth - (self.convergence_const * (lon - self.refernece_meridian))

class WGS84ToUTMTransformer:

    # https://epsg.io/32635
    # axes: easting, northing

    def __init__(self, use_custom_origin, origin_x=58.384565, origin_y=26.725676):
        
        self.use_custom_origin = use_custom_origin
        self.origin_x = origin_x
        self.origin_y = origin_y

        self.crs_wgs84 = CRS.from_epsg(4326)
        self.crs_utm = CRS.from_epsg(32635)
        self.transformer = Transformer.from_crs(self.crs_wgs84, self.crs_utm)

        # transform origin from WGS84 to UTM
        self.origin_x, self.origin_y = self.transform(self.origin_x, self.origin_y)
        print(self.origin_x, self.origin_y)

    def transform(self, lat, lon):
        coords = self.transformer.transform(lat, lon)
        
        if self.use_custom_origin:
            northing = coords[1] - self.origin_y
            easting = coords[0] - self.origin_x
        else:
            northing = coords[1]
            easting = coords[0]

        # return fist easting, then northing to match x and y in ROS map frame
        return easting, northing

    def azimuth_correction(self, lon, azimuth):
        # TODO - implement azimuth correction for UTM
        return azimuth


# Helper functions

def calculate_velocity(x_vel, y_vel):
    return np.sqrt(y_vel * y_vel + x_vel * x_vel)


def convert_angles_to_orientation(roll, pitch, yaw):
    
    # convert angles to radians
    roll = np.radians(roll)
    pitch = np.radians(pitch)
    yaw = np.radians(yaw)

    roll, pitch, yaw = convertAzimuthToENU(roll, pitch, yaw)
    q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    orientation = Quaternion(q[0], q[1], q[2], q[3])

    return orientation


def convertAzimuthToENU(roll, pitch, yaw):

    # These transforms are taken from gpsins_localizer_nodelet.cpp
    # Convert from Azimuth (CW from North) to ENU (CCW from East)
    yaw = -yaw + np.pi/2

    # Clamp within 0 to 2 pi
    if yaw > 2 * np.pi:
        yaw = yaw - 2 * np.pi
    elif yaw < 0:
        yaw += 2 * np.pi
    
    # Novatel GPS uses different vehicle body frame (y forward, x right, z up)
    pitch = -pitch

    return roll, pitch, yaw


if __name__ == '__main__':
    rospy.init_node('gnss_localizer', anonymous=True, log_level=rospy.INFO)
    node = Localizer()
    node.run()
