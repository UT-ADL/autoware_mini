#!/usr/bin/env python

import math
import rospy
import message_filters
import tf
from tf2_ros import TransformBroadcaster

from novatel_oem7_msgs.msg import INSPVA, BESTPOS
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion, TransformStamped


class NovatelOem7Localizer:
    def __init__(self):

        # Parameters
        self.coordinate_transformer = rospy.get_param("~coordinate_transformer", "utm")
        self.use_msl_height = rospy.get_param("~use_msl_height", True)
        self.use_custom_origin = rospy.get_param("~use_custom_origin", True)

        # variable to store undulation value from bestpos message
        self.undulation = 0.0

        # initialize coordinate_transformer
        if self.coordinate_transformer == "utm":
            import WGS84ToUTMTransformer
            self.transfromer = WGS84ToUTMTransformer.WGS84ToUTMTransformer(self.use_custom_origin)
        elif self.coordinate_transformer == "lest97":
            import WGS84ToLest97Transformer
            self.transfromer = WGS84ToLest97Transformer.WGS84ToLest97Transformer(self.use_custom_origin)
        else:
            rospy.logfatal("novatel_localizer - coordinate_transformer not supported: %s ", str(self.coordinate_transformer))
            exit(1)

        # Subscribers
        if self.use_msl_height:
            self.bestpos_sub = rospy.Subscriber('/novatel/oem7/bestpos', BESTPOS, self.bestpos_callback)
        self.inspva_sub = message_filters.Subscriber('/novatel/oem7/inspva', INSPVA)
        self.imu_sub = message_filters.Subscriber('/gps/imu', Imu)
        
        # Sync 2 main source topics in callback
        ts = message_filters.ApproximateTimeSynchronizer([self.inspva_sub, self.imu_sub], queue_size=10, slop=0.005)
        ts.registerCallback(self.data_callback)

        # Publishers
        self.current_pose_pub = rospy.Publisher('/current_pose', PoseStamped, queue_size=1)
        self.current_velocity_pub = rospy.Publisher('/current_velocity', TwistStamped, queue_size=1)

        # output localizer settings to console
        rospy.loginfo("novatel_oem7_localizer - coordinate_transformer: %s ", str(self.coordinate_transformer))
        rospy.loginfo("novatel_oem7_localizer - use_msl_height: %s ", str(self.use_msl_height))
        rospy.loginfo("novatel_oem7_localizer - use_custom_origin: %s ", str(self.use_custom_origin))


    def data_callback(self, inspva_msg, imu_msg):

        stamp = inspva_msg.header.stamp

        # transform GNSS coordinates and correct azimuth
        x, y = self.transfromer.transform_lat_lon(inspva_msg.latitude, inspva_msg.longitude, inspva_msg.height)
        azimuth = self.transfromer.correct_azimuth(inspva_msg.latitude, inspva_msg.longitude, inspva_msg.azimuth)

        velocity = calculate_velocity(inspva_msg.east_velocity, inspva_msg.north_velocity)

        # angles from GNSS (degrees) need to be converted to orientation (quaternion) in map frame
        orientation = convert_angles_to_orientation(inspva_msg.roll, inspva_msg.pitch, azimuth)

        # inspva_msg contains ellipsoid height if msl (mean sea level) height is wanted then undulation is subtracted
        height = inspva_msg.height
        if self.use_msl_height == True:
            height -= self.undulation

        # get IMU angular speeds for /current_velocity topic
        ang_vel_x = imu_msg.angular_velocity.x
        ang_vel_y = imu_msg.angular_velocity.y
        ang_vel_z = imu_msg.angular_velocity.z

        # Publish 
        self.publish_current_pose(stamp, x, y, height, orientation)
        self.publish_current_velocity(stamp, velocity, ang_vel_x, ang_vel_y, ang_vel_z)
        self.publish_map_to_baselink_tf(stamp, x, y, height, orientation)

    def bestpos_callback(self, bestpos_msg):
        self.undulation = bestpos_msg.undulation


    def publish_current_pose(self, stamp, x, y, height, orientation):

        pose_msg = PoseStamped()

        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = "map"

        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
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


    def publish_map_to_baselink_tf(self, stamp, x, y, height, orientation):
            
        br = TransformBroadcaster()
        t = TransformStamped()

        t.header.stamp = stamp
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = height
        t.transform.rotation = orientation

        br.sendTransform(t)


    def run(self):
        rospy.spin()


# Helper functions

def calculate_velocity(x_vel, y_vel):
    return math.sqrt(y_vel * y_vel + x_vel * x_vel)


def convert_angles_to_orientation(roll, pitch, yaw):
    
    # convert angles to radians
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)

    roll, pitch, yaw = convertAzimuthToENU(roll, pitch, yaw)
    q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    orientation = Quaternion(q[0], q[1], q[2], q[3])

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


if __name__ == '__main__':
    rospy.init_node('novatel_oem7_localizer', log_level=rospy.INFO)
    node = NovatelOem7Localizer()
    node.run()
