#!/usr/bin/env python3

import math
import rospy
import tf
from tf2_ros import TransformBroadcaster

from novatel_oem7_msgs.msg import INSPVA, BESTPOS
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion, TransformStamped

from localization.WGS84ToUTMTransformer import WGS84ToUTMTransformer
from localization.WGS84ToLest97Transformer import WGS84ToLest97Transformer


class NovatelOem7Localizer:
    def __init__(self):

        # Parameters
        self.coordinate_transformer = rospy.get_param("coordinate_transformer", "utm")
        self.use_custom_origin = rospy.get_param("use_custom_origin", True)
        self.utm_origin_lat = rospy.get_param("utm_origin_lat")
        self.utm_origin_lon = rospy.get_param("utm_origin_lon")
        self.lest97_origin_northing = rospy.get_param("lest97_origin_northing")
        self.lest97_origin_easting = rospy.get_param("lest97_origin_easting")
        self.use_msl_height = rospy.get_param("~use_msl_height", True)
        self.child_frame = rospy.get_param("~child_frame", "base_link")

        # variable to store undulation value from bestpos message
        self.undulation = 0.0

        # initialize coordinate_transformer
        if self.coordinate_transformer == "utm":
            self.transformer = WGS84ToUTMTransformer(self.use_custom_origin, self.utm_origin_lat, self.utm_origin_lon)
        elif self.coordinate_transformer == "lest97":
            self.transformer = WGS84ToLest97Transformer(self.use_custom_origin, self.lest97_origin_northing, self.lest97_origin_easting)
        else:
            rospy.logfatal("novatel_localizer - coordinate_transformer not supported: %s ", str(self.coordinate_transformer))
            exit(1)

        # Publishers
        self.current_pose_pub = rospy.Publisher('current_pose', PoseStamped, queue_size=1)
        self.current_velocity_pub = rospy.Publisher('current_velocity', TwistStamped, queue_size=1)
        
        # Subscribers
        if self.use_msl_height:
            self.bestpos_sub = rospy.Subscriber('/novatel/oem7/bestpos', BESTPOS, self.bestpos_callback)
        self.inspva_sub = rospy.Subscriber('/novatel/oem7/inspva', INSPVA, self.inspva_callback)

        # output information to console
        rospy.loginfo("novatel_localizer - localizer initialized using %s coordinates", str(self.coordinate_transformer))


    def inspva_callback(self, inspva_msg):

        stamp = inspva_msg.header.stamp

        # transform GNSS coordinates and correct azimuth
        x, y = self.transformer.transform_lat_lon(inspva_msg.latitude, inspva_msg.longitude, inspva_msg.height)
        azimuth = self.transformer.correct_azimuth(inspva_msg.latitude, inspva_msg.longitude, inspva_msg.azimuth)

        velocity = calculate_velocity(inspva_msg.east_velocity, inspva_msg.north_velocity)

        # angles from GNSS (degrees) need to be converted to orientation (quaternion) in map frame
        orientation = convert_angles_to_orientation(inspva_msg.roll, inspva_msg.pitch, azimuth)

        # inspva_msg contains ellipsoid height if msl (mean sea level) height is wanted then undulation is subtracted
        height = inspva_msg.height
        if self.use_msl_height:
            height -= self.undulation

        # Publish 
        self.publish_current_pose(stamp, x, y, height, orientation)
        self.publish_current_velocity(stamp, velocity)
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


    def publish_current_velocity(self, stamp, velocity):
        
        vel_msg = TwistStamped()

        vel_msg.header.stamp = stamp
        vel_msg.header.frame_id = self.child_frame
        vel_msg.twist.linear.x = velocity

        self.current_velocity_pub.publish(vel_msg)


    def publish_map_to_baselink_tf(self, stamp, x, y, height, orientation):
            
        br = TransformBroadcaster()
        t = TransformStamped()

        t.header.stamp = stamp
        t.header.frame_id = "map"
        t.child_frame_id = self.child_frame

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
    x, y, z, w = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    return Quaternion(x, y, z, w)


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
