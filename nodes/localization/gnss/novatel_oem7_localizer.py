#!/usr/bin/env python3

import math
import traceback
import rospy
import numpy as np
import lanelet2

import tf2_ros
from tf.transformations import quaternion_from_euler
from ros_numpy import numpify, msgify
from novatel_oem7_msgs.msg import INSPVA, BESTPOS
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion, TransformStamped, PoseWithCovarianceStamped, Pose
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty, EmptyResponse

from autoware_mini.transform import transform_pose, transform_to_pose, pose_to_transform
from autoware_mini.lanelet2 import load_lanelet2_map, get_height_at_position


class NovatelOem7Localizer:
    def __init__(self):

        # Parameters
        use_custom_origin = rospy.get_param("use_custom_origin")
        utm_origin_lat = rospy.get_param("utm_origin_lat")
        utm_origin_lon = rospy.get_param("utm_origin_lon")
        self.use_msl_height = rospy.get_param("~use_msl_height")
        self.offline_height = rospy.get_param("~offline_height")
        self.offline_azimuth = rospy.get_param("~offline_azimuth")
        self.offline_lat = rospy.get_param("~offline_lat")
        self.offline_lon = rospy.get_param("~offline_lon")
        self.parent_frame = rospy.get_param("~parent_frame")
        self.child_frame = rospy.get_param("~child_frame")
        self.broadcast_tf = rospy.get_param("~broadcast_tf")
        self.time_source = rospy.get_param("~time_source")
        self.gps_time_leap_seconds = rospy.get_param("~gps_time_leap_seconds")
        self.gps_time_sync_threshold = rospy.get_param("~gps_time_sync_threshold")
        self.lanelet2_map = load_lanelet2_map(rospy.get_param("~lanelet2_map_path"))

        if self.time_source not in ("ros", "gps", "auto"):
            raise ValueError(f"{rospy.get_name()} - 'time_source' must be one of 'ros', 'gps', or 'auto', not '{self.time_source}'")

        # variable to store undulation value from bestpos message
        self.undulation = 0.0
        self.current_pose = None
        self.relative_pose_matrix = None

        # initialize UTM projector using Lanelet2
        origin = lanelet2.io.Origin(utm_origin_lat, utm_origin_lon)
        self.projector = lanelet2.projection.UtmProjector(origin, use_custom_origin, False)
        self.central_meridian = (utm_origin_lon // 6.0) * 6.0 + 3.0

        # initialize tf2 buffer and listener for converting initialpose to map frame coordinates
        self.tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Publishers
        self.current_pose_pub = rospy.Publisher('current_pose', PoseStamped, queue_size=1, tcp_nodelay=True)
        self.current_velocity_pub = rospy.Publisher('current_velocity', TwistStamped, queue_size=1, tcp_nodelay=True)
        self.odometry_pub = rospy.Publisher('odometry', Odometry, queue_size=1, tcp_nodelay=True)

        # static base_footprint -> base_link offset, used to lift the repositioned ground-level pose to
        # base_link, as reported by the GNSS on the real vehicle. Generous timeout to tolerate the TF
        # (published from the URDF) not being available yet at startup.
        transform = self.tf_buffer.lookup_transform("base_footprint", "base_link", rospy.Time(0), rospy.Duration(100))
        self.base_footprint_to_base_link = transform_to_pose(transform)

        # Subscribers
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.initialpose_callback, queue_size=1, tcp_nodelay=True)
        if self.use_msl_height:
            self.bestpos_sub = rospy.Subscriber('/novatel/oem7/bestpos', BESTPOS, self.bestpos_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/novatel/oem7/inspva', INSPVA, self.inspva_callback, queue_size=1, tcp_nodelay=True)

        # Services
        rospy.Service('cancel_pose', Empty, self.cancel_pose_callback)

        # output information to console
        rospy.loginfo("%s - initialized", rospy.get_name())


    def initialpose_callback(self, pose_msg):
        if self.current_pose is None:
            return

        if pose_msg.header.frame_id != "map":
            # Convert non-map frame to map frame
            transform = self.tf_buffer.lookup_transform("map", pose_msg.header.frame_id, pose_msg.header.stamp, rospy.Duration(0.06))
            # Construct PoseStamped from the extracted position and heading
            pose_msg.pose = transform_pose(transform, pose_msg.pose)

        # take the z value for initialpose from current_pose
        pose_msg.pose.pose.position.z = self.current_pose.position.z
        initialpose_matrix = numpify(pose_msg.pose.pose)
        current_pose_matrix = numpify(self.current_pose)
        # get the difference between initialpose and current_pose
        self.relative_pose_matrix = initialpose_matrix.dot(np.linalg.pinv(current_pose_matrix))


    def cancel_pose_callback(self, req):
        self.relative_pose_matrix = None
        return EmptyResponse()


    def get_timestamp(self, inspva_msg):
        """
        Get the timestamp to use for publishing based on time_source setting.
        For 'auto' mode, detects GPS sync on first message and overrides time_source.
        """

        # auto-detect on first call if time_source is "auto"
        if self.time_source == "auto":
            gps_stamp = self.convert_gps_time_to_ros_time(inspva_msg.nov_header.gps_week_number, inspva_msg.nov_header.gps_week_milliseconds)
            time_diff = abs((gps_stamp - inspva_msg.header.stamp).to_sec())
            if time_diff < self.gps_time_sync_threshold:
                self.time_source = "gps"
                rospy.loginfo("%s - auto-detected GPS-synced time (diff: %.3fs), using GPS time", rospy.get_name(), time_diff)
            else:
                self.time_source = "ros"
                rospy.loginfo("%s - auto-detected non-synced time (diff: %.1fs), using ROS time", rospy.get_name(), time_diff)

        if self.time_source == "gps":
            return self.convert_gps_time_to_ros_time(inspva_msg.nov_header.gps_week_number, inspva_msg.nov_header.gps_week_milliseconds)
        else:
            return inspva_msg.header.stamp


    def inspva_callback(self, inspva_msg):
        try:
            stamp = self.get_timestamp(inspva_msg)

            # transform GNSS coordinates and correct azimuth, if lat=lon=0 from INSPVA message use offline values
            if inspva_msg.latitude == 0 and inspva_msg.longitude == 0:
                rospy.logwarn_throttle(30, "Received 0 Latitude and Longitude from INSPVA message, using offline values")
                latitude = self.offline_lat
                longitude = self.offline_lon
                azimuth = self.offline_azimuth
                # offline_height uses msl height
                height = self.offline_height
            else:
                latitude = inspva_msg.latitude
                longitude = inspva_msg.longitude
                azimuth = inspva_msg.azimuth
                # inspva_msg contains ellipsoid height if msl (mean sea level) height is wanted then undulation is subtracted
                height = inspva_msg.height
                if self.use_msl_height:
                    height -= self.undulation

            gps_point = lanelet2.core.GPSPoint(latitude, longitude, height)
            utm_point = self.projector.forward(gps_point)
            azimuth = self.correct_azimuth(latitude, longitude, azimuth)
            linear_speed = math.sqrt(inspva_msg.east_velocity**2 + inspva_msg.north_velocity**2)

            # angles from GNSS (degrees) need to be converted to orientation (quaternion) in map frame
            orientation = convert_angles_to_orientation(inspva_msg.roll, inspva_msg.pitch, azimuth)

            current_pose = Pose()
            current_pose.position.x = utm_point.x
            current_pose.position.y = utm_point.y
            current_pose.position.z = height
            current_pose.orientation = orientation

            # set true current_pose (from GNSS) to class variable
            self.current_pose = current_pose

            # if initalpose is set reposition car
            if self.relative_pose_matrix is not None:
                current_pose_matrix = numpify(self.current_pose)
                new_current_pose_matrix = self.relative_pose_matrix.dot(current_pose_matrix)
                # replace current_pose with new_current_pose
                current_pose = msgify(Pose, new_current_pose_matrix)
                # set the z value to the lanelet2 map height, which is at ground level (base_footprint)
                current_pose.position.z = get_height_at_position(self.lanelet2_map, current_pose.position.x, current_pose.position.y, current_pose.position.z)
                # lift the ground-level pose to base_link, as reported by the GNSS on the real vehicle
                # map -> base_link = (map -> base_footprint) * (base_footprint -> base_link)
                current_pose = transform_pose(pose_to_transform(current_pose), self.base_footprint_to_base_link)

            # Publish
            self.publish_current_pose(stamp, current_pose)
            self.publish_current_velocity(stamp, linear_speed)
            if self.broadcast_tf:
                self.publish_map_to_baselink_tf(stamp, current_pose)
            self.publish_odometry(stamp, linear_speed, current_pose)
        except Exception as e:
            rospy.logerr_throttle(10, "%s - Exception in callback: %s", rospy.get_name(), traceback.format_exc())

    def bestpos_callback(self, bestpos_msg):
        self.undulation = bestpos_msg.undulation

    def correct_azimuth(self, lat, lon, azimuth):
        # calculate grid convergence and use to correct the azimuth
        # https://gis.stackexchange.com/questions/115531/calculating-grid-convergence-true-north-to-grid-north
        a = math.tan(math.radians(lon - self.central_meridian))
        b = math.sin(math.radians(lat))
        correction = math.degrees(math.atan(a * b))
        return azimuth - correction

    def publish_current_pose(self, stamp, current_pose):

        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = "map"
        pose_msg.pose = current_pose

        self.current_pose_pub.publish(pose_msg)


    def publish_current_velocity(self, stamp, linear_speed):

        vel_msg = TwistStamped()

        vel_msg.header.stamp = stamp
        vel_msg.header.frame_id = self.child_frame
        vel_msg.twist.linear.x = linear_speed

        self.current_velocity_pub.publish(vel_msg)

    def publish_odometry(self, stamp, speed, current_pose):

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose = current_pose
        odom_msg.twist.twist.linear.x = speed

        self.odometry_pub.publish(odom_msg)


    def publish_map_to_baselink_tf(self, stamp, current_pose):

        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()

        t.header.stamp = stamp
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame

        t.transform.translation.x = current_pose.position.x
        t.transform.translation.y = current_pose.position.y
        t.transform.translation.z = current_pose.position.z
        t.transform.rotation = current_pose.orientation

        br.sendTransform(t)


    def convert_gps_time_to_ros_time(self, gps_week_number, gps_week_milliseconds):
        gps_epoch_to_unix_epoch_offset = 315964800 # seconds

        unix_time_seconds = gps_week_number * 7 * 24 * 3600 + gps_epoch_to_unix_epoch_offset + gps_week_milliseconds / 1000.0 - self.gps_time_leap_seconds
        return rospy.Time.from_sec(unix_time_seconds)


    def run(self):
        rospy.spin()


# Helper functions

def convert_angles_to_orientation(roll, pitch, yaw):

    # convert angles to radians
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)

    roll, pitch, yaw = convertAzimuthToENU(roll, pitch, yaw)
    x, y, z, w = quaternion_from_euler(roll, pitch, yaw).tolist()
    return Quaternion(x=x, y=y, z=z, w=w)


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
