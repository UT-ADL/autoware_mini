#!/usr/bin/env python3
#
# Copyright (c) 2023 Autonomous Driving Lab (ADL), University of Tartu.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
converts carla GNSS and odometry to novatel OEM7 messages
"""
import math
import traceback
import rospy
import message_filters

from sensor_msgs.msg import NavSatFix
from novatel_oem7_msgs.msg import INSPVA, BESTPOS, InertialSolutionStatus
from nav_msgs.msg import Odometry
import pyproj


class CarlaNovatelDriver():
    def __init__(self):

        # Internal parameters
        self.geodesic = pyproj.Geod(ellps='WGS84')

        # Publishers
        self.inspva_pub = rospy.Publisher("/novatel/oem7/inspva", INSPVA, queue_size=2)
        self.bestpos_pub = rospy.Publisher("/novatel/oem7/bestpos", BESTPOS, queue_size=2)

        # Subscribers
        gnss_sub = message_filters.Subscriber('/gps/fix', NavSatFix, queue_size=2)
        gnss_forward_sub = message_filters.Subscriber('/gps/fix_forward', NavSatFix, queue_size=2)
        odometry_sub = message_filters.Subscriber('/carla/odometry', Odometry, queue_size=2)
        ts = message_filters.ApproximateTimeSynchronizer([gnss_sub, gnss_forward_sub, odometry_sub], queue_size=10, slop=0.05)
        ts.registerCallback(self.sync_callback)

    def sync_callback(self, gnss_data, gnss_forward_data, odometry_data):
        """
        callback GNSS sensor
        """

        try:

            msg = INSPVA()
            msg.header.stamp = gnss_data.header.stamp
            msg.status.status = InertialSolutionStatus.INS_SOLUTION_GOOD
            msg.latitude = gnss_data.latitude
            msg.longitude = gnss_data.longitude

            msg.height = gnss_data.altitude
            msg.roll = 0
            msg.pitch = 0 
            msg.north_velocity = math.sqrt(odometry_data.twist.twist.linear.x**2 + odometry_data.twist.twist.linear.y**2 + odometry_data.twist.twist.linear.z**2)
            msg.east_velocity = 0.0

            lat_init = math.radians(gnss_data.latitude)
            lat_final = math.radians(gnss_forward_data.latitude)

            lon_init = math.radians(gnss_data.longitude)
            lon_final = math.radians(gnss_forward_data.longitude)

            # NOTE: Azimuth angle is required. CARLA simulator doesn't provide it.
            msg.azimuth, _, _ = self.geodesic.inv(lon_init, lat_init, lon_final, lat_final)

            self.inspva_pub.publish(msg)

            bestposMsg = BESTPOS()
            bestposMsg.header.stamp = gnss_data.header.stamp
            # publish with zero undulation

            self.bestpos_pub.publish(bestposMsg)

        except Exception as e:
            rospy.logerr_throttle(10, "%s - Exception in callback: %s", rospy.get_name(), traceback.format_exc())


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('carla_novatel_driver', log_level=rospy.INFO)
    node = CarlaNovatelDriver()
    node.run()
