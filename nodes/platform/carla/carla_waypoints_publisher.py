#!/usr/bin/env python3
#
# Copyright (c) 2023 Autonomous Driving Lab (ADL), University of Tartu.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
receive a path from carla_ros_waypoint_publisher and convert it to autoware
"""
import rospy
from autoware_msgs.msg import LaneArray
from autoware_msgs.msg import Lane
from autoware_msgs.msg import Waypoint
from nav_msgs.msg import Path


class CarlaWaypointsPublisher():

    def __init__(self):
        # Publishers
        self.waypoints_pub = rospy.Publisher('global_path', LaneArray, queue_size=10, latch=True, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber('/carla/ego_vehicle/waypoints', Path, self.path_callback, queue_size=None, tcp_nodelay=True)

    def path_callback(self, data):
        """
        callback for path. Convert it to Autoware LaneArray and publish it
        """
        msg = LaneArray()
        lane = Lane()
        lane.header = data.header
        lane.waypoints = [Waypoint(pose=pose) for pose in data.poses]
        msg.lanes.append(lane)

        self.waypoints_pub.publish(msg)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('carla_waypoints_publisher', log_level=rospy.INFO)
    node = CarlaWaypointsPublisher()
    node.run()
