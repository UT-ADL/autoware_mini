#!/usr/bin/env python3

import math
import traceback
import rospy

from nav_msgs.msg import Odometry
import xml.etree.ElementTree as ET

from helpers.geometry import get_heading_from_orientation


class CarlaRouteSaver:
    def __init__(self):

        # Parameters
        self.interval = rospy.get_param("~interval", default=2) # distance interval between waypoints 2 meters by default
        self.routes_file = rospy.get_param("~routes_file")

        # Internal params
        self.written_x = 0  # last x coordinate written into text file, kept to calculate distance interval
        self.written_y = 0  # last y coordinate written into text file, kept to calculate distance interval
        self.root = ET.Element("route")

        # Register the shutdown hook for writing XML
        rospy.on_shutdown(self.shutdown_hook)

        # Subscribers
        rospy.Subscriber('/carla/odometry', Odometry, self.odometry_callback, queue_size=1, tcp_nodelay=True)

        # loginfo
        rospy.loginfo("%s - interval: %i m", rospy.get_name(), self.interval)
        rospy.loginfo("%s - save to %s ", rospy.get_name(), self.routes_file)

    def odometry_callback(self, odometry_msg):

        current_pose = odometry_msg.pose

        try:
            x = current_pose.pose.position.x
            y = current_pose.pose.position.y

            # distance between current and last written waypoint coordinates
            distance = math.sqrt((self.written_x - x) ** 2 + (self.written_y - y) ** 2)

            if distance >= self.interval:
                # calculate current_heading
                current_heading = math.degrees(get_heading_from_orientation(current_pose.pose.orientation))
                
                # x, y, z, yaw ------> x, -y, z, -yaw ROS to Carla coordinate system conversion
                ET.SubElement(self.root, "waypoint", pitch="0.0", x=str(x), y=str(-y), z=str(current_pose.pose.position.z), yaw=str(-current_heading))

                # update stored values
                self.written_x = x
                self.written_y = y

        except Exception as e:
            rospy.logerr_throttle(10, "%s - Exception in callback: %s", rospy.get_name(), traceback.format_exc())

        
    def shutdown_hook(self):
        try:
            # Write data to waypoints.xml file
            ET.ElementTree(self.root).write(self.routes_file)
        except Exception as e:
            rospy.logerr("%s - Exception in shutdown_hook: %s", rospy.get_name(), traceback.format_exc())


    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('carla_route_saver', log_level=rospy.INFO)
    node = CarlaRouteSaver()
    node.run()