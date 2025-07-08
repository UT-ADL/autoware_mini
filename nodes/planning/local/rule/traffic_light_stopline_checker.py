#!/usr/bin/env python3

import rospy
import shapely
import numpy as np
from autoware_mini.msg import Path, TrafficLightResultArray
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import PointCloud2
from autoware_mini.path import PathWrapper
from autoware_mini.collision import CollisionPoints
from autoware_mini.lanelet2 import load_lanelet2_map, get_traffic_light_stop_lines

class TrafficLightStoplineChecker:

    def __init__(self):

        # parameters
        self.tfl_force_stop_speed_limit = rospy.get_param("~tfl_force_stop_speed_limit")
        self.braking_safety_distance_stopline = rospy.get_param("~braking_safety_distance_stopline")
        self.tfl_maximum_deceleration = rospy.get_param("~tfl_maximum_deceleration")
        lanelet2_map_path = rospy.get_param("~lanelet2_map_path")

        # variables
        self.current_speed = None
        self.stopline_statuses = {}
        lanelet2_map = load_lanelet2_map(lanelet2_map_path)
        self.all_stoplines = get_traffic_light_stop_lines(lanelet2_map)

        # publishers
        self.traffic_light_stopline_pub = rospy.Publisher('tfl_stopline_collision_points', PointCloud2, queue_size=1, tcp_nodelay=True)

        # subscribers
        rospy.Subscriber('extracted_local_path', Path, self.local_path_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/detection/traffic_light_status', TrafficLightResultArray, self.traffic_light_status_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/localization/current_velocity', TwistStamped, self.current_velocity_callback, queue_size=1, tcp_nodelay=True)

    def current_velocity_callback(self, msg):
        self.current_speed = msg.twist.linear.x

    def traffic_light_status_callback(self, msg):
        stopline_statuses = {}
        for result in msg.results:
            stopline_statuses[result.stopline_id] = result.recognition_result

        self.stopline_statuses = stopline_statuses

    def local_path_callback(self, msg):

        current_speed = self.current_speed

        if current_speed is None:
            rospy.logwarn_throttle(3, "%s - current velocity not received!", rospy.get_name())
            return

        stopline_statuses = self.stopline_statuses
        collision_points = CollisionPoints()

        if len(msg.waypoints) > 0 and len(stopline_statuses) > 0:
            local_path = PathWrapper(msg.waypoints)

            for stopline_id, stopline_linestring in self.all_stoplines.items():
                # if RED and intersects with local path add as collision point
                if stopline_id in stopline_statuses and stopline_statuses[stopline_id] == 0 and stopline_linestring.intersects(local_path.linestring):
                    intersection_point = local_path.linestring.intersection(stopline_linestring)
                    assert isinstance(intersection_point, shapely.Point), "Stop line and local path intersection is not a shapely.Point"

                    collision_points.add_point(x = intersection_point.x,
                                                y = intersection_point.y,
                                                z = intersection_point.z,
                                                vx = 0.0,
                                                vy = 0.0,
                                                vz = 0.0,
                                                distance_to_stop = self.braking_safety_distance_stopline,
                                                deceleration_limit = np.inf if current_speed < (self.tfl_force_stop_speed_limit / 3.6) else self.tfl_maximum_deceleration,
                                                category = CollisionPoints.TRAFFIC_LIGHT_STOPLINE)

        collision_points_msg = collision_points.create_message()
        collision_points_msg.header = msg.header
        self.traffic_light_stopline_pub.publish(collision_points_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('traffic_light_stopline_checker')
    node = TrafficLightStoplineChecker()
    node.run()