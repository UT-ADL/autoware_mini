#!/usr/bin/env python3

import rospy
import shapely
import numpy as np
from autoware_mini.lanelet2 import load_lanelet2_map, get_stop_lines_using_subtype
from autoware_mini.collision import CollisionPoints
from std_msgs.msg import Int32
from autoware_mini.msg import Path, Log
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Empty, EmptyResponse

class AutomaticStopChecker:

    def __init__(self):

        # parameters
        lanelet2_map_path = rospy.get_param("~lanelet2_map_path")
        self.keep_stop_line_for = rospy.get_param("~keep_stop_line_for")
        self.braking_safety_distance_stop_line = rospy.get_param("~braking_safety_distance_stop_line")

        # variables
        self.stop_lines_on_global_path = None
        self.current_closest_stop_line_id = -1
        self.ignore_stop_line_id = -1
        self.timer = rospy.Time.now()

        lanelet2_map = load_lanelet2_map(lanelet2_map_path)
        self.stop_lines = get_stop_lines_using_subtype(lanelet2_map, subtypes=["yield_stop"])

        # publishers
        self.lets_go_pub = rospy.Publisher('lets_go', Int32, queue_size=1, tcp_nodelay=True)
        self.stop_line_collision_pub = rospy.Publisher('stop_line_collision_points', PointCloud2, queue_size=1, tcp_nodelay=True)
        self.log_message_pub = rospy.Publisher('/dashboard/log_message', Log, queue_size=5, tcp_nodelay=True)

        # subscribers
        rospy.Subscriber('global_path', Path, self.global_path_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('extracted_local_path', Path, self.local_path_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('lets_go', Int32, self.lets_go_callback, queue_size=1, tcp_nodelay=True)

        # Services
        rospy.Service('service_lets_go', Empty, self.lets_go_handler)

    def global_path_callback(self, msg):
        global_path_linestring = shapely.LineString([(waypoint.position.x, waypoint.position.y) for waypoint in msg.waypoints])
        global_path_linestring = global_path_linestring.simplify(0.01)
        shapely.prepare(global_path_linestring)

        stop_lines_on_global_path = {}
        for id, stop_line in self.stop_lines.items():
            if stop_line.intersects(global_path_linestring):
                stop_lines_on_global_path[id] = stop_line

        self.stop_lines_on_global_path = stop_lines_on_global_path

    def local_path_callback(self, msg):

        stop_lines_on_global_path = self.stop_lines_on_global_path

        if stop_lines_on_global_path is None:
            return

        collision_points = CollisionPoints()

        local_path_linestring = shapely.LineString([(waypoint.position.x, waypoint.position.y) for waypoint in msg.waypoints])
        shapely.prepare(local_path_linestring)

        stop_line_distance = np.inf
        closest_stop_line_id = -1

        for id, stop_line in stop_lines_on_global_path.items():
            if stop_line.intersects(local_path_linestring):
                stop_line_intersection_result = stop_line.intersection(local_path_linestring)
                # assert stop_line_intersection_result.geom_type == shapely.Point, "Stop line intersection with local_path is not a shapely Point"

                # if not "remove point" then add to collision points
                if id != self.ignore_stop_line_id:
                    collision_points.add_point(x = stop_line_intersection_result.x,
                                                y = stop_line_intersection_result.y,
                                                z = stop_line_intersection_result.z,
                                                vx = 0.0,
                                                vy = 0.0, 
                                                vz = 0.0,
                                                distance_to_stop = self.braking_safety_distance_stop_line,
                                                deceleration_limit = np.inf,
                                                category = CollisionPoints.STOP_LINE_FORCED_STOP)

                # find closest stop line
                distance = local_path_linestring.project(stop_line_intersection_result)
                if distance < stop_line_distance:
                    stop_line_distance = distance
                    closest_stop_line_id = id

        self.current_closest_stop_line_id = closest_stop_line_id

        # set ignore_stop_line_id to -1 if it current closest stopline changes or the timer has expired
        if self.ignore_stop_line_id != -1 and (self.current_closest_stop_line_id != self.ignore_stop_line_id or self.timer + rospy.Duration(self.keep_stop_line_for) < rospy.Time.now()):
            self.ignore_stop_line_id = -1
            self.lets_go_pub.publish(Int32(self.ignore_stop_line_id))
        collision_points_msg = collision_points.create_message()
        collision_points_msg.header = msg.header
        self.stop_line_collision_pub.publish(collision_points_msg)

    def lets_go_callback(self, msg):
        if msg.data == -1:
            return

        # reset timer and set current closest stop line id as the one to be removed
        self.timer = rospy.Time.now()
        self.ignore_stop_line_id = self.current_closest_stop_line_id
        self.log_message_pub.publish(Log(message = "Allow crossing the yield line", color = "white"))
        rospy.loginfo("Removed forced stop for stopline id %d for %d seconds", self.ignore_stop_line_id, self.keep_stop_line_for)

    # service call to simulate Go button press from rviz
    def lets_go_handler(self, msg):
        self.lets_go_pub.publish(Int32(self.current_closest_stop_line_id))
        return EmptyResponse()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('automatic_stop_checker')
    node = AutomaticStopChecker()
    node.run()