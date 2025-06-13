#!/usr/bin/env python3

import rospy
import math
import numpy as np
from autoware_mini.msg import Path
from sensor_msgs.msg import PointCloud2
from autoware_mini.collision import CollisionPoints
from autoware_mini.geometry import get_distance_between_two_points_2d

class GoalStopChecker:

    def __init__(self):

        # parameters
        self.braking_safety_distance_goal = rospy.get_param("~braking_safety_distance_goal")

        # variables
        self.goal_point = None

        # publishers
        self.goal_point_pub = rospy.Publisher('goal_collision_points', PointCloud2, queue_size=1, latch=True, tcp_nodelay=True)

        # subscribers
        rospy.Subscriber('extracted_local_path', Path, self.local_path_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('global_path', Path, self.global_path_callback, queue_size=1, tcp_nodelay=True)

    def global_path_callback(self, msg):

        if len(msg.waypoints) > 0:
            # lasst point of the global path is goal point
            self.goal_point = msg.waypoints[-1].position
        else:
            self.goal_point = None

    def local_path_callback(self, msg):

        collision_points = CollisionPoints()
        goal_point = self.goal_point

        if goal_point is not None and len(msg.waypoints) > 0:
            # check if goal point is at the end of the local path
            if math.isclose(get_distance_between_two_points_2d(goal_point, msg.waypoints[-1].position), 0.0):
                # add goal point as collision point
                collision_points.add_point(x = goal_point.x,
                                           y = goal_point.y,
                                           z = goal_point.z,
                                           vx = 0.0,
                                           vy = 0.0,
                                           vz = 0.0,
                                           distance_to_stop = self.braking_safety_distance_goal,
                                           deceleration_limit = np.inf,
                                           category = CollisionPoints.GOAL_POINT)

        collision_points_msg = collision_points.create_message()
        collision_points_msg.header = msg.header
        self.goal_point_pub.publish(collision_points_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('goal_stop_checker')
    node = GoalStopChecker()
    node.run()