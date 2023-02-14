#!/usr/bin/env python

import rospy
import tf
import math

from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose
from std_msgs.msg import ColorRGBA


class StanleyFollower:
    def __init__(self, waypoints, waypoint_tree):

        self.waypoints = waypoints
        self.waypoint_tree = waypoint_tree

        # Parameters
        self.wheel_base = rospy.get_param("~wheel_base", 2.789)

        # Variables - init
        self.last_wp_idx = len(self.waypoints) - 1

        # Publishers
        self.stanley_rviz_pub = rospy.Publisher('follower_markrs', MarkerArray, queue_size=1)


    def calc_steering_and_velocity(self, current_pose, current_velocity):


        # Find pose for the front wheel
        front_wheel_pose = get_front_wheel_pose(current_pose, self.wheel_base)
        quaternion = (front_wheel_pose.orientation.x, front_wheel_pose.orientation.y, front_wheel_pose.orientation.z, front_wheel_pose.orientation.w)
        _, _, front_wheel_heading = tf.transformations.euler_from_quaternion(quaternion)
        print("DEBUG - current     pose: %f, %f" % (current_pose.position.x, current_pose.position.y))
        print("DEBUG - front wheel pose: %f, %f" % (front_wheel_pose.position.x, front_wheel_pose.position.y))
        
        # Find the closest waypoint to the front wheel
        self.nearest_wp_distance, self.nearest_wp_idx = self.waypoint_tree.query([[front_wheel_pose.position.x, front_wheel_pose.position.y]], 1)
        print("DEBUG - nearest waypoint: %i, distance: %f" % (self.nearest_wp_idx, self.nearest_wp_distance))

        # find nearest waypoint heading
        nearest_wp = self.waypoints[int(self.nearest_wp_idx)]
        
        self.target_velocity = nearest_wp.twist.twist.linear.x * 4.0    

        quaternion = (nearest_wp.pose.pose.orientation.x, nearest_wp.pose.pose.orientation.y, nearest_wp.pose.pose.orientation.z, nearest_wp.pose.pose.orientation.w)
        _, _, nearest_wp_heading = tf.transformations.euler_from_quaternion(quaternion)
        print("DEBUG - front wheel heading: %f" % front_wheel_heading)
        print("DEBUG - nearest waypoint heading: %f" % nearest_wp_heading)

        # find heading error
        heading_error = nearest_wp_heading - front_wheel_heading
        print("DEBUG - heading error: %f" % heading_error)

        # calc delta error
        self.k = 1.0
        delta_error = math.atan2(self.k * self.nearest_wp_distance * math.sin(heading_error), 1)
        print("DEBUG - delta error: %f" % delta_error)

        # calc steering angle
        self.steering_angle = heading_error + delta_error
        print("DEBUG - steering angle: %f" % self.steering_angle)

        # TODO limit steering angle before output

        return self.steering_angle, self.target_velocity


def get_front_wheel_pose(current_pose, wheel_base):

    quaternion = (current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w)
    _, _, current_heading = tf.transformations.euler_from_quaternion(quaternion)

    pose = current_pose
    pose.position.x = current_pose.position.x + wheel_base * math.cos(current_heading)
    pose.position.y = current_pose.position.y + wheel_base * math.sin(current_heading)

    return pose
