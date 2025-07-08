#!/usr/bin/env python3

import rospy
import math
import message_filters
import traceback
import shapely
import numpy as np
from numpy.lib.recfunctions import structured_to_unstructured
from ros_numpy import numpify
from autoware_mini.msg import Path, Log
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3
from autoware_mini.path import PathWrapper
from autoware_mini.geometry import project_vector_to_heading, get_distance_between_two_points_2d
from autoware_mini.transform import get_distance_to_car_front
from autoware_mini.collision import CollisionPoints


class SpeedPlanner:

    def __init__(self):

        # parameters
        self.default_deceleration = rospy.get_param("default_deceleration")
        self.braking_reaction_time = rospy.get_param("braking_reaction_time")
        synchronization_method = rospy.get_param("~synchronization_method")
        synchronization_queue_size = rospy.get_param("~synchronization_queue_size")
        synchronization_slop = rospy.get_param("~synchronization_slop")

        # variables
        self.collision_points = None
        self.current_position = None
        self.current_speed = None
        self.distance_to_car_front = get_distance_to_car_front()

        # publishers
        self.local_path_pub = rospy.Publisher('local_path', Path, queue_size=1, tcp_nodelay=True)
        self.log_message_pub = rospy.Publisher('/dashboard/log_message', Log, queue_size=1, tcp_nodelay=True)

        # subscribers
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/localization/current_velocity', TwistStamped, self.current_velocity_callback, queue_size=1, tcp_nodelay=True)

        collision_points_sub = message_filters.Subscriber('collision_points', PointCloud2, tcp_nodelay=True)
        local_path_sub = message_filters.Subscriber('extracted_local_path', Path, tcp_nodelay=True)

        if synchronization_method == "approximate":
            ts = message_filters.ApproximateTimeSynchronizer([collision_points_sub, local_path_sub], queue_size=synchronization_queue_size, slop=synchronization_slop)
        elif synchronization_method == "exact":
            ts = message_filters.TimeSynchronizer([collision_points_sub, local_path_sub], queue_size=2)
        else:
            raise ValueError(f"'{synchronization_method}' is not a known synchronization method")

        ts.registerCallback(self.collision_points_and_path_callback)

    def current_velocity_callback(self, msg):
        self.current_speed = msg.twist.linear.x

    def current_pose_callback(self, msg):
        self.current_position = shapely.Point(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def collision_points_and_path_callback(self, collision_points_msg, local_path_msg):
        try:
            collision_points = numpify(collision_points_msg)
            current_position = self.current_position
            current_speed = self.current_speed

            if current_speed is None or current_position is None:
                rospy.logwarn_throttle(3, "%s - current speed or position not received!", rospy.get_name())
                return

            if  len(local_path_msg.waypoints) == 0 or len(collision_points) == 0:
                # no local path or no collision points menas no alterations to the path
                self.local_path_pub.publish(local_path_msg)
                return

            closest_object_distance = 0.0
            closest_object_velocity = 0.0
            stopping_point_distance = 0.0

            # create local path
            local_path = PathWrapper(local_path_msg.waypoints)
            ego_distance_from_local_path_start = local_path.linestring.project(current_position)

            # extract collision_point distances
            collision_points_shapely = shapely.points(structured_to_unstructured(collision_points[['x', 'y', 'z']]))
            collision_point_distances = local_path.linestring.project(collision_points_shapely)

            # calculate deceleration for every collision point
            deceleration_distances = collision_point_distances - self.distance_to_car_front
            decelerations = (current_speed**2) / np.maximum(2.0 * deceleration_distances - ego_distance_from_local_path_start, 0.001)
            deceleration_exceeded_mask = (decelerations > collision_points['deceleration_limit'])
            for i in np.where(deceleration_exceeded_mask)[0]:
                self.log_message_pub.publish(Log(message = CollisionPoints.COLLISION_POINT_CATEGORY_IGNORE_CAPTION[collision_points[i]['category']], color = "white"))
                rospy.logwarn_throttle(3, f"{rospy.get_name()} - {CollisionPoints.COLLISION_POINT_CATEGORY_IGNORE_CAPTION[collision_points[i]['category']]} ({deceleration_distances[i]:.1f} m) - deceleration of {decelerations[i]:.2f} m/s2 exceeds limit ({collision_points[i]['deceleration_limit']:.2f} m/s2), ignore!")

            # if all collision points exceed deceleration - publish the original path and return
            if np.all(deceleration_exceeded_mask):
                self.local_path_pub.publish(local_path_msg)
                return

            # filter out collision points that exceed deceleration limit
            collision_points = collision_points[~deceleration_exceeded_mask]
            collision_point_distances = collision_point_distances[~deceleration_exceeded_mask]
            deceleration_distances = deceleration_distances[~deceleration_exceeded_mask]

            # calculate target velocity for every collision point
            collision_point_path_headings = [local_path.get_heading_at_distance(distance) for distance in collision_point_distances]
            collision_point_velocities = np.array([project_vector_to_heading(heading, Vector3(vx, vy, vz))
                                        for heading, (vx, vy, vz) in zip(collision_point_path_headings, collision_points[['vx', 'vy', 'vz']])])
            collision_point_braking_distances = collision_points['distance_to_stop']

            # calculate target velocity for every collision point
            # 'abs' is used to turn negative speed of approaching cars into positive, so that target distance would be smaller and thus target_speed will be decreased
            target_distances = deceleration_distances - np.maximum(collision_point_braking_distances, self.braking_reaction_time * np.abs(collision_point_velocities))
            target_velocities = np.sqrt(np.maximum(0.0, np.maximum(0.0, collision_point_velocities)**2 + 2 * self.default_deceleration * target_distances))

            # select min target velocity among the ones that do not exceed deceleration limit and is closest to the ego vehicle
            min_target_velocity = np.min(target_velocities)
            mask = np.isclose(target_velocities, min_target_velocity)
            adjusted_distances = np.where(mask, collision_point_distances, np.inf)
            min_value_index = np.argmin(adjusted_distances)

            closest_object_distance = collision_point_distances[min_value_index] - ego_distance_from_local_path_start - self.distance_to_car_front
            closest_object_velocity = collision_point_velocities[min_value_index]
            stopping_point_distance = collision_point_distances[min_value_index] - collision_point_braking_distances[min_value_index]
            collision_point_category = collision_points[min_value_index]["category"]

            # Recalculate target_velocity for all the waypoints using the closest object
            zero_speeds_onwards = False
            target_distance_object = target_distances[min_value_index]
            for i, wp in enumerate(local_path.waypoints):

                # once we get zero speed, keep it that way
                if zero_speeds_onwards:
                    wp.speed = 0.0
                    continue

                if i > 0:
                    target_distance_object -= get_distance_between_two_points_2d(local_path.waypoints[i-1].position, local_path.waypoints[i].position)
                target_velocity_object = np.sqrt(np.maximum(0.0, np.maximum(0.0, closest_object_velocity)**2 + 2 * self.default_deceleration * target_distance_object))

                # overwrite target velocity of wp
                wp.speed = min(target_velocity_object, wp.speed)

                # from stop point onwards all speeds are set to zero
                if math.isclose(wp.speed, 0.0):
                    zero_speeds_onwards = True

            # Update the lane message with the calculated values
            path = Path()
            path.header = local_path_msg.header
            path.waypoints = local_path.waypoints
            path.closest_object_distance = closest_object_distance
            path.closest_object_velocity = closest_object_velocity
            path.is_blocked = True
            path.stopping_point_distance = stopping_point_distance
            path.collision_point_category = collision_point_category
            self.local_path_pub.publish(path)

        except Exception as e:
            rospy.logerr_throttle(10, "%s - Exception in callback: %s", rospy.get_name(), traceback.format_exc())


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('speed_planner')
    node = SpeedPlanner()
    node.run()