#!/usr/bin/env python3

import math
import traceback
import rospy
import message_filters
import shapely
import numpy as np
from numpy.lib.recfunctions import structured_to_unstructured

from ros_numpy import numpify
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, TwistStamped

from autoware_mini.msg import Path, LocalPath, Log
from autoware_mini.messages import path_to_local_path
from autoware_mini.path import PathWrapper
from autoware_mini.geometry import get_distance_between_two_points_2d, calculate_headings
from autoware_mini.transform import get_distance_to_car_front
from autoware_mini.collision import CollisionPoints


class SpeedPlanner:

    def __init__(self):

        # parameters
        self.braking_reaction_time = rospy.get_param("braking_reaction_time")
        self.stopped_speed_limit = rospy.get_param("stopped_speed_limit")
        self.safety_box_width = rospy.get_param("safety_box_width")
        synchronization_method = rospy.get_param("~synchronization_method")
        synchronization_queue_size = rospy.get_param("~synchronization_queue_size")
        synchronization_slop = rospy.get_param("~synchronization_slop")

        # variables
        self.collision_points = None
        self.current_position = None
        self.current_speed = None
        self.distance_to_car_front = get_distance_to_car_front()

        # publishers
        self.local_path_pub = rospy.Publisher('local_path', LocalPath, queue_size=1, tcp_nodelay=True)
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

        rospy.loginfo("%s - initialized", rospy.get_name())

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

            if  not local_path_msg.waypoints or collision_points.size == 0:
                # no local path or no collision points menas no alterations to the path
                self.publish_local_path(path_to_local_path(local_path_msg))
                return

            target_object_distance = 0.0
            target_object_speed = 0.0
            stopping_point_distance = 0.0

            # create local path
            local_path = PathWrapper(local_path_msg.waypoints)
            ego_distance_from_local_path_start = local_path.linestring.project(current_position)

            # create safety buffer and filter collision points
            safety_buffer = local_path.linestring.buffer(self.safety_box_width / 2.0, cap_style="flat")
            shapely.prepare(safety_buffer)

            collision_points_shapely = shapely.points(structured_to_unstructured(collision_points[['x', 'y', 'z']]))
            collision_points_mask = safety_buffer.intersects(collision_points_shapely)

            # if no collision points in the safety buffer - publish the original path and return
            if not np.any(collision_points_mask):
                self.publish_local_path(path_to_local_path(local_path_msg))
                return

            # filter out collision points that are outside the safety buffer
            collision_points = collision_points[collision_points_mask]
            collision_points_shapely = collision_points_shapely[collision_points_mask]

            # extract collision_point distances
            collision_point_distances = local_path.linestring.project(collision_points_shapely)

            # calculate deceleration for every collision point
            deceleration_distances = collision_point_distances - self.distance_to_car_front - ego_distance_from_local_path_start
            required_decelerations = (current_speed**2) / np.maximum(2.0 * deceleration_distances, 0.001)
            deceleration_exceeded_mask = (required_decelerations > collision_points['deceleration_limit'])
            for i in np.where(deceleration_exceeded_mask)[0]:
                self.log_message_pub.publish(Log(message = CollisionPoints.COLLISION_POINT_CATEGORY_IGNORE_CAPTION[collision_points[i]['category']], color = "white", instant = True))
                rospy.logwarn_throttle(3, f"{rospy.get_name()} - {CollisionPoints.COLLISION_POINT_CATEGORY_IGNORE_CAPTION[collision_points[i]['category']]} ({deceleration_distances[i]:.1f} m) - deceleration of {required_decelerations[i]:.2f} m/s2 exceeds limit ({collision_points[i]['deceleration_limit']:.2f} m/s2), ignore!")

            # if all collision points exceed deceleration - publish the original path and return
            if np.all(deceleration_exceeded_mask):
                self.publish_local_path(path_to_local_path(local_path_msg))
                return

            # filter out collision points that exceed deceleration limit
            collision_points = collision_points[~deceleration_exceeded_mask]
            collision_point_distances = collision_point_distances[~deceleration_exceeded_mask]
            deceleration_distances = deceleration_distances[~deceleration_exceeded_mask]

            # calculate target speed for every collision point
            collision_point_path_headings = calculate_headings(shapely.get_coordinates(local_path.linestring), collision_point_distances)
            collision_point_speeds = collision_points['vx'] * np.cos(collision_point_path_headings) + collision_points['vy'] * np.sin(collision_point_path_headings)
            collision_point_braking_distances = collision_points['distance_to_stop']

            # calculate target speed for every collision point
            # for approaching objects (v < -stopped_speed_limit), approaching speed accounts for the object closing distance during ego braking
            target_distances = deceleration_distances - np.maximum(collision_point_braking_distances, self.braking_reaction_time * np.abs(collision_point_speeds))
            approaching_speeds = np.minimum(collision_point_speeds, 0.0)
            decelerations = collision_points['deceleration']
            target_speeds = np.maximum(0.0, approaching_speeds + np.sqrt(np.maximum(0.0, collision_point_speeds**2 + 2 * decelerations * target_distances)))

            # select min target speed (treat equally speeds below stopped_speed_limit) among the ones that do not exceed deceleration limit and is closest to the ego vehicle
            min_target_speed = max(self.stopped_speed_limit, np.min(target_speeds))
            mask = target_speeds <= min_target_speed
            adjusted_distances = np.where(mask, collision_point_distances, np.inf)
            min_value_index = np.argmin(adjusted_distances)

            target_object_distance = deceleration_distances[min_value_index]
            target_object_speed = collision_point_speeds[min_value_index]
            stopping_point_distance = collision_point_distances[min_value_index] - collision_point_braking_distances[min_value_index]
            collision_point_category = collision_points[min_value_index]["category"]

            # Recalculate target speed for all the waypoints using the closest object
            zero_speeds_onwards = False
            # calculate target distance relative to the waypoints therefore adding ego_distance_from_local_path_start
            target_distance_object = target_distances[min_value_index] + ego_distance_from_local_path_start
            approaching_speed = min(target_object_speed, 0.0)
            deceleration = collision_points[min_value_index]['deceleration']
            for i, wp in enumerate(local_path.waypoints):

                # once we get zero speed, keep it that way
                if zero_speeds_onwards:
                    wp.speed = 0.0
                    continue

                if i > 0:
                    target_distance_object -= get_distance_between_two_points_2d(local_path.waypoints[i-1].position, local_path.waypoints[i].position)
                target_speed_object = max(0.0, approaching_speed + math.sqrt(max(0.0, target_object_speed**2 + 2 * deceleration * target_distance_object)))

                # overwrite target speed of wp
                wp.speed = min(target_speed_object, wp.speed)

                # from stop point onwards all speeds are set to zero
                if math.isclose(wp.speed, 0.0):
                    zero_speeds_onwards = True

            # Update the lane message with the calculated values
            local_path_out = LocalPath()
            local_path_out.header = local_path_msg.header
            local_path_out.waypoints = local_path.waypoints
            local_path_out.target_object_distance = float(target_object_distance)
            local_path_out.target_object_speed = float(target_object_speed)
            local_path_out.is_blocked = True
            local_path_out.stopping_point_distance = float(stopping_point_distance)
            local_path_out.collision_point_category = int(collision_point_category)
            self.publish_local_path(local_path_out)

        except Exception as e:
            rospy.logerr_throttle(10, "%s - Exception in callback: %s", rospy.get_name(), traceback.format_exc())


    def publish_local_path(self, local_path_out):
        self.local_path_pub.publish(local_path_out)
        if len(local_path_out.waypoints) > 1:
            category = local_path_out.collision_point_category
        else:
            category = CollisionPoints.NO_PATH
        self.log_message_pub.publish(Log(message=CollisionPoints.COLLISION_POINT_CATEGORY_CAPTION[category], color=CollisionPoints.COLLISION_POINT_CATEGORY_COLOR[category]))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('speed_planner')
    node = SpeedPlanner()
    node.run()