#!/usr/bin/env python3

import rospy
import copy
import math
import threading
from tf2_ros import Buffer, TransformListener, TransformException

from lanelet2.io import Origin, load
from lanelet2.projection import UtmProjector

import numpy as np
from sklearn.neighbors import NearestNeighbors

from autoware_msgs.msg import Lane, DetectedObjectArray, TrafficLightResultArray
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Point
from std_msgs.msg import ColorRGBA

from helpers.geometry import get_closest_point_on_line, get_distance_between_two_points_2d
from helpers.waypoints import get_two_nearest_waypoint_idx
from helpers.transform import transform_vector3


class VelocityLocalPlanner:

    def __init__(self):

        # Parameters
        self.local_path_length = rospy.get_param("~local_path_length")
        self.nearest_neighbor_search = rospy.get_param("~nearest_neighbor_search")
        self.transform_timeout = rospy.get_param("~transform_timeout")
        self.braking_safety_distance = rospy.get_param("braking_safety_distance")
        self.braking_reaction_time = rospy.get_param("braking_reaction_time")
        self.car_safety_radius = rospy.get_param("car_safety_radius")
        self.current_pose_to_car_front = rospy.get_param("current_pose_to_car_front")
        self.speed_deceleration_limit = rospy.get_param("speed_deceleration_limit")
        self.tfl_deceleration_limit = rospy.get_param("~tfl_deceleration_limit")

        # Internal variables
        self.lock = threading.Lock()
        self.output_frame = None
        self.global_path_array = None
        self.global_path_waypoints = None
        self.global_path_tree = None
        self.current_pose = None
        self.current_velocity = 0.0
        self.stop_lines = {}
        self.red_stop_lines = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        # Publishers
        self.local_path_pub = rospy.Publisher('local_path', Lane, queue_size=1)

        # Subscribers
        rospy.Subscriber('smoothed_path', Lane, self.path_callback, queue_size=1)
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1)
        rospy.Subscriber('/localization/current_velocity', TwistStamped, self.current_velocity_callback, queue_size=1)
        rospy.Subscriber('/detection/detected_objects', DetectedObjectArray, self.detected_objects_callback, queue_size=1)
        rospy.Subscriber('/detection/traffic_light_status', TrafficLightResultArray, self.traffic_light_status_callback, queue_size=1)


    def path_callback(self, msg):

        if len(msg.waypoints) == 0:
            with self.lock:
                self.output_frame = msg.header.frame_id
                self.global_path_array = None
                self.global_path_waypoints = None
                self.global_path_tree = None
            return

        # extract all waypoint attributes
        global_path_waypoints = msg.waypoints
        global_path_array = np.array([(
                wp.pose.pose.position.x,
                wp.pose.pose.position.y,
                wp.pose.pose.position.z,
                wp.twist.twist.linear.x,
                wp.stop_line_id
            ) for wp in msg.waypoints])

        # create stop line dictionary
        self.stop_lines = {}
        stop_line_wp_idx = np.where(global_path_array[:,4] > 0)[0]
        for idx in stop_line_wp_idx:
            self.stop_lines[global_path_array[idx,4]] = {'global_path_index': idx, 'location': [global_path_array[idx,0], global_path_array[idx,1], global_path_array[idx,2], 0]}

        # create global_wp_tree
        global_path_tree = NearestNeighbors(n_neighbors=1, algorithm=self.nearest_neighbor_search).fit(global_path_array[:,:2])

        with self.lock:
            self.output_frame = msg.header.frame_id
            self.global_path_array = global_path_array
            self.global_path_waypoints = global_path_waypoints
            self.global_path_tree = global_path_tree


    def current_velocity_callback(self, msg):
        # save current velocity
        self.current_velocity = msg.twist.linear.x


    def current_pose_callback(self, msg):
        # save current pose
        self.current_pose = msg.pose.position


    def traffic_light_status_callback(self, msg):

        red_stop_lines = {}

        for result in msg.results:
            if result.lane_id in self.stop_lines and result.recognition_result == 0:
                red_stop_lines[result.lane_id] = self.stop_lines[result.lane_id]

        self.red_stop_lines = red_stop_lines


    def detected_objects_callback(self, msg):

        # get global path
        with self.lock:
            output_frame = self.output_frame
            global_path_array = self.global_path_array
            global_path_waypoints = self.global_path_waypoints
            global_path_tree = self.global_path_tree

        red_stop_lines = self.red_stop_lines
        current_pose = self.current_pose
        current_velocity = self.current_velocity

        # if global path or current pose is empty, publish empty local path, which stops the vehicle
        if global_path_array is None or current_pose is None:
            self.publish_local_path_wp([], msg.header.stamp, output_frame)
            return

        # extract local path points from global path
        wp_backward, _ = get_two_nearest_waypoint_idx(global_path_tree, current_pose.x, current_pose.y)
        end_index = wp_backward + self.local_path_length
        if end_index > len(global_path_array):
            end_index = len(global_path_array)

        # create local_path_array and extract waypoints
        local_path_array = global_path_array[wp_backward:end_index, :].copy()
        local_path_waypoints = copy.deepcopy(global_path_waypoints[wp_backward:end_index])

        # project current_pose to path
        current_pose_on_path = get_closest_point_on_line(Point(x = current_pose.x, y = current_pose.y, z = current_pose.z),
                                                         Point(x = local_path_array[0,0], y = local_path_array[0,1], z = local_path_array[0,2]),
                                                         Point(x = local_path_array[1,0], y = local_path_array[1,1], z = local_path_array[1,2]))

        # for all calculations consider the current pose as the first point of the local path
        local_path_array[0] = [current_pose_on_path.x, current_pose_on_path.y, current_pose_on_path.z, current_velocity, 0]
        local_path_waypoints[0].pose.pose.position = current_pose_on_path

        # if current position overlaps with the first waypoint, remove it
        if math.isclose(get_distance_between_two_points_2d(local_path_waypoints[0].pose.pose.position, local_path_waypoints[1].pose.pose.position), 0):
            local_path_array = local_path_array[1:]
            local_path_waypoints = local_path_waypoints[1:]
            # update wp_backward to reflect the new first waypoint - used to extract tfl distances
            wp_backward += 1

        # calculate distances up to each waypoint
        local_path_dists = np.cumsum(np.sqrt(np.sum(np.diff(local_path_array[:,:2], axis=0)**2, axis=1)))
        local_path_dists = np.insert(local_path_dists, 0, 0.0)

        # initialize closest object distance and velocity
        closest_object_distance = 0.0 
        closest_object_velocity = 0.0
        blocked = False

        points_list = []
        # fetch the transform from the object frame to the base_link frame to align the speed with ego vehicle
        try:
            transform = self.tf_buffer.lookup_transform("base_link", msg.header.frame_id, msg.header.stamp, rospy.Duration(self.transform_timeout))
        except (TransformException, rospy.ROSTimeMovedBackwardsException) as e:
            rospy.logwarn("%s - unable to transform object speed to base frame, using speed 0: %s", rospy.get_name(), e)
            transform = None

        # extract object points from detected objects
        for obj in msg.objects:
            # project object velocity to base_link frame to get longitudinal speed
            # TODO: project velocity to the path
            # in case there is no transform assume the object is not moving
            if transform is not None:
                velocity = transform_vector3(obj.velocity.linear, transform)
            else:
                velocity = Vector3()
            # add object center point and convex hull points to the points list
            points_list.append([obj.pose.position.x, obj.pose.position.y, obj.pose.position.z, velocity.x])
            for point in obj.convex_hull.polygon.points:
                points_list.append([point.x, point.y, point.z, velocity.x])

        # add stop line points to the points list
        for stop_line in red_stop_lines.values():

            tfl_local_path_index = stop_line['global_path_index'] - wp_backward
            # if the stop line is not on the local path, ignore it
            if tfl_local_path_index >= self.local_path_length or tfl_local_path_index <= 0:
                continue

            # calculate deceleration needed to stop for the traffic light
            deceleration = -(current_velocity**2) / (2 * local_path_dists[tfl_local_path_index])
            if deceleration < self.tfl_deceleration_limit:
                rospy.logwarn_throttle(3, "%s - ignore RED tfl, deceleration: %f", rospy.get_name(), deceleration)
            else:
                points_list.append(stop_line['location'])

        if len(points_list) > 0:
            # convert the points list to a numpy array
            obstacle_array = np.array(points_list)
            # create spatial index over obstacles for quick search
            obstacle_tree = NearestNeighbors(n_neighbors=1, algorithm=self.nearest_neighbor_search).fit(obstacle_array[:,:2])
            # find closest obstacle points to local path
            obstacle_dists, obstacle_idx = obstacle_tree.radius_neighbors(local_path_array[:,:2], self.car_safety_radius, return_distance=True)
            # calculate obstacle distances from the start of the local path
            obstacle_dists = [local_path_dists[i] + d for i, d in enumerate(obstacle_dists)]

            # If any calculated target_vel drops close to 0 then use this boolean to set all following wp speeds to 0
            zero_speeds_onwards = False

            # calculate velocity based on distance to obstacle using deceleration limit
            for i, wp in enumerate(local_path_waypoints):

                # mark waypoints with obstacles for visualizer
                if len(obstacle_idx[i]) > 0:
                    wp.cost = 1.0

                # once we get zero speed, keep it that way
                if zero_speeds_onwards:
                    wp.twist.twist.linear.x = 0.0
                    continue

                # list of points on path from the current waypoint
                obstacles_ahead_idx = np.concatenate(obstacle_idx[i:])
                if len(obstacles_ahead_idx) > 0:
                    # distances of obstacles ahead
                    obstacles_ahead_dists = np.concatenate(obstacle_dists[i:])

                    # subtract current waypoint distance from ahead distances
                    obstacles_ahead_dists -= local_path_dists[i]

                    # get speeds of those obstacles
                    obstacles_ahead_speeds = obstacle_array[obstacles_ahead_idx, 3]

                    # calculate stopping distances - following distance increased when obstacle has higher speed
                    stopping_distances = obstacles_ahead_dists - self.current_pose_to_car_front - self.braking_safety_distance \
                                            - self.braking_reaction_time * obstacles_ahead_speeds

                    # calculate target velocity based on stopping distance and deceleration limit
                    target_velocities = np.sqrt(np.maximum(0, obstacles_ahead_speeds**2 + 2 * self.speed_deceleration_limit * stopping_distances))

                    # pick object that causes the lowest target velocity
                    lowest_target_velocity_idx = np.argmin(target_velocities)
                    target_vel = target_velocities[lowest_target_velocity_idx]

                    # record the closest object from the first waypoint and decide if the lane is blocked
                    if i == 0:
                        # target_vel drops below the map velocity at current_pose (wp[0]) - object causing ego vehicle to slow down
                        if target_vel < wp.twist.twist.linear.x:
                            blocked = True

                        # closest object distance is calculated from the front of the car
                        closest_object_distance = obstacles_ahead_dists[lowest_target_velocity_idx] - self.current_pose_to_car_front
                        closest_object_velocity = obstacles_ahead_speeds[lowest_target_velocity_idx]

                    # overwrite target velocity of the waypoint if lower than the current one
                    wp.twist.twist.linear.x = min(target_vel, wp.twist.twist.linear.x)

                    # from stop point onwards all speeds are zero
                    if math.isclose(wp.twist.twist.linear.x, 0.0):
                        zero_speeds_onwards = True

        self.publish_local_path_wp(local_path_waypoints, msg.header.stamp, output_frame, closest_object_distance, closest_object_velocity, blocked)


    def publish_local_path_wp(self, local_path_waypoints, stamp, output_frame, closest_object_distance=0, closest_object_velocity=0, blocked=False):
        # create lane message
        lane = Lane()
        lane.header.frame_id = output_frame
        lane.header.stamp = stamp
        lane.waypoints = local_path_waypoints
        lane.closest_object_distance = closest_object_distance
        lane.closest_object_velocity = closest_object_velocity
        lane.is_blocked = blocked
        
        self.local_path_pub.publish(lane)


    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('velocity_local_planner')
    node = VelocityLocalPlanner()
    node.run()