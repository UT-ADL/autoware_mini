#!/usr/bin/env python3

import rospy
import copy
import math
import threading
from tf2_ros import Buffer, TransformListener, TransformException

import numpy as np
from sklearn.neighbors import NearestNeighbors

from autoware_msgs.msg import Lane, DetectedObjectArray, TrafficLightResultArray
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3

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
        self.stopping_lateral_distance = rospy.get_param("stopping_lateral_distance")
        self.slowdown_lateral_distance = rospy.get_param("slowdown_lateral_distance")
        self.waypoint_interval = rospy.get_param("waypoint_interval")
        self.dense_waypoint_interval = rospy.get_param("~dense_waypoint_interval")
        self.current_pose_to_car_front = rospy.get_param("current_pose_to_car_front")
        self.default_deceleration = rospy.get_param("default_deceleration")
        self.tfl_maximum_deceleration = rospy.get_param("~tfl_maximum_deceleration")

        # Internal variables
        self.lock = threading.Lock()
        self.output_frame = None
        self.global_path_array = None
        self.global_path_waypoints = None
        self.global_path_tree = None
        self.current_position = None
        self.current_speed = None
        self.red_stop_lines = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
        self.waypoint_lookup_radius = np.sqrt(self.slowdown_lateral_distance**2 + self.dense_waypoint_interval**2)

        # Publishers
        self.local_path_pub = rospy.Publisher('local_path', Lane, queue_size=1)

        # Subscribers
        rospy.Subscriber('smoothed_path', Lane, self.path_callback, queue_size=1)
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1)
        rospy.Subscriber('/localization/current_velocity', TwistStamped, self.current_velocity_callback, queue_size=1)
        rospy.Subscriber('/detection/final_objects', DetectedObjectArray, self.detected_objects_callback, queue_size=1)
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
                wp.pose.pose.position.y
            ) for wp in msg.waypoints])

        # create global_wp_tree
        global_path_tree = NearestNeighbors(n_neighbors=1, algorithm=self.nearest_neighbor_search)
        global_path_tree.fit(global_path_array[:,:2])

        with self.lock:
            self.output_frame = msg.header.frame_id
            self.global_path_array = global_path_array
            self.global_path_waypoints = global_path_waypoints
            self.global_path_tree = global_path_tree


    def current_velocity_callback(self, msg):
        # save current velocity
        self.current_speed = msg.twist.linear.x


    def current_pose_callback(self, msg):
        # save current pose
        self.current_position = msg.pose.position


    def traffic_light_status_callback(self, msg):

        red_stop_lines = {}
        for result in msg.results:
            if result.recognition_result == 0:
                red_stop_lines[result.lane_id] = result.recognition_result

        self.red_stop_lines = red_stop_lines


    def detected_objects_callback(self, msg):

        # get global path
        with self.lock:
            output_frame = self.output_frame
            global_path_array = self.global_path_array
            global_path_waypoints = self.global_path_waypoints
            global_path_tree = self.global_path_tree

        red_stop_lines = self.red_stop_lines
        current_position = self.current_position
        current_speed = self.current_speed

        # if global path or current pose is empty, publish empty local path, which stops the vehicle
        if global_path_array is None or current_position is None or current_speed is None:
            self.publish_local_path_wp([], msg.header.stamp, output_frame)
            return

        # extract local path points from global path
        wp_backward, _ = get_two_nearest_waypoint_idx(global_path_tree, current_position.x, current_position.y)
        end_index = min(wp_backward + int(self.local_path_length / self.waypoint_interval), len(global_path_array))

        # create local_path_array and extract waypoints
        local_path_array = global_path_array[wp_backward:end_index, :].copy()
        local_path_waypoints = copy.deepcopy(global_path_waypoints[wp_backward:end_index])

        # project current_position to path
        current_position_on_path = get_closest_point_on_line(current_position, local_path_waypoints[0].pose.pose.position, local_path_waypoints[1].pose.pose.position)

        # for all calculations consider the current pose as the first point of the local path
        local_path_array[0] = [current_position_on_path.x, current_position_on_path.y]

        # calculate distances up to each waypoint
        local_path_dists = np.cumsum(np.sqrt(np.sum(np.diff(local_path_array[:,:2], axis=0)**2, axis=1)))
        local_path_dists = np.insert(local_path_dists, 0, 0.0)

        # interpolate dense local path and us it to calculate the closest object distance and velocity
        local_path_dense_dists = np.linspace(0, local_path_dists[-1], num=int(local_path_dists[-1] / self.dense_waypoint_interval))
        x_new = np.interp(local_path_dense_dists, local_path_dists, local_path_array[:,0])
        y_new = np.interp(local_path_dense_dists, local_path_dists, local_path_array[:,1])
        local_path_dense = np.stack((x_new, y_new), axis=1)

        # initialize closest object distance and velocity
        closest_object_distance = 0.0 
        closest_object_velocity = 0.0
        blocked = False
        increment = 0

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
            # add predicted path points to the points list, if they exist
            #if len(obj.candidate_trajectories.lanes) > 0:
            #    for wp in obj.candidate_trajectories.lanes[0].waypoints:
            #        points_list.append([wp.pose.pose.position.x, wp.pose.pose.position.y, wp.pose.pose.position.z, velocity.x])

        # Red traffic lights: add wp with stop line id's where the traffic light status is red to obstacle list
        for i, wp in enumerate(local_path_waypoints):

            if wp.stop_line_id > 0 and wp.stop_line_id in red_stop_lines:
                # use wp distance and caclulate necessary deceleration for stopping
                deceleration = (current_speed**2) / (2 * (local_path_dists[i] - self.current_pose_to_car_front))

                if deceleration > self.tfl_maximum_deceleration:
                    rospy.logwarn_throttle(3, "%s - ignore RED tfl, deceleration: %f", rospy.get_name(), deceleration)
                else:
                    points_list.append([wp.pose.pose.position.x, wp.pose.pose.position.y, wp.pose.pose.position.z, 0.0])

        if len(points_list) > 0:
            # convert the points list to a numpy array
            obstacle_array = np.array(points_list)
            # create spatial index over obstacles for quick search
            obstacle_tree = NearestNeighbors(n_neighbors=1, algorithm=self.nearest_neighbor_search).fit(obstacle_array[:,:2])
            # find closest local_path points to obstacles - lateral dist from path
            obstacle_d, obstacle_idx = obstacle_tree.radius_neighbors(local_path_dense[:,:2], self.waypoint_lookup_radius, return_distance=True)

            # create indexes for obstacles with respect to local_path_dense
            index_array = np.repeat(np.arange(len(obstacle_idx)), [len(i) for i in obstacle_idx])
            obstacles_dense_path = np.column_stack((index_array, np.concatenate(obstacle_idx), np.concatenate(obstacle_d)))

            # filter with self.slowdown_lateral_distance (since nearest neighbor search is a "circle" some points might be slightly over the limit)
            obstacles_dense_path = obstacles_dense_path[obstacles_dense_path[:,2] <= self.slowdown_lateral_distance]

            if len(obstacles_dense_path) > 0:

                # sort and for all obstacle points keep only the one with min lateral distance - closest to being perpendicular with path
                obstacles_dense_path = obstacles_dense_path[obstacles_dense_path[:,2].argsort()]
                _, unique_indices = np.unique(obstacles_dense_path[:, 1], return_index=True)
                obstacles_dense_path = obstacles_dense_path[unique_indices]

                # Extract necessary arrays
                obstacles_ahead_speeds = obstacle_array[obstacles_dense_path[:,1].astype(int)][:,3]
                obstacles_ahead_dists = local_path_dense_dists[obstacles_dense_path[:,0].astype(int)]
                obstacles_ahead_lateral_dists = obstacles_dense_path[:,2]

                # ALL OBSTACLES: Calculate target velocity as if they were within stopping_lateral_distance
                obstacle_distances = obstacles_ahead_dists - self.current_pose_to_car_front
                # subtract braking_safety_distance and additionally reaction_time and obstacle_speed based distance for larger longitudinal distance when driving faster
                following_distances = obstacle_distances - self.braking_safety_distance - self.braking_reaction_time * obstacles_ahead_speeds
                # use following_distance and if sqrt is negative use 0 - we do not want to drive!
                target_velocities = np.sqrt(np.maximum(0, obstacles_ahead_speeds**2 + 2 * self.default_deceleration * following_distances))

                # OBS IN SLOWDOWN area: Calculate target velocity only for obstacles within slowdown_lateral_distance
                scale = np.zeros_like(target_velocities)
                target_velocities_slowdown = np.zeros_like(target_velocities)
                # create mask to select only obstacles in slowdown_lateral_distance and outside stopping_lateral_distance
                mask = obstacles_ahead_lateral_dists > self.stopping_lateral_distance

                # calculate scaling coefficient to transform lateral distance between 0 to 1
                scale[mask] = (obstacles_ahead_lateral_dists[mask] - self.stopping_lateral_distance ) / (self.slowdown_lateral_distance - self.stopping_lateral_distance)
                # calculate target velocity: scale possible max and min velocity  difference and add to min velocity
                target_velocities_slowdown[mask] = scale[mask] * np.maximum((local_path_waypoints[0].twist.twist.linear.x - target_velocities[mask]), 0) + target_velocities[mask]

                # Update target_velocities with obstacles within slowdown_lateral_distance
                target_velocities[mask] = target_velocities_slowdown[mask]

                # Find obstacle causing smallest target velocity at ego car location
                lowest_target_velocity_idx = np.argmin(target_velocities)

                # If any calculated target_vel drops close to 0 then use this boolean to set all following wp speeds to 0
                zero_speeds_onwards = False

                # calculate target velocity for each waypoint - only one obstacle (causing lowest target velocity) is considered inside for loop 
                # TODO: problem - distances using array (first point changed) and here we are using waypoints!!!
                for i, wp in enumerate(local_path_waypoints):

                    # once we get zero speed, keep it that way
                    if zero_speeds_onwards:
                        wp.twist.twist.linear.x = 0.0
                        continue

                    # obstacle distance from car front
                    obstacle_distance = obstacles_ahead_dists[lowest_target_velocity_idx] - local_path_dists[i] - self.current_pose_to_car_front 

                    # calculate target velocity as if it is blocking the lane (inisde stopping_lateral_distance)
                    following_distance = obstacle_distance - self.braking_safety_distance - self.braking_reaction_time * obstacles_ahead_speeds[lowest_target_velocity_idx]
                    target_velocity = np.sqrt(np.maximum(0, obstacles_ahead_speeds[lowest_target_velocity_idx]**2 + 2 * self.default_deceleration * following_distance))

                    # calculate target velocity in case of obstacle in slowdown_lateral_distance
                    if obstacles_ahead_lateral_dists[lowest_target_velocity_idx] > self.stopping_lateral_distance:
                        target_velocity = scale[lowest_target_velocity_idx] * np.maximum((local_path_waypoints[0].twist.twist.linear.x - target_velocity), 0) + target_velocity

                    # target velocity cannot be higher than the map based speed
                    target_velocity = min(target_velocity, wp.twist.twist.linear.x)

                    # record the closest object from the first waypoint and decide if the lane is blocked
                    if i == 0:

                        # closest object distance is calculated from the front of the car
                        closest_object_distance = obstacles_ahead_dists[lowest_target_velocity_idx] - self.current_pose_to_car_front
                        closest_object_velocity = obstacles_ahead_speeds[lowest_target_velocity_idx]

                        # obstacle has to be at least within slowdown_lateral_distance
                        increment = 1

                        # obstacle within stopping_lateral_distance - blocking
                        if obstacles_ahead_lateral_dists[lowest_target_velocity_idx] <= self.stopping_lateral_distance:
                            blocked = True
                            increment = 2

                            # obtacle blocking and causing lower target velocity than current wp speed (map based speed) - following car
                            if target_velocity < wp.twist.twist.linear.x:
                                increment = 3

                                # obstacle blocking and not moving - stopping
                                if closest_object_velocity < 1.0:
                                    increment = 4

                    # overwrite target velocity of wp
                    wp.twist.twist.linear.x = target_velocity

                    # from stop point onwards all speeds are set to zero
                    if math.isclose(wp.twist.twist.linear.x, 0.0):
                        zero_speeds_onwards = True

        self.publish_local_path_wp(local_path_waypoints, msg.header.stamp, output_frame, closest_object_distance, closest_object_velocity, blocked, increment)


    def publish_local_path_wp(self, local_path_waypoints, stamp, output_frame, closest_object_distance=0.0, closest_object_velocity=0.0, blocked=False, increment=0):
        # create lane message
        lane = Lane()
        lane.header.frame_id = output_frame
        lane.header.stamp = stamp
        lane.waypoints = local_path_waypoints
        lane.closest_object_distance = closest_object_distance
        lane.closest_object_velocity = closest_object_velocity
        lane.is_blocked = blocked
        lane.increment = increment
        self.local_path_pub.publish(lane)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('velocity_local_planner')
    node = VelocityLocalPlanner()
    node.run()