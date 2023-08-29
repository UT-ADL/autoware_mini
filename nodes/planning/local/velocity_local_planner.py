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
        self.current_speed = 0.0
        self.stop_lines = {}
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
        current_position = self.current_position
        current_speed = self.current_speed

        # if global path or current pose is empty, publish empty local path, which stops the vehicle
        if global_path_array is None or current_position is None:
            self.publish_local_path_wp([], msg.header.stamp, output_frame)
            return

        # extract local path points from global path
        wp_backward, _ = get_two_nearest_waypoint_idx(global_path_tree, current_position.x, current_position.y)

        # calculate cumsum of distances between waypoints starting from wp_backward
        distances = np.cumsum(np.sqrt(np.sum(np.diff(global_path_array[wp_backward:,:2], axis=0)**2, axis=1)))
        distances = np.insert(distances, 0, 0.0)
        # return index of the waypoint equal or smaller with local_path_length
        ind = len(distances) - np.argmax((distances <= self.local_path_length)[::-1])
        # extract distances only for local path
        local_path_dists = distances[:ind]
        # calculate end_index in global_path context
        end_index = wp_backward + ind

        # create local_path_array and extract waypoints
        local_path_array = global_path_array[wp_backward:end_index, :].copy()
        local_path_waypoints = copy.deepcopy(global_path_waypoints[wp_backward:end_index])

        # project current_position to path
        current_position_on_path = get_closest_point_on_line(current_position, local_path_waypoints[0].pose.pose.position, local_path_waypoints[1].pose.pose.position)

        # for all calculations consider the current pose as the first point of the local path
        local_path_array[0] = [current_position_on_path.x, current_position_on_path.y, current_position_on_path.z, current_speed, 0]

        # interpolate dense local path and us it to calculate the closest object distance and velocity
        local_path_dense_dists = np.linspace(0, local_path_dists[-1], num=int(local_path_dists[-1] / self.dense_waypoint_interval))
        local_path_dense = self.interpolate_dense_local_path(local_path_dense_dists, local_path_dists, local_path_array)

        # initialize closest object distance and velocity
        closest_object_distance = 0.0 
        closest_object_velocity = 0.0
        blocked = False
        cost = 0.0

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
 
        # add stop line points to the points list
        for stop_line in red_stop_lines.values():

            tfl_local_path_index = stop_line['global_path_index'] - wp_backward
            # if the stop line is not on the local path, ignore it
            if tfl_local_path_index >= self.local_path_length or tfl_local_path_index <= 0:
                continue

            # calculate deceleration needed to stop for the traffic light
            deceleration = (current_speed**2) / (2 * local_path_dists[tfl_local_path_index])
            if deceleration > self.tfl_maximum_deceleration:
                rospy.logwarn_throttle(3, "%s - ignore RED tfl, deceleration: %f", rospy.get_name(), deceleration)
            else:
                points_list.append(stop_line['location'])

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
                obstacles_ahead_map_speeds = local_path_dense[obstacles_dense_path[:,0].astype(int)][:,3]

                # -----------------------------------------------------------------------------------------------
                ### Calculate target velocity for ALL OBSTACLES - as if they were within stopping_lateral_distance and we are following them
                # -----------------------------------------------------------------------------------------------
                obstacle_distances = obstacles_ahead_dists - self.current_pose_to_car_front
                # subtract braking_safety_distance and additionally reaction_time and obstacle_speed based distance for larger longitudinal distance when driving faster
                following_distances = obstacle_distances - self.braking_safety_distance - self.braking_reaction_time * obstacles_ahead_speeds
                # use following_distance and if sqr is negative use 0 - we do not want to drive!
                target_velocities_follow = np.sqrt(np.maximum(0, obstacles_ahead_speeds**2 + 2 * self.default_deceleration * following_distances))

                # -----------------------------------------------------------------------------------------------
                ### Calculate target velocity only for obstacles within slowdown_lateral_distance using the map speed
                # -----------------------------------------------------------------------------------------------
                scale = np.zeros_like(obstacles_ahead_map_speeds)
                target_velocities_close = np.zeros_like(obstacles_ahead_map_speeds)
                # create mask to select only obstacles in slowdown_lateral_distance and outside stopping_lateral_distance
                mask = obstacles_ahead_lateral_dists > self.stopping_lateral_distance

                # calculate scaling coefficient to transform lateral distance between 0 to 1
                scale[mask] = (obstacles_ahead_lateral_dists[mask] - self.stopping_lateral_distance ) / (self.slowdown_lateral_distance - self.stopping_lateral_distance)
                # abs is used to increase the target velocity once the car has passed, but the obstacle is still close (target velocity symmetric around "0 distance point" - car front)
                target_velocities_close[mask] = np.sqrt(np.maximum(0, (obstacles_ahead_map_speeds[mask] * scale[mask])**2 + abs(2 * self.default_deceleration * obstacle_distances[mask])))

                # -----------------------------------------------------------------------------------------------
                ### Final target velocities for all obstacles at ego car location
                # -----------------------------------------------------------------------------------------------
                target_velocities = np.maximum(target_velocities_close, target_velocities_follow)

                # Find obstacle causing smallest target velocity at ego car location
                lowest_target_velocity_idx = np.argmin(target_velocities)

                # If any calculated target_vel drops close to 0 then use this boolean to set all following wp speeds to 0
                zero_speeds_onwards = False
                target_velocity_close = 0.0
                
                # calculate target velocity for each waypoint - only one obstacle (causing lowest target velocity) is considered inside for loop 
                # TODO: problem - distances using array (first point changed) and here we are using waypoints!!!
                for i, wp in enumerate(local_path_waypoints):

                    # once we get zero speed, keep it that way
                    if zero_speeds_onwards:
                        wp.twist.twist.linear.x = 0.0
                        continue

                    # obstacle distance from car front
                    obstacle_distance = obstacles_ahead_dists[lowest_target_velocity_idx] - local_path_dists[i] - self.current_pose_to_car_front 

                    # calculate target velocity in case of obstacle in slowdown_lateral_distance
                    if obstacles_ahead_lateral_dists[lowest_target_velocity_idx] > self.stopping_lateral_distance:
                        # abs is used to increase the target velocity once the car has passed, but the obstacle is still close
                        target_velocity_close = np.sqrt(np.maximum(0, (obstacles_ahead_map_speeds[lowest_target_velocity_idx]*scale[lowest_target_velocity_idx])**2 + abs(2 * self.default_deceleration * obstacle_distance)))

                    # calculate target velocity as if it is blocking the lane (inisde stopping_lateral_distance)
                    following_distance = obstacle_distance - self.braking_safety_distance - self.braking_reaction_time * obstacles_ahead_speeds[lowest_target_velocity_idx]
                    target_velocity_follow = np.sqrt(np.maximum(0, obstacles_ahead_speeds[lowest_target_velocity_idx]**2 + 2 * self.default_deceleration * following_distance))

                    # record the closest object from the first waypoint and decide if the lane is blocked
                    if i == 0:
                        if obstacles_ahead_lateral_dists[lowest_target_velocity_idx] <= self.stopping_lateral_distance:
                            blocked = True

                        cost = scale[lowest_target_velocity_idx]

                        # closest object distance is calculated from the front of the car
                        closest_object_distance = obstacles_ahead_dists[lowest_target_velocity_idx] - self.current_pose_to_car_front
                        closest_object_velocity = obstacles_ahead_speeds[lowest_target_velocity_idx]

                    # overwrite target velocity of wp
                    wp.twist.twist.linear.x = min(max(target_velocity_follow, target_velocity_close), wp.twist.twist.linear.x)

                    # from stop point onwards all speeds are set to zero
                    if math.isclose(wp.twist.twist.linear.x, 0.0):
                        zero_speeds_onwards = True

        self.publish_local_path_wp(local_path_waypoints, msg.header.stamp, output_frame, closest_object_distance, closest_object_velocity, blocked, cost)

        self.publish_local_path_wp(local_path_waypoints, msg.header.stamp, output_frame, closest_object_distance, closest_object_velocity, blocked, cost)


    def interpolate_dense_local_path(self, local_path_dense_dists, local_path_dists, local_path_array):

        # interpolate x_new, y_new, z_new and v_new from local_path_array
        x_new = np.interp(local_path_dense_dists, local_path_dists, local_path_array[:,0])
        y_new = np.interp(local_path_dense_dists, local_path_dists, local_path_array[:,1])
        z_new = np.interp(local_path_dense_dists, local_path_dists, local_path_array[:,2])
        v_new = np.interp(local_path_dense_dists, local_path_dists, local_path_array[:,3])

        return np.stack((x_new, y_new, z_new, v_new), axis=1)


    def publish_local_path_wp(self, local_path_waypoints, stamp, output_frame, closest_object_distance=0, closest_object_velocity=0, blocked=False, cost=0.0):
        # create lane message
        lane = Lane()
        lane.header.frame_id = output_frame
        lane.header.stamp = stamp
        lane.waypoints = local_path_waypoints
        lane.closest_object_distance = closest_object_distance
        lane.closest_object_velocity = closest_object_velocity
        lane.is_blocked = blocked
        lane.cost = cost
        self.local_path_pub.publish(lane)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('velocity_local_planner')
    node = VelocityLocalPlanner()
    node.run()