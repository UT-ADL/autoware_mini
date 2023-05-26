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
from visualization_msgs.msg import Marker

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
        self.car_safety_width = rospy.get_param("car_safety_width")
        self.close_obstacle_limit = rospy.get_param("close_obstacle_limit")
        self.wp_safety_radius = rospy.get_param("wp_safety_radius")
        self.dense_wp_interval = rospy.get_param("~dense_wp_interval")
        self.current_pose_to_car_front = rospy.get_param("current_pose_to_car_front")
        self.speed_deceleration_limit = rospy.get_param("speed_deceleration_limit")

        lanelet2_map_name = rospy.get_param("~lanelet2_map_name")
        coordinate_transformer = rospy.get_param("/localization/coordinate_transformer")
        use_custom_origin = rospy.get_param("/localization/use_custom_origin")
        utm_origin_lat = rospy.get_param("/localization/utm_origin_lat")
        utm_origin_lon = rospy.get_param("/localization/utm_origin_lon")

        # TODO: temporary - need to remove?
        self.waypoint_interval = rospy.get_param("waypoint_interval")
        # TODO - will the calculation give exact result - circles intersect at that distance?
        self.car_safety_radius = math.sqrt(self.car_safety_width**2 + (self.waypoint_interval / 2.0)**2)

        # Load the map using Lanelet2
        if coordinate_transformer == "utm":
            projector = UtmProjector(Origin(utm_origin_lat, utm_origin_lon), use_custom_origin, False)
        else:
            rospy.logfatal("%s - only utm and custom origin currently supported for lanelet2 map loading", rospy.get_name())
            exit(1)

        self.lanelet2_map = load(lanelet2_map_name, projector)

        # Internal variables
        self.lock = threading.Lock()
        self.output_frame = None
        self.global_path_array = None
        self.global_path_waypoints = None
        self.global_path_tree = None
        self.current_pose = None
        self.current_velocity = 0.0
        self.stop_lines = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

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
                wp.twist.twist.linear.x
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
        self.current_velocity = msg.twist.linear.x


    def current_pose_callback(self, msg):
        # save current pose
        self.current_pose = msg.pose.position


    def traffic_light_status_callback(self, msg):
        stop_lines = {}
        for result in msg.results:
            # check if we have already outputted the status of this stopline
            if result.lane_id in stop_lines:
                assert result.recognition_result == 0, "traffic light status callback - multiple stoplines with different status"
                continue

            # add stop line points to the list if the traffic light is red or yellow
            if result.recognition_result == 0:
                stop_line = self.lanelet2_map.lineStringLayer.get(result.lane_id)
                stop_lines[result.lane_id] = stop_line

        self.stop_lines = stop_lines


    def detected_objects_callback(self, msg):

        # get global path
        with self.lock:
            output_frame = self.output_frame
            global_path_array = self.global_path_array
            global_path_waypoints = self.global_path_waypoints
            global_path_tree = self.global_path_tree

        # no need for lock, because assignment is atomic in Python
        stop_lines = self.stop_lines
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
        current_pose_on_path = get_closest_point_on_line(current_pose, local_path_waypoints[0].pose.pose.position, local_path_waypoints[1].pose.pose.position)

        # for all calculations consider the current pose as the first point of the local path
        local_path_array[0] = [current_pose_on_path.x, current_pose_on_path.y, current_pose_on_path.z, current_velocity]
        local_path_waypoints[0].pose.pose.position = current_pose_on_path

        # if current position overlaps with the first waypoint, remove it
        if math.isclose(get_distance_between_two_points_2d(local_path_waypoints[0].pose.pose.position, local_path_waypoints[1].pose.pose.position), 0):
            local_path_array = local_path_array[1:]
            local_path_waypoints = local_path_waypoints[1:]

        # calculate distances up to each waypoint
        local_path_dists = np.cumsum(np.sqrt(np.sum(np.diff(local_path_array[:,:2], axis=0)**2, axis=1)))
        local_path_dists = np.insert(local_path_dists, 0, 0.0)

        # interpolate dense local path and us it to calculate the closest object distance and velocity
        local_path_dense_dists = np.linspace(0, local_path_dists[-1], num=int(local_path_dists[-1] / self.dense_wp_interval))
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
        for stop_line in stop_lines.values():
            for point in stop_line:
                points_list.append([point.x, point.y, point.z, 0.0])

        if len(points_list) > 0:
            # convert the points list to a numpy array
            obstacle_array = np.array(points_list)
            # create spatial index over obstacles for quick search
            obstacle_tree = NearestNeighbors(n_neighbors=1, algorithm=self.nearest_neighbor_search).fit(obstacle_array[:,:2])
            # find closest local_path points to obstacles - lateral dist from path
            obstacle_d, obstacle_idx = obstacle_tree.radius_neighbors(local_path_dense[:,:2], self.wp_safety_radius, return_distance=True)

            # create indexes for obstacles with respect to local_path_dense
            index_array = np.repeat(np.arange(len(obstacle_idx)), [len(i) for i in obstacle_idx])
            obstacles_dense_path = np.vstack((index_array, np.concatenate(obstacle_idx), np.concatenate(obstacle_d))).T
            
            # filter with self.close_obstacle_limit
            obstacles_dense_path = obstacles_dense_path[obstacles_dense_path[:,2] < self.close_obstacle_limit]

            if len(obstacles_dense_path) > 0:

                # sort and for all obstacle points keep only the one with min distance
                obstacles_dense_path = obstacles_dense_path[obstacles_dense_path[:,2].argsort()]
                _, unique_indices = np.unique(obstacles_dense_path[:, 1], return_index=True)
                obstacles_dense_path = obstacles_dense_path[unique_indices]

                # Extract necessary arrays
                obstacles_ahead_speeds = obstacle_array[obstacles_dense_path[:,1].astype(int)][:,3]
                obstacles_ahead_dists = local_path_dense_dists[obstacles_dense_path[:,0].astype(int)]
                obstacles_ahead_lateral_dists = obstacles_dense_path[:,2]
                obstacles_ahead_map_speeds = local_path_dense[obstacles_dense_path[:,0].astype(int)][:,3]
                obstacles_ahead_map_speeds_scaled = self.scale_speeds_using_lateral_distance(obstacles_ahead_map_speeds, obstacles_ahead_lateral_dists)

                obstacles_ahead_target_velocity = self.calculate_target_velocity(obstacles_ahead_speeds, obstacles_ahead_dists, obstacles_ahead_map_speeds_scaled, obstacles_ahead_lateral_dists)

                # find obstacle causing smallest target velocity at current pose
                lowest_target_velocity_idx = np.argmin(obstacles_ahead_target_velocity)

                # If any calculated target_vel drops close to 0 then use this boolean to set all following wp speeds to 0
                zero_speeds_onwards = False
                target_velocity_close = 0.0
                
                # calculate target velocity for each waypoint - only one obstacle (causing lowest target velocity) is considered inside for loop 
                for i, wp in enumerate(local_path_waypoints):

                    # once we get zero speed, keep it that way
                    if zero_speeds_onwards:
                        wp.twist.twist.linear.x = 0.0
                        continue

                    # obstacle distance from car front
                    obstacle_distance = obstacles_ahead_dists[lowest_target_velocity_idx] - local_path_dists[i] - self.current_pose_to_car_front 

                    # calculate target velocity in case of obstacle close to path
                    if obstacles_ahead_lateral_dists[lowest_target_velocity_idx] > self.car_safety_width:
                        # abs is used to increase the target velocity once the car has passed, but the obstacle is still close
                        target_velocity_close = np.sqrt(np.maximum(0, obstacles_ahead_map_speeds_scaled[lowest_target_velocity_idx]**2 + abs(2 * self.speed_deceleration_limit * obstacle_distance)))

                    # calculate target velocity in case of following the obstacle
                    following_distance = obstacle_distance - self.braking_safety_distance - self.braking_reaction_time * obstacles_ahead_speeds[lowest_target_velocity_idx]
                    target_velocity_follow = np.sqrt(np.maximum(0, obstacles_ahead_speeds[lowest_target_velocity_idx]**2 + 2 * self.speed_deceleration_limit * following_distance))

                    # record the closest object from the first waypoint and decide if the lane is blocked
                    if i == 0:
                        # target_vel drops below the map velocity at current_pose (wp[0]) - object causing ego vehicle to slow down
                        if target_velocity_follow < wp.twist.twist.linear.x:
                            blocked = True
                        if obstacles_ahead_lateral_dists[lowest_target_velocity_idx] > self.car_safety_width:
                            # cost calculated here is used only for visualization decisions
                            cost = (obstacles_ahead_lateral_dists[lowest_target_velocity_idx] - self.car_safety_width)  / (self.close_obstacle_limit - self.car_safety_width)

                        # closest object distance is calculated from the front of the car
                        closest_object_distance = obstacles_ahead_dists[lowest_target_velocity_idx] - self.current_pose_to_car_front
                        closest_object_velocity = obstacles_ahead_speeds[lowest_target_velocity_idx]

                    # overwrite target velocity of wp
                    wp.twist.twist.linear.x = min(max(target_velocity_follow, target_velocity_close), wp.twist.twist.linear.x)

                    # from stop point onwards all speeds are zero
                    if math.isclose(wp.twist.twist.linear.x, 0.0):
                        zero_speeds_onwards = True

        self.publish_local_path_wp(local_path_waypoints, msg.header.stamp, output_frame, closest_object_distance, closest_object_velocity, blocked, cost)


    def scale_speeds_using_lateral_distance(self, map_speed, lateral_dist):

        scale = np.zeros_like(map_speed)
        # calculate scaling coefficient to transform lateral distance between 0 to 1  using self.car_safety_width and self.close_obstacle_limit
        scale[(lateral_dist >= self.car_safety_width)] = (lateral_dist[(lateral_dist >= self.car_safety_width )] - self.car_safety_width ) \
             / (self.close_obstacle_limit - self.car_safety_width)
        # these should be filtered out, but just in case
        scale[lateral_dist > self.close_obstacle_limit] = 1

        return map_speed * scale


    def calculate_target_velocity(self, obstacles_ahead_speeds, obstacles_ahead_dists, obstacles_ahead_map_speeds_scaled, obstacles_ahead_lateral_dists):

        obstacle_distances = obstacles_ahead_dists - self.current_pose_to_car_front

        # for all obstacles calculate target velocity based on obstacle speed and distance to obstacle - following
        following_distances = obstacle_distances - self.braking_safety_distance - self.braking_reaction_time * obstacles_ahead_speeds
        target_velocities_follow = np.sqrt(np.maximum(0, obstacles_ahead_speeds**2 + 2 * self.speed_deceleration_limit * following_distances))

        # TODO only obstacles_ahead_dists or add - self.current_pose_to_car_front - self.braking_safety_distance?
        target_velocities_close = np.sqrt(np.maximum(0, obstacles_ahead_map_speeds_scaled**2 + 2 * self.speed_deceleration_limit * obstacle_distances))

        # combine target velocities from following and close to the path obstacles
        target_velocities = np.where(obstacles_ahead_lateral_dists <= self.car_safety_width, target_velocities_follow, np.maximum(target_velocities_close, target_velocities_follow))

        return target_velocities


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