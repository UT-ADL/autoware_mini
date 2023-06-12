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

from helpers.timer import Timer

class VelocityLocalPlanner:

    def __init__(self):

        # Parameters
        self.local_path_length = rospy.get_param("~local_path_length")
        self.nearest_neighbor_search = rospy.get_param("~nearest_neighbor_search")
        self.transform_timeout = rospy.get_param("~transform_timeout")
        self.braking_safety_distance = rospy.get_param("braking_safety_distance")
        self.braking_reaction_time = rospy.get_param("braking_reaction_time")
        self.car_safety_width = rospy.get_param("car_safety_width")
        self.wp_safety_radius = rospy.get_param("wp_safety_radius")
        self.current_pose_to_car_front = rospy.get_param("current_pose_to_car_front")
        self.speed_deceleration_limit = rospy.get_param("speed_deceleration_limit")
        self.publish_debug_info = rospy.get_param("~publish_debug_info")

        lanelet2_map_name = rospy.get_param("~lanelet2_map_name")
        coordinate_transformer = rospy.get_param("/localization/coordinate_transformer")
        use_custom_origin = rospy.get_param("/localization/use_custom_origin")
        utm_origin_lat = rospy.get_param("/localization/utm_origin_lat")
        utm_origin_lon = rospy.get_param("/localization/utm_origin_lon")

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
        if self.publish_debug_info:
            self.collision_points_pub = rospy.Publisher('collision_points', Marker, queue_size=1)

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

        t = Timer()

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

        ## INTERPOLATE DENSE LOCAL PATH
        # make local_path dense 10cm? - TODO: hardcoded 0.1 m
        local_path_dense_dists = np.linspace(0, local_path_dists[-1], num=int(local_path_dists[-1] / 0.1))

        # interpolate x_new, y_new, z_new
        x_new = np.interp(local_path_dense_dists, local_path_dists, local_path_array[:,0])
        y_new = np.interp(local_path_dense_dists, local_path_dists, local_path_array[:,1])
        z_new = np.interp(local_path_dense_dists, local_path_dists, local_path_array[:,2])

        local_path_dense = np.stack((x_new, y_new, z_new), axis=1)

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
            # find closest obstacle local?path points to obstacles - lateral dist from path
            obstacle_d, obstacle_idx = obstacle_tree.radius_neighbors(local_path_dense[:,:2], self.wp_safety_radius, return_distance=True)
            # create indexes for obstacles with respect to local_path_dense
            index_array = np.repeat(np.arange(len(obstacle_idx)), [len(i) for i in obstacle_idx])
            obstacles_dense_path = np.vstack((index_array, np.concatenate(obstacle_idx), np.concatenate(obstacle_d))).T
            # sort and min d
            obstacles_dense_path = obstacles_dense_path[obstacles_dense_path[:,2].argsort()]
            _, unique_indices = np.unique(obstacles_dense_path[:, 1], return_index=True)
            obstacles_dense_path = obstacles_dense_path[unique_indices]
            print("obstacles_dense_path: \n", obstacles_dense_path)

            #filtered_array = np.delete(unique_rows, np.where(unique_rows[:,2] > self.car_safety_width), axis=0)
            # print("filtered_array: \n", filtered_array)

            obstacle_speed = obstacle_array[obstacles_dense_path[:,1].astype(int)][:,3]
            obstacle_dists = local_path_dense_dists[obstacles_dense_path[:,0].astype(int)]


            # If any calculated target_vel drops close to 0 then use this boolean to set all following wp speeds to 0
            zero_speeds_onwards = False

            # list of points on path from the current waypoint
            obstacles_ahead_idx = np.unique(np.concatenate(obstacle_idx[:]))
            obstacles_on_local_path = obstacle_array[obstacles_ahead_idx]
            obstacles_within_car_safety_width = self.filter_obstacles_using_car_safety_width(obstacles_on_local_path, global_path_tree, global_path_waypoints, wp_backward, local_path_dists)

            if len(obstacles_within_car_safety_width) > 0:

                # calculate velocity based on distance to obstacle using deceleration limit
                for i, wp in enumerate(local_path_waypoints):

                    # once we get zero speed, keep it that way
                    if zero_speeds_onwards:
                        wp.twist.twist.linear.x = 0.0
                        continue

                    obstacles_ahead_dists = obstacles_within_car_safety_width[:,1]
                    obstacles_ahead_speeds = obstacles_within_car_safety_width[:,3]

                    # subtract current waypoint distance from ahead distances
                    obstacles_ahead_dists -= local_path_dists[i]

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
        if self.publish_debug_info:
            self.publish_collision_points(obstacles_within_car_safety_width, msg.header.stamp, output_frame)

        t('obs_cb end')
        print(t)

    def filter_obstacles_using_car_safety_width(self, obstacles_on_local_path, global_path_tree, global_path_waypoints, global_wp_backward, local_path_dists):

        obstacles_within_car_safety_width = []
        # iterate over obstacle_array_unique and calc 2 closest waypoint indexes
        for x, y, z, v in obstacles_on_local_path:
            wp_backward, wp_forward = get_two_nearest_waypoint_idx(global_path_tree, x, y)
            # if obstacle in between or after 2 last wp in global path, don't clamp calculated point_on_line coordinates
            if wp_forward == len(global_path_waypoints) - 1:
                closest_point = get_closest_point_on_line(Point(x=x, y=y, z=z), global_path_waypoints[wp_backward].pose.pose.position, global_path_waypoints[wp_forward].pose.pose.position, clamp_output=False)
            else:
                closest_point = get_closest_point_on_line(Point(x=x, y=y, z=z), global_path_waypoints[wp_backward].pose.pose.position, global_path_waypoints[wp_forward].pose.pose.position)

            d_from_path = get_distance_between_two_points_2d(Point(x=x, y=y, z=z), closest_point)
            # ignore obstacles that are further than car_safety_width
            if d_from_path > self.car_safety_width:
                continue

            d_from_prev_wp = get_distance_between_two_points_2d(global_path_waypoints[wp_backward].pose.pose.position, closest_point)
            wp_ind_local = wp_backward - global_wp_backward

            # if wp_safety_radius is large then it can include obstacle that are not within the reach of local_path
            if wp_ind_local > 99:
                continue

            d_from_localpath_start = local_path_dists[wp_ind_local] + d_from_prev_wp
            obstacles_within_car_safety_width.append([wp_ind_local, d_from_localpath_start, d_from_path, v, closest_point.x, closest_point.y, closest_point.z])

        return np.array(obstacles_within_car_safety_width)



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

    def publish_collision_points(self, obstacles_within_car_safety_width, stamp, output_frame):

        marker = Marker()
        marker.header.frame_id = output_frame
        marker.header.stamp = stamp
        marker.ns = "Collision points"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color = ColorRGBA(a=1.0, r=1.0, g=0.0, b=0.0)
        for _, _, _, _, cx, cy, cz in obstacles_within_car_safety_width:
            marker.points.append(Point(x=cx, y=cy, z=cz))

        self.collision_points_pub.publish(marker)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('velocity_local_planner')
    node = VelocityLocalPlanner()
    node.run()