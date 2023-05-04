#!/usr/bin/env python3

import rospy
import copy
import math
import threading
import tf

from lanelet2.io import Origin, load
from lanelet2.projection import UtmProjector

import numpy as np
from sklearn.neighbors import NearestNeighbors

from autoware_msgs.msg import Lane, DetectedObjectArray, TrafficLightResultArray
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped, Point
from std_msgs.msg import ColorRGBA

from helpers import get_two_nearest_waypoint_idx, get_closest_point_on_line, get_distance_between_two_points

GREEN = ColorRGBA(0.0, 1.0, 0.0, 0.4)
RED = ColorRGBA(1.0, 0.0, 0.0, 0.4)
YELLOW = ColorRGBA(0.8, 0.8, 0.0, 0.4)


class SpeedOnlyLocalPlanner:

    def __init__(self):

        # Parameters
        self.local_path_length = rospy.get_param("~local_path_length", 100)
        self.nearest_neighbor_search = rospy.get_param("~nearest_neighbor_search", "kd_tree")
        self.braking_safety_distance = rospy.get_param("~braking_safety_distance", 4.0)
        self.braking_reaction_time = rospy.get_param("~braking_reaction_time", 1.6)
        self.car_safety_radius = rospy.get_param("~car_safety_radius", 1.3)
        self.current_pose_to_car_front = rospy.get_param("~current_pose_to_car_front", 4.0)
        self.speed_deceleration_limit = rospy.get_param("~speed_deceleration_limit", 1.0)

        lanelet2_map_name = rospy.get_param("~lanelet2_map_name")
        coordinate_transformer = rospy.get_param("/localization/coordinate_transformer")
        use_custom_origin = rospy.get_param("/localization/use_custom_origin")
        utm_origin_lat = rospy.get_param("/localization/utm_origin_lat")
        utm_origin_lon = rospy.get_param("/localization/utm_origin_lon")

        # Load the map using Lanelet2
        if coordinate_transformer == "utm":
            projector = UtmProjector(Origin(utm_origin_lat, utm_origin_lon), use_custom_origin, False)
        else:
            rospy.logfatal("lanelet2_global_planner - only utm and custom origin currently supported for lanelet2 map loading")
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
        self.tf = tf.TransformListener()

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
                wp.twist.twist.linear.x
            ) for wp in msg.waypoints])

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
        stop_lines = {}
        for result in msg.results:
            # check if we have already outputted the status of this stopline
            if result.lane_id in stop_lines:
                assert result.recognition_result == 0, "traffic light status callback - multiple stoplines with different status"
                continue

            # add stop line points to the list if the traffic light is red or yellow
            if result.recognition_result == 0:
                stop_line = self.lanelet2_map.lineStringLayer.get(result.lane_id)
                stop_lines[result.lane_id] = [point for point in stop_line]

        self.stop_lines = stop_lines


    def detected_objects_callback(self, msg):

        # get global path
        with self.lock:
            output_frame = self.output_frame
            global_path_array = self.global_path_array
            global_path_waypoints = self.global_path_waypoints
            global_path_tree = self.global_path_tree

        stop_lines = self.stop_lines
        current_pose = self.current_pose
        current_velocity = self.current_velocity

        # if global path or current pose is empty, publish empty local path, which stops the vehicle
        if global_path_array is None or current_pose is None:
            self.publish_local_path_wp([], msg.header.stamp, output_frame)
            return

        # extract local path points from global path
        wp_backward, wp_forward = get_two_nearest_waypoint_idx(global_path_tree, current_pose.x, current_pose.y)
        end_index = wp_backward + self.local_path_length
        if end_index > len(global_path_array):
            end_index = len(global_path_array)

        # project current_pose to path
        current_pose_on_path = get_closest_point_on_line(Point(x = current_pose.x, y = current_pose.y, z = current_pose.z),
                                                         Point(x = global_path_array[wp_backward,0], y = global_path_array[wp_backward,1], z = global_path_array[wp_backward,2]),
                                                         Point(x = global_path_array[wp_forward,0], y = global_path_array[wp_forward,1], z = global_path_array[wp_forward,2]))
        
        # create local_path_array and extract waypoints
        local_path_array = global_path_array[wp_forward : end_index,:]
        local_path_waypoints = copy.deepcopy(global_path_waypoints[wp_forward:end_index])

        # current_pose_on_path is behind or on top of the wp_forward, add it if distance is greater than 1cm
        if get_distance_between_two_points(current_pose_on_path, Point(x = local_path_array[0,0], y = local_path_array[0,1], z = local_path_array[0,2])) > 0.01:
            # add to local_path_array
            local_path_array = np.insert(local_path_array, 0, [current_pose_on_path.x, current_pose_on_path.y, current_pose_on_path.z, current_velocity], axis=0)
            # add to local_path_waypoints
            local_path_waypoints.insert(0, copy.deepcopy(local_path_waypoints[0]))
            local_path_waypoints[0].pose.pose.position = current_pose_on_path

        # calculate distances up to each waypoint
        local_path_dists = np.cumsum(np.sqrt(np.sum(np.diff(local_path_array[:,:2], axis=0)**2, axis=1)))
        local_path_dists = np.insert(local_path_dists, 0, 0.0)

        # initialize closest object distance and velocity
        closest_object_distance = 0.0 
        closest_object_velocity = 0.0
        blocked = False

        # react to objects on the local path
        points_list = []
        for obj in msg.objects:
            # project object velocity to base_link frame to get longitudinal speed
            velocity = Vector3Stamped(header=msg.header, vector=obj.velocity.linear)
            try:
                velocity = self.tf.transformVector3("base_link", velocity)
            except Exception as e:
                rospy.logerr(str(e))
                # safe option - assume the object is not moving
                velocity.vector.x = 0.0
            # add object center point and convex hull points to the points list
            points_list.append([obj.pose.position.x, obj.pose.position.y, obj.pose.position.z, velocity.vector.x])
            for point in obj.convex_hull.polygon.points:
                points_list.append([point.x, point.y, point.z, velocity.vector.x])

        # add stop line points to the points list
        for stop_line in stop_lines.values():
            for point in stop_line:
                points_list.append([point.x, point.y, point.z, 0.0])

        if len(points_list) > 0:
            # convert the points list to a numpy array
            obstacle_array = np.array(points_list)
            # create spatial index over obstacles for quick search
            obstacle_tree = NearestNeighbors(n_neighbors=1, algorithm=self.nearest_neighbor_search).fit(obstacle_array[:,:2])
            # find closest obstacle points to local path
            obstacle_dists, obstacle_idx = obstacle_tree.radius_neighbors(local_path_array[:,:2], self.car_safety_radius, return_distance=True)
            # calculate obstacle distances from the start of the local path
            obstacle_dists = [local_path_dists[i] + d for i, d in enumerate(obstacle_dists)]

            # calculate velocity based on distance to obstacle using deceleration limit
            for i, wp in enumerate(local_path_waypoints):

                # mark waypoints with obstacles for visualizer
                if len(obstacle_idx[i]) > 0:
                    wp.cost = 1.0

                # once we get zero speed, keep it that way
                if blocked:
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
                    lowest_target_velocity_idx = np.argmin(target_velocities)
                    target_vel = target_velocities[lowest_target_velocity_idx]
                    wp.twist.twist.linear.x = min(target_vel, wp.twist.twist.linear.x)

                    # from stop point onwards all speeds are zero
                    if math.isclose(wp.twist.twist.linear.x, 0.0):
                        blocked = True

                    # record the closest object from the first waypoint
                    if i == 0:
                        # closest object distance is calculated from the front of the car
                        closest_object_distance = obstacles_ahead_dists[lowest_target_velocity_idx] - self.current_pose_to_car_front
                        closest_object_velocity = obstacles_ahead_speeds[lowest_target_velocity_idx]

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
    rospy.init_node('speed_only_local_planner')
    node = SpeedOnlyLocalPlanner()
    node.run()