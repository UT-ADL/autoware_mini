#!/usr/bin/env python3

import rospy
import copy
import math
import threading
import tf

import numpy as np
from sklearn.neighbors import NearestNeighbors

from autoware_msgs.msg import Lane, DetectedObjectArray
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from std_msgs.msg import ColorRGBA

from helpers import get_two_nearest_waypoint_idx

GREEN = ColorRGBA(0.0, 1.0, 0.0, 0.4)
RED = ColorRGBA(1.0, 0.0, 0.0, 0.4)
YELLOW = ColorRGBA(0.8, 0.8, 0.0, 0.4)


class LocalPlanner:

    def __init__(self):

        # Parameters
        self.local_path_length = rospy.get_param("~local_path_length", 100)
        self.nearest_neighbor_search = rospy.get_param("~nearest_neighbor_search", "kd_tree")
        self.braking_safety_distance = rospy.get_param("~braking_safety_distance", 4.0)
        self.braking_reaction_time = rospy.get_param("~braking_reaction_time", 1.6)
        self.car_safety_radius = rospy.get_param("~car_safety_radius", 1.3)
        self.current_pose_to_car_front = rospy.get_param("~current_pose_to_car_front", 4.0)
        self.speed_deceleration_limit = rospy.get_param("~speed_deceleration_limit", 1.0)

        # Internal variables
        self.lock = threading.Lock()
        self.output_frame = None
        self.global_path_array = None
        self.global_path_waypoints = None
        self.global_path_tree = None
        self.current_velocity = 0.0
        self.tf = tf.TransformListener()

        # Publishers
        self.local_path_pub = rospy.Publisher('local_path', Lane, queue_size=1)

        # Subscribers
        rospy.Subscriber('smoothed_path', Lane, self.path_callback, queue_size=1)
        rospy.Subscriber('current_pose', PoseStamped, self.current_pose_callback, queue_size=1)
        rospy.Subscriber('current_velocity', TwistStamped, self.current_velocity_callback, queue_size=1)
        rospy.Subscriber('detected_objects', DetectedObjectArray, self.detected_objects_callback, queue_size=1)


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


    def detected_objects_callback(self, msg):

        # get global path
        with self.lock:
            output_frame = self.output_frame
            global_path_array = self.global_path_array
            global_path_waypoints = self.global_path_waypoints
            global_path_tree = self.global_path_tree

        # if global path is empty, publish empty local path, which stops the vehicle
        if global_path_array is None:
            self.publish_local_path_wp([], msg.header.stamp, output_frame)
            return

        # extract local path points from global path
        wp_backward, _ = get_two_nearest_waypoint_idx(global_path_tree, self.current_pose.x, self.current_pose.y)
        end_index = wp_backward + self.local_path_length
        if end_index > len(global_path_array):
            end_index = len(global_path_array)
        local_path_array = global_path_array[wp_backward : end_index,:]

        # slice waypoints from global path to local path
        local_path_waypoints = copy.deepcopy(global_path_waypoints[wp_backward:end_index])

        # initialize closest object distance and velocity
        closest_object_distance = 0.0 
        closest_object_velocity = 0.0
        blocked = False

        # react to objects on the local path
        if len(msg.objects) > 0:
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

            # convert the points list to a numpy array
            obstacle_array = np.array(points_list)
            # create spatial index over obstacles for quick search
            obstacle_tree = NearestNeighbors(n_neighbors=1, algorithm=self.nearest_neighbor_search).fit(obstacle_array[:,:3])
            # find closest obstacle points to local path
            obstacle_idx = obstacle_tree.radius_neighbors(local_path_array[:,:3], self.car_safety_radius, return_distance=False)

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
                obstacles_ahead_idx = np.unique(obstacles_ahead_idx)
                if len(obstacles_ahead_idx) > 0:
                    # get coordinates of obstacles ahead
                    obstacles_ahead = obstacle_array[obstacles_ahead_idx]
                    # get distances to those obstacles
                    obstacles_ahead_dists = np.linalg.norm(obstacles_ahead[:,:3] - local_path_array[i,:3], axis=1)
                    # get speeds of those obstacles
                    obstacles_ahead_speeds = obstacles_ahead[:,3]
                    # calculate stopping distances - following distance increased when obstacle has higher speed
                    stopping_distances = obstacles_ahead_dists - self.current_pose_to_car_front - self.braking_safety_distance \
                                            - self.braking_reaction_time * obstacles_ahead_speeds
                    # calculate target velocity based on stopping distance and deceleration limit
                    target_vel = np.min(np.sqrt(np.maximum(0, obstacles_ahead_speeds**2 + 2 * self.speed_deceleration_limit * stopping_distances)))
                    wp.twist.twist.linear.x = min(target_vel, wp.twist.twist.linear.x)
                    # from stop point onwards all speeds are zero
                    if math.isclose(wp.twist.twist.linear.x, 0.0):
                        blocked = True
                    # record the closest object from the first waypoint
                    if i == 0:
                        obstacles_ahead_dists = np.linalg.norm(obstacles_ahead[:,:3] - np.array([self.current_pose.x, self.current_pose.y, self.current_pose.z]), axis=1)
                        closest_object_idx = np.argmin(obstacles_ahead_dists)
                        # closest object distance is calculated from the front of the car
                        closest_object_distance = obstacles_ahead_dists[closest_object_idx] - self.current_pose_to_car_front
                        closest_object_velocity = obstacles_ahead_speeds[closest_object_idx]

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
    rospy.init_node('local_planner')
    node = LocalPlanner()
    node.run()