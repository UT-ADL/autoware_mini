#!/usr/bin/env python3

import rospy
import copy
import threading
import numpy as np
from sklearn.neighbors import NearestNeighbors
from autoware_msgs.msg import Lane, DetectedObjectArray
from geometry_msgs.msg import PoseStamped, TwistStamped, Point
from std_msgs.msg import ColorRGBA

from helpers import get_closest_point_on_line, get_two_nearest_waypoint_idx


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

        # Publishers
        self.local_path_pub = rospy.Publisher('local_path', Lane, queue_size=1)

        # Subscribers
        rospy.Subscriber('smoothed_path', Lane, self.path_callback, queue_size=1)
        rospy.Subscriber('current_pose', PoseStamped, self.current_pose_callback, queue_size=1)
        rospy.Subscriber('current_velocity', TwistStamped, self.current_velocity_callback, queue_size=1)
        rospy.Subscriber('detected_objects', DetectedObjectArray, self.detect_objects_callback, queue_size=1)


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


    def detect_objects_callback(self, msg):

        # get global path
        with self.lock:
            output_frame = self.output_frame
            global_path_array = self.global_path_array
            global_path_waypoints = self.global_path_waypoints
            global_path_tree = self.global_path_tree

        if global_path_array is None:
            self.publish_local_path_wp([], msg.header.stamp, output_frame)
            return

        # internal variables
        obstacles_on_local_path = False
        obstacle_tree = None
        wp_with_obs = []                    # indexes of local wp where obstacles are found witihn search radius

        # CREATE OBSTACLE ARRAY AND TREE
        if len(msg.objects) > 0:
            points_list = []
            # add object center point and convex hull points to points list
            for i, obj in enumerate(msg.objects):
                points_list.append([obj.pose.position.x, obj.pose.position.y, obj.pose.position.z, obj.velocity.linear.x, i])
                for point in obj.convex_hull.polygon.points:
                    points_list.append([point.x, point.y, point.z, obj.velocity.linear.x, i])

            # Convert the points list to a numpy array
            obstacle_array = np.array(points_list)
            obstacle_tree = NearestNeighbors(n_neighbors=1, algorithm=self.nearest_neighbor_search).fit(obstacle_array[:,:3])

        # KEEP TRACK OF LOCAL PATH
        wp_backward, wp_forward = get_two_nearest_waypoint_idx(global_path_tree, self.current_pose.x, self.current_pose.y)
        nearest_point = get_closest_point_on_line(self.current_pose,
                                                  global_path_waypoints[wp_backward].pose.pose.position, 
                                                  global_path_waypoints[wp_forward].pose.pose.position)

        end_index = wp_backward + self.local_path_length
        if end_index > len(global_path_array):
            end_index = len(global_path_array)

        # extract local path points from global path
        local_path_array = global_path_array[wp_backward : end_index,:]

        # replace the first point with nearest point in local_path_array
        local_path_array[0, :] = (nearest_point.x, nearest_point.y, nearest_point.z, 0.0)

        # calc local path distances and insert 0 in front for 1st waypoint
        local_path_distances = np.cumsum(np.sqrt(np.sum(np.diff(local_path_array[:,:2], axis=0)**2, axis=1)))
        local_path_distances = np.insert(local_path_distances, 0, 0.0)

        # path end
        closest_object_distance = local_path_distances[-1]
        closest_object_velocity = 0.0

        # OBJECT DETECTION
        # iterate over local path waypoints and check if there are objects within car_safety_radius
        if obstacle_tree is not None:
            # ask closest obstacle points to local_path_array, except 1st (local_path_discurrent_pose on path) to be in sync later with sliced waypoint idx's
            obstacle_d, obstacle_idx = obstacle_tree.radius_neighbors(local_path_array[:,:3], self.car_safety_radius, return_distance=True)

            # get list of indexes of local_path waypoints where obstacles are found
            wp_with_obs = [i for i, d in enumerate(obstacle_d) if len(d) > 0]

            if len(wp_with_obs) > 0:
                obstacles_on_local_path = True

                # add corresponding "along path distances" to obstacle_d
                obstacle_d = [np.array(d_list) + local_path_distances[i] for i, d_list in enumerate(obstacle_d)]
                # flatten obstacle_d and obstacle_idx and then and stack
                obs_on_path = np.vstack((np.hstack(obstacle_d), np.hstack(obstacle_idx)))
                # by matching index update array by adding 3rd row that contains object id taken from obstacle_array
                obs_on_path = np.vstack((obs_on_path, obstacle_array[obs_on_path[1].astype(int), 4]))
                # sort by distance and keep only closest point from each object
                obs_on_path = obs_on_path[:, obs_on_path[0].argsort()]
                obs_on_path = obs_on_path[:, np.unique(obs_on_path[2], return_index=True)[1]]
                # adding velocity, array will contain: distance, obstacle_array_id, obstacle_id, velocity
                obs_on_path = np.vstack((obs_on_path, obstacle_array[obs_on_path[1].astype(int), 3]))

                # calculate closest object in terms of distance, velocity and fixed deceleration - sqrt(v**2 + 2 * a * d)
                obs_on_path = np.vstack((obs_on_path, np.sqrt((obs_on_path[3]**2 + 2 * self.speed_deceleration_limit * obs_on_path[0]))))
                obs_on_path = obs_on_path[:, obs_on_path[4].argsort()]
                closest_object_distance = obs_on_path[0,0]
                closest_object_velocity = obs_on_path[3,0]

        # LOCAL PATH WAYPOINTS
        # slice waypoints from global path to local path
        local_path_waypoints = copy.deepcopy(global_path_waypoints[wp_backward:end_index])

        # Calculate velocity profile in local path waypoints
        if obstacles_on_local_path:
            # calculate velocity based on distance to obstacle using deceleration limit
            for i in range(len(local_path_waypoints)):
                # adjust distance based on car speed - following distance increased when obstacle has higher speed
                object_distance_at_i = closest_object_distance - self.current_pose_to_car_front - self.braking_safety_distance \
                                        - self.braking_reaction_time * closest_object_velocity - local_path_distances[i]
                target_vel = np.sqrt(max(0, closest_object_velocity**2 + 2 * self.speed_deceleration_limit * object_distance_at_i))
                local_path_waypoints[i].twist.twist.linear.x = min(target_vel, local_path_waypoints[i].twist.twist.linear.x)

        # set cost to 1.0 if there is an obstacle
        for i, wp in enumerate(local_path_waypoints):
            if i in wp_with_obs:
                wp.cost = 1.0

        self.publish_local_path_wp(local_path_waypoints, msg.header.stamp, output_frame, closest_object_distance, closest_object_velocity)

    def publish_local_path_wp(self, local_path_waypoints, stamp, output_frame, closest_object_distance=0, closest_object_velocity=0):
        # create lane message
        lane = Lane()
        lane.header.frame_id = output_frame
        lane.header.stamp = stamp
        lane.waypoints = local_path_waypoints
        lane.closest_object_distance = closest_object_distance
        lane.closest_object_velocity = closest_object_velocity
        
        self.local_path_pub.publish(lane)


    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('local_planner')
    node = LocalPlanner()
    node.run()