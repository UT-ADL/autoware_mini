#!/usr/bin/env python3

import rospy
import copy
import threading
import numpy as np
from sklearn.neighbors import NearestNeighbors
from autoware_msgs.msg import Lane, DetectedObjectArray
from geometry_msgs.msg import PoseStamped, TwistStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

from helpers import get_closest_point_on_line, interpolate_point_to_path, get_orientation_from_yaw, get_heading_between_two_points
from helpers.timer import Timer


CURRENT_POSE_TO_CAR_FRONT_METERS = 4.0  # taken from current_pose to radar transform
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
        self.car_safety_width = rospy.get_param("~car_safety_width", 1.3)
        self.speed_deceleration_limit = rospy.get_param("~speed_deceleration_limit", 1.0)
        self.max_speed_limit = rospy.get_param("~max_speed_limit", 40.0) / 3.6

        # Internal variables
        self.lock = threading.Lock()
        self.global_path_array = None
        self.global_path_waypoints = None
        self.global_path_tree = None
        self.current_velocity = 0.0

        self.closest_object_distance = 0.0
        self.closest_object_velocity = 0.0

        # Publishers
        self.local_path_pub = rospy.Publisher('local_path', Lane, queue_size=1)
        self.stop_point_visualization_pub = rospy.Publisher('stop_point_visualization', Marker, queue_size=1)

        # Subscribers
        self.path_sub = rospy.Subscriber('smoothed_path', Lane, self.path_callback, queue_size=1)
        self.current_pose_sub = rospy.Subscriber('current_pose', PoseStamped, self.current_pose_callback, queue_size=1)
        self.current_velocity_sub = rospy.Subscriber('current_velocity', TwistStamped, self.current_velocity_callback, queue_size=1)
        self.detect_objects_sub = rospy.Subscriber('detected_objects', DetectedObjectArray, self.detect_objects_callback, queue_size=1)


    def path_callback(self, msg):

        self.output_frame = msg.header.frame_id
        
        if len(msg.waypoints) == 0:
            self.lock.acquire()
            self.global_path_array = None
            self.global_path_waypoints = None
            self.global_path_tree = None
            self.lock.release()

            self.publish_local_path_wp([])

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
        global_path_tree = NearestNeighbors(n_neighbors=1, algorithm=self.nearest_neighbor_search).fit(global_path_array[:,0:2])

        self.lock.acquire()
        self.global_path_array = global_path_array
        self.global_path_waypoints = global_path_waypoints
        self.global_path_tree = global_path_tree
        self.lock.release()

    def current_velocity_callback(self, msg):
        self.current_velocity = msg.twist.linear.x


    def current_pose_callback(self, msg):

        # get current pose
        self.current_pose = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])


    def detect_objects_callback(self, msg):

        # internal variables
        obstacles_on_local_path = False
        stop_point = None
        stop_quaternion = None
        obstacle_tree = None
        wp_with_obs = []                    # indexes of local wp where obstacles are found witihn search radius
        stop_color = GREEN                       # use for stop_point_viz 

        if self.global_path_array is None:
            return

        # get global path
        self.lock.acquire()
        global_path_array = self.global_path_array
        global_path_waypoints = self.global_path_waypoints
        global_path_tree = self.global_path_tree
        self.lock.release()

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
            obstacle_tree = NearestNeighbors(n_neighbors=1, algorithm=self.nearest_neighbor_search).fit(obstacle_array[:,0:3])

        # KEEP TRACK OF LOCAL PATH
        wp_backward, wp_forward = self.find_two_nearest_waypoint_idx(global_path_tree, self.current_pose[0], self.current_pose[1])
        nearest_point = get_closest_point_on_line(Point(self.current_pose[0], self.current_pose[1], self.current_pose[2]),
                                                  global_path_waypoints[wp_backward].pose.pose.position, 
                                                  global_path_waypoints[wp_forward].pose.pose.position)

        end_index = wp_forward + self.local_path_length
        if end_index > len(global_path_array):
            end_index = len(global_path_array)

        local_path_array = global_path_array[wp_forward : end_index,:]
        nearest_point_array = np.array([(nearest_point.x, nearest_point.y, nearest_point.z, 0.0)])

        # add nearest point as first point in local_path_array
        local_path_array = np.concatenate((nearest_point_array, local_path_array), axis=0)

        # calc local path distances and insert 0 in front for 1st waypoint
        local_path_distances = np.cumsum(np.sqrt(np.diff(local_path_array[:,0])**2 + np.diff(local_path_array[:,1])**2))
        local_path_distances = np.insert(local_path_distances, 0, 0.0)
        # path end
        self.closest_object_distance = local_path_distances[-1]
        self.closest_object_velocity = 0.0

        # OBJECT DETECTION
        # iterate over local path waypoints and check if there are objects within car_safety_width
        if obstacle_tree is not None:
            # ask closest obstacle points to local_path_array, except 1st (current_pose on path) to be in sync later with sliced waypoint idx's
            obstacle_d, obstacle_idx = obstacle_tree.radius_neighbors(local_path_array[1:,0:3], self.car_safety_width, return_distance=True)

            # get list of indexes of local_path waypoints where obstacles are found
            wp_with_obs = [i for i in range(len(obstacle_d)) if len(obstacle_d[i]) > 0]

            if len(wp_with_obs) > 0:
                obstacles_on_local_path = True

                # add corresponding "along path distances" to obstacle_d
                obstacle_d = [np.add(obstacle_d[i], local_path_distances[i+1]) for i in range(len(obstacle_d))]
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
                self.closest_object_distance = obs_on_path[0,0] - CURRENT_POSE_TO_CAR_FRONT_METERS - self.braking_safety_distance
                self.closest_object_velocity = obs_on_path[3,0]

                # find index of point when the value in array exeeds the distance to the nearest obstacle
                idx_after_obj = np.where(local_path_distances > obs_on_path[0,0])[0][0]
                stop_point = interpolate_point_to_path(obstacle_array[obs_on_path[1,0].astype(int), 0:3],        # obstacle point coordinates
                                                        local_path_array[idx_after_obj-1,0:3],                   # point on path before
                                                        local_path_array[idx_after_obj,0:3])                     # point on path after
                stop_heading = get_heading_between_two_points(Point(local_path_array[idx_after_obj-1,0], local_path_array[idx_after_obj-1,1], local_path_array[idx_after_obj-1,2]),
                                                        Point(local_path_array[idx_after_obj,0], local_path_array[idx_after_obj,1], local_path_array[idx_after_obj,2])) - np.pi/2
                stop_quaternion = get_orientation_from_yaw(stop_heading)

        # LOCAL PATH WAYPOINTS
        # slice waypoints from global path to local path - should be replaced with
        local_path_waypoints = copy.deepcopy(global_path_waypoints[wp_forward:end_index])

        # Calculate velocity profile in local path waypoints
        if obstacles_on_local_path:
            # calculate velocity based on distance to obstacle using deceleration limit
            for i in range(len(local_path_waypoints)):
                # adjust distance based on car speed - following distance increased when obstacle has higher speed
                object_distance_at_i = self.closest_object_distance - self.braking_reaction_time * self.closest_object_velocity - local_path_distances[i]
                target_vel = np.sqrt(max(0, self.closest_object_velocity**2 + 2 * self.speed_deceleration_limit * object_distance_at_i))

                if i == 0:
                    # if obstacle causes to go slower than map speed in current waypoint then we are braking - yellow line
                    # and if obstacle is close to 0 speed then red line - braking to stop
                    if target_vel < local_path_waypoints[i].twist.twist.linear.x:
                        stop_color = YELLOW
                        if self.closest_object_velocity < 0.5:
                            stop_color = RED

                local_path_waypoints[i].twist.twist.linear.x = min(target_vel, local_path_waypoints[i].twist.twist.linear.x)

        # set cost to 1.0 if there is an obstacle
        for i, wp in enumerate(local_path_waypoints):
            if i in wp_with_obs:
                wp.cost = 1.0

        self.visualize_stop_point(stop_point, stop_quaternion, stop_color)
        self.publish_local_path_wp(local_path_waypoints)


    def visualize_stop_point(self, stop_point, stop_quaternion, color):

        marker = Marker()
        marker.header.frame_id = self.output_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "stop_point"
        marker.id = 0

        if stop_point == None:
            # create marker_array to delete all visualization markers
            marker.action = Marker.DELETEALL
            self.stop_point_visualization_pub.publish(marker)
            return

        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.pose.position = stop_point
        marker.pose.orientation = stop_quaternion
        marker.scale.x = 5.0
        marker.scale.y = 0.2
        marker.scale.z = 3.0
        marker.color = color
        self.stop_point_visualization_pub.publish(marker)


    def publish_local_path_wp(self, local_path_waypoints):
        # create lane message
        lane = Lane()
        lane.header.frame_id = self.output_frame
        lane.header.stamp = rospy.Time.now()
        lane.waypoints = local_path_waypoints
        lane.closest_object_distance = self.closest_object_distance
        lane.closest_object_velocity = self.closest_object_velocity
        
        self.local_path_pub.publish(lane)

    def find_two_nearest_waypoint_idx(self, waypoint_tree, x, y):
        idx = waypoint_tree.kneighbors([(x, y)], 2, return_distance=False)
        # sort to get them in ascending order - follow along path
        idx[0].sort()
        return idx[0][0], idx[0][1]

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('local_planner')
    node = LocalPlanner()
    node.run()