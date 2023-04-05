#!/usr/bin/env python

import rospy
import threading
import numpy as np
from sklearn.neighbors import NearestNeighbors
from autoware_msgs.msg import Lane, Waypoint, DetectedObjectArray
from geometry_msgs.msg import PoseStamped, TwistStamped, Point

from helpers import get_closest_point_on_line
from helpers.timer import Timer

class LocalPlanner:

    def __init__(self):

        # Parameters
        self.local_path_length = rospy.get_param("~local_path_length", 100)
        self.output_frame = rospy.get_param("~output_frame", "map")
        self.nearest_neighbor_search = rospy.get_param("~nearest_neighbor_search", "kd_tree")
        self.safety_distance = rospy.get_param("~safety_distance", 4.0)
        self.car_safety_width = rospy.get_param("~car_safety_width", 2.0)

        # Internal variables
        self.lock = threading.Lock()
        self.global_path_array = None
        self.global_path_last_idx = None
        self.global_path_waypoints = None
        self.global_path_tree = None
        self.current_velocity = 0.0
        self.local_path_start_global_idx = 0

        self.closest_object_distance = 0.0
        self.closest_object_velocity = 0.0

        # Publishers
        self.local_path_pub = rospy.Publisher('local_path', Lane, queue_size=1)

        # Subscribers
        self.path_sub = rospy.Subscriber('smoothed_path', Lane, self.path_callback, queue_size=1)
        self.current_pose_sub = rospy.Subscriber('current_pose', PoseStamped, self.current_pose_callback, queue_size=1)
        self.current_velocity_sub = rospy.Subscriber('current_velocity', TwistStamped, self.current_velocity_callback, queue_size=1)
        self.detect_objects_sub = rospy.Subscriber('detected_objects', DetectedObjectArray, self.detect_objects_callback, queue_size=1)


    def path_callback(self, msg):
        
        if len(msg.waypoints) == 0:
            self.lock.acquire()
            self.global_path_array = None
            self.global_path_waypoints = None
            self.global_path_last_idx = None
            self.global_path_tree = None
            self.local_path_start_global_idx = 0
            self.lock.release()
            return

        # extract all waypoint attributes with one loop
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
        self.global_path_last_idx = len(global_path_array) - 1
        self.local_path_start_global_idx = 0
        self.lock.release()

        # when global path is received, local path will be recalculated
        # publish local path - send empty np.array
        # self.publish_local_path(np.array([]))

    def current_velocity_callback(self, msg):
        self.current_velocity = msg.twist.linear.x


    def current_pose_callback(self, msg):

        # get current pose
        self.current_pose = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])


    def detect_objects_callback(self, msg):

        t = Timer()

        if self.global_path_array is None:
            return

        # get global path
        self.lock.acquire()
        global_path_array = self.global_path_array
        global_path_waypoints = self.global_path_waypoints
        global_path_tree = self.global_path_tree
        global_path_last_idx = self.global_path_last_idx
        self.lock.release()
        
        obstacle_tree = None

        if len(msg.objects) > 0:
            obstacle_array = np.array([(
                    obj.pose.position.x,
                    obj.pose.position.y,
                    obj.pose.position.z,
                    obj.velocity.linear.x
                ) for obj in msg.objects])
            obstacle_tree = NearestNeighbors(n_neighbors=1, algorithm=self.nearest_neighbor_search).fit(obstacle_array[:,0:3])

        # find local path start index
        wp_backward, wp_forward = self.find_two_nearest_waypoint_idx(global_path_tree, self.current_pose[0], self.current_pose[1])

        # TODO: take from array - optimize here!!! Do not create Point object
        nearest_point = get_closest_point_on_line(Point(self.current_pose[0], self.current_pose[1], self.current_pose[2]),
                                                  global_path_waypoints[wp_backward].pose.pose.position, 
                                                  global_path_waypoints[wp_forward].pose.pose.position)

        end_index = wp_forward + self.local_path_length
        if end_index > global_path_last_idx:
            end_index = global_path_last_idx + 1

        local_path_array = global_path_array[wp_forward : end_index,:]
        # TODO optimize! make array out of nearest point and insert it in front of local_path_array
        nearest_point_array = np.array([(nearest_point.x, nearest_point.y, nearest_point.z, 0.0)])
        # add nearest point as first point in local path
        local_path_array = np.concatenate((nearest_point_array, local_path_array), axis=0)

        # calc local path distances and set closest object distance to the last point in local path
        local_path_distances = np.cumsum(np.sqrt(np.diff(local_path_array[:,0])**2 + np.diff(local_path_array[:,1])**2))
        local_path_distances = np.insert(local_path_distances, 0, 0.0)
        self.closest_object_distance = local_path_distances[-1]
        self.closest_object_velocity = 0.0

        t('distances')

        # TODO: add object avoidance
        # iterate over local path and check if there is an object in front of the car
        if obstacle_tree is not None:
            for i in range(len(local_path_array)):
                d, idx = obstacle_tree.kneighbors([(local_path_array[i,0], local_path_array[i,1], local_path_array[i,2])], 1)
                if d[0][0] < self.car_safety_width:
                    # obstacle is within width of the car from path
                    # print(i, "idx: ", idx, "d: ", d)
                    # might be close to many points (2m), need to know closest or 2 closest / project onto path and calc distance
                    # create additional marker for closest point and publish it
                    if local_path_distances[i] + d[0][0]  < self.closest_object_distance:
                        # TODO need to stop before the OBS - car length + buffer zone
                        self.closest_object_distance = local_path_distances[i] + d[0][0] - 4.0 - self.safety_distance
                        if self.closest_object_distance < 0.0:
                            self.closest_object_distance = 0.0
                        self.closest_object_velocity = obstacle_array[idx[0][0],3]

        t('obstacles')

        # slice waypoints from global path to local path
        local_path_waypoints = global_path_waypoints[wp_forward:end_index]
        self.publish_local_path_wp(local_path_waypoints)

        t('publish')

        print(t)


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