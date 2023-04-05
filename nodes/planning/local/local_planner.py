#!/usr/bin/env python

import rospy
import threading
import numpy as np
from autoware_msgs.msg import Lane, Waypoint, DetectedObjectArray
from geometry_msgs.msg import PoseStamped, TwistStamped


class LocalPlanner:

    def __init__(self):

        # Parameters
        self.local_path_length = rospy.get_param("~local_path_length", 100)
        self.output_frame = rospy.get_param("~output_frame", "map")

        # Internal variables
        self.lock = threading.Lock()
        self.global_path_array = None
        self.global_path_last_idx = None
        self.global_path_waypoints = None
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
        
        self.lock.acquire()
        self.global_path_array = global_path_array
        self.global_path_waypoints = global_path_waypoints
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
        self.objects = msg.objects

        if self.global_path_array is None:
            return

        # get global path
        self.lock.acquire()
        global_path_array = self.global_path_array
        global_path_waypoints = self.global_path_waypoints
        global_path_last_idx = self.global_path_last_idx
        local_path_start_global_idx = self.local_path_start_global_idx
        self.lock.release()
        
        # use while loop to calculate distance between current pose and points in global path
        # continue until the distance is decreasing
        
        # initial distance
        distance = 10000.0
        end_index = 0

        for i in range(local_path_start_global_idx, global_path_last_idx + 1):
            d = np.sqrt(np.sum((global_path_array[i,0:2] - self.current_pose[0:2])**2))
            if d < distance:
                distance = d
            else:
                local_path_start_global_idx = i - 1
                end_index = local_path_start_global_idx + self.local_path_length
                if end_index > global_path_last_idx:
                    end_index = global_path_last_idx + 1
                # print("start: ", self.local_path_start_global_idx, "end: ", end_index, "last: ", self.global_path_last_idx, "len: ", len(global_path_array))
                break

        # slice local path from global path
        local_path_array = global_path_array[local_path_start_global_idx:end_index,:]

        # TODO: add object avoidance
        # use local_path_array



        # TODO just to test - approach goal
        distance_to_path_end = np.cumsum(np.sqrt(np.diff(local_path_array[:,0])**2 + np.diff(local_path_array[:,1])**2))[-1]
        self.closest_object_distance = distance_to_path_end
        self.closest_object_velocity = 0.0
        
        # slice waypoints from global path to local path
        local_path_waypoints = global_path_waypoints[local_path_start_global_idx:end_index]
        self.publish_local_path_wp(local_path_waypoints)


        # update local path start index
        self.lock.acquire()
        self.local_path_start_global_idx = local_path_start_global_idx
        self.lock.release()


    def publish_local_path_wp(self, local_path_waypoints):
        # create lane message
        lane = Lane()
        lane.header.frame_id = self.output_frame
        lane.header.stamp = rospy.Time.now()
        lane.waypoints = local_path_waypoints
        lane.closest_object_distance = self.closest_object_distance
        lane.closest_object_velocity = self.closest_object_velocity
        
        self.local_path_pub.publish(lane)


    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('local_planner')
    node = LocalPlanner()
    node.run()