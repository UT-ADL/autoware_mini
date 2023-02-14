#!/usr/bin/env python

import rospy
import message_filters
import numpy as np
from sklearn.neighbors import KDTree

from autoware_msgs.msg import Lane, VehicleCmd
from geometry_msgs.msg import PoseStamped,TwistStamped


class WaypointFollower:
    def __init__(self):

        # Parameters
        self.waypoint_topic = rospy.get_param("~waypoint_topic", "waypoints")
        self.follower = rospy.get_param("~follower", "pure_pursuit")
        self.planning_time = rospy.get_param("~planning_time", 2.0)
        self.wheel_base = rospy.get_param("~wheel_base", 2.789)
        self.min_lookahead_distance = rospy.get_param("~min_lookahead_distance", 5.5)

        # Variables - init
        self.current_velocity = 0.0
        self.waypoint_tree = None
        self.waypoints = None
        self.target_velocity = 0.0

        # Subscribers
        self.waypoints_sub = rospy.Subscriber('/waypoints', Lane, self.waypoints_callback)
        self.current_pose_sub = message_filters.Subscriber('/current_pose', PoseStamped)
        self.current_velocity_sub = message_filters.Subscriber('/current_velocity', TwistStamped)
        # TODO if no sleep then current_status_callback is called too early (kdtree and waypoints are not ready)
        rospy.sleep(1.0)
        ts = message_filters.TimeSynchronizer([self.current_pose_sub, self.current_velocity_sub], queue_size=10)
        ts.registerCallback(self.current_status_callback)

        # Publishers
        self.vehicle_command_pub = rospy.Publisher('vehicle_cmd', VehicleCmd, queue_size=10)

        # output information to console
        rospy.loginfo("waypoint_follower - follower: %s" % self.follower)
        rospy.loginfo("waypoint_follower - initiliazed")


    def waypoints_callback(self, waypoints_msg):
        self.waypoints = waypoints_msg.waypoints

        # create kd-tree for nearest neighbor search
        waypoints_xy = np.array([[w.pose.pose.position.x, w.pose.pose.position.y] for w in self.waypoints])
        self.waypoint_tree = KDTree(waypoints_xy)

        if self.follower == "pure_pursuit":
            import PurePursuitFollower
            self.follow_path = PurePursuitFollower.PurePursuitFollower(self.waypoints, self.waypoint_tree)
        elif self.follower == "stanley":
            import StanleyFollower
            self.follow_path = StanleyFollower.StanleyFollower(self.waypoints, self.waypoint_tree)


    def current_status_callback(self, current_pose_msg, current_velocity_msg):

        self.current_pose = current_pose_msg.pose
        self.current_velocity = current_velocity_msg.twist.linear.x

        # get nearest waypoint distance and idx - make sure waypoints are loaded
        # or create / init this whole callback after waypoints are loaded - to skip the check here! 
        if self.waypoint_tree is not None:
            
            self.steering_angle, self.target_velocity = self.follow_path.calc_steering_and_velocity(self.current_pose, self.current_velocity)

            self.publish_vehicle_command(self.target_velocity, self.steering_angle)


    def publish_vehicle_command(self, velocity, steering_angle):
        vehicle_cmd = VehicleCmd()
        vehicle_cmd.header.stamp = rospy.Time.now()
        vehicle_cmd.header.frame_id = "/map"
        vehicle_cmd.ctrl_cmd.linear_velocity = velocity
        vehicle_cmd.ctrl_cmd.steering_angle = steering_angle
        self.vehicle_command_pub.publish(vehicle_cmd)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('waypoint_follower', log_level=rospy.INFO)
    node = WaypointFollower()
    node.run()
