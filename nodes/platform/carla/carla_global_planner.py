#!/usr/bin/env python3
#
# Copyright (c) 2023 Autonomous Driving Lab (ADL), University of Tartu.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
receive a path from carla_ros_waypoint_publisher and convert it to autoware
"""
import rospy

from autoware_mini.msg import Path, Waypoint
from nav_msgs.msg import Path as NavPath
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_srvs.srv import Empty, EmptyResponse

from autoware_mini.geometry import get_distance_between_two_points_2d


class CarlaGlobalPlanner():

    def __init__(self):

        # Node parameters
        self.output_frame = rospy.get_param("output_frame")
        self.distance_to_goal_limit = rospy.get_param("distance_to_goal_limit")
        self.speed_limit = rospy.get_param("speed_limit")
        self.ego_vehicle_stopped_speed_limit = rospy.get_param("ego_vehicle_stopped_speed_limit")

        # Local variables
        self.goal_point = None
        self.current_speed = None
        
        # Publishers
        self.waypoints_pub = rospy.Publisher('lane_change_global_path', Path, queue_size=10, latch=True, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/localization/current_velocity', TwistStamped, self.current_velocity_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/carla/ego_vehicle/waypoints', NavPath, self.path_callback, queue_size=None, tcp_nodelay=True)

        # Services
        rospy.Service('cancel_route', Empty, self.cancel_route_callback)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def path_callback(self, data):
        """
        Callback for path. Convert it to Autoware Path and publish it
        """
        msg = Path()
        msg.header.frame_id = self.output_frame
        msg.header.stamp = data.header.stamp

        waypoints = []
        last_pose = data.poses[-1].pose
        self.goal_point = last_pose.position
        for pose in data.poses:
            waypoint = Waypoint(position=pose.pose.position, speed=self.speed_limit / 3.6)
            waypoints.append(waypoint)

        msg.waypoints = waypoints

        self.waypoints_pub.publish(msg)

    def current_pose_callback(self, msg):
        """
        Clears the global path when the vehicle has reached the goal point
        """
        if self.goal_point is not None and self.current_speed is not None:
            d = get_distance_between_two_points_2d(msg.pose.position, self.goal_point)
            if d < self.distance_to_goal_limit and self.current_speed < self.ego_vehicle_stopped_speed_limit:
                self.goal_point = None

                path = Path()        
                path.header.frame_id = self.output_frame
                path.header.stamp = rospy.Time.now()
                path.waypoints = []

                self.waypoints_pub.publish(path)
                rospy.loginfo("%s - goal reached, clearing path!", rospy.get_name())

    def current_velocity_callback(self, msg):
        self.current_speed = msg.twist.linear.x

    def cancel_route_callback(self, msg):
        self.goal_point = None

        path = Path()
        path.header.frame_id = self.output_frame
        path.header.stamp = rospy.Time.now()
        path.waypoints = []

        self.waypoints_pub.publish(path)
        rospy.loginfo("%s - route cancelled!", rospy.get_name())
        return EmptyResponse()

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('carla_global_planner', log_level=rospy.INFO)
    node = CarlaGlobalPlanner()
    node.run()
