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
import shapely
from autoware_mini.msg import Path, Waypoint
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_srvs.srv import Empty, EmptyResponse

from localization.SimulationToUTMTransformer import SimulationToUTMTransformer
from localization.UTMToSimulationTransformer import UTMToSimulationTransformer


class CarlaWaypointsPublisher():

    def __init__(self):

        # Node parameters
        self.output_frame = rospy.get_param("output_frame")
        self.distance_to_goal_limit = rospy.get_param("distance_to_goal_limit")
        self.speed_limit = rospy.get_param("speed_limit")
        self.ego_vehicle_stopped_speed_limit = rospy.get_param("ego_vehicle_stopped_speed_limit")
        use_custom_origin = rospy.get_param("/localization/use_custom_origin")
        utm_origin_lat = rospy.get_param("/localization/utm_origin_lat")
        utm_origin_lon = rospy.get_param("/localization/utm_origin_lon")

        # Internal parameters
        self.sim2utm_transformer = SimulationToUTMTransformer(use_custom_origin=use_custom_origin,
                                                              origin_lat=utm_origin_lat,
                                                              origin_lon=utm_origin_lon)
        self.utm2sim_transformer = UTMToSimulationTransformer(use_custom_origin=use_custom_origin,
                                                              origin_lat=utm_origin_lat,
                                                              origin_lon=utm_origin_lon)
        self.goal_point = None
        self.current_speed = None
        
        # Publishers
        self.waypoints_pub = rospy.Publisher('lane_change_global_path', Path, queue_size=10, latch=True, tcp_nodelay=True)
        self.goal_publisher = rospy.Publisher('/carla/ego_vehicle/goal', PoseStamped, queue_size=10, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/localization/current_velocity', TwistStamped, self.current_velocity_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/carla/ego_vehicle/waypoints', Path, self.path_callback, queue_size=None, tcp_nodelay=True)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback, queue_size=None, tcp_nodelay=True)

        # Services
        rospy.Service('cancel_route', Empty, self.cancel_route_callback)

    def path_callback(self, data):
        """
        Callback for path. Convert it to Autoware LaneArray and publish it
        """
        msg = Path()
        msg.header = data.header

        waypoints = []
        last_pose = self.sim2utm_transformer.transform_pose(data.poses[-1].pose)
        self.goal_point = shapely.Point(last_pose.position.x, last_pose.position.y, last_pose.position.z)
        for pose in data.poses:
            pose.pose = self.sim2utm_transformer.transform_pose(pose.pose)
            waypoint = Waypoint(pose=pose)
            waypoint.speed = self.speed_limit / 3.6
            waypoints.append(waypoint)

        msg.waypoints = waypoints

        self.waypoints_pub.publish(msg)

    def goal_callback(self, msg):
        """
        Converts goal point simulation coordinates to UTM coordinates
        """
        self.goal_point = shapely.Point(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

        goal_msg = PoseStamped()
        goal_msg.header = msg.header
        goal_msg.pose = self.utm2sim_transformer.transform_pose(msg.pose)

        self.goal_publisher.publish(goal_msg)

    def current_pose_callback(self, msg):
        """
        Clears the global path when the vehicle has reached the goal point
        """
        self.current_location = shapely.Point(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

        if self.goal_point is not None:
            d = shapely.distance(self.current_location, self.goal_point)
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
    rospy.init_node('carla_waypoints_publisher', log_level=rospy.INFO)
    node = CarlaWaypointsPublisher()
    node.run()
