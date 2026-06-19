#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation &
# Copyright (c) 2023 Autonomous Driving Lab (ADL), University of Tartu.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

import math
from copy import deepcopy

import carla
import rospy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path

from tf.transformations import quaternion_from_euler
from std_srvs.srv import Empty

from srunner.autoagents.autonomous_agent import AutonomousAgent
from srunner.scenariomanager.timer import GameTime
from srunner.tools.route_manipulation import downsample_route

from autoware_mini.geometry import  get_point_using_heading_and_distance, get_heading_from_orientation
from autoware_mini.transform import get_distance_between_origins


def get_entry_point():
    return 'CarlaMinimalAgent'


class CarlaMinimalAgent(AutonomousAgent):
    '''
    A minimal ROS agent required by Route Scenario for publishing route waypoints and goal points
    '''

    def setup(self, path_to_conf_file):
        """
        setup agent
        """
        self.global_plan_published = False
              
        rospy.init_node('carla_minimal_agent')

        self.init_goal_delay = rospy.get_param("~init_goal_delay")
        self.downsampling_interval = rospy.get_param("~downsampling_interval")

        # Waypoints are used for Path visualisation in RVIZ
        self.waypoint_publisher = rospy.Publisher(
            '/carla/ego_vehicle/waypoints', Path, queue_size=1, tcp_nodelay=True, latch=True)
        self.goal_publisher = rospy.Publisher(
            '/move_base_simple/goal', PoseStamped, queue_size=0, tcp_nodelay=True, latch=True)

        # Position the ego at the route start via /initialpose. In vehicle-in-the-loop the
        # bicycle simulation consumes /initialpose and carla_pose_relay then holds the
        # CARLA ego there (a plain CARLA set_transform would be overwritten by the relay).
        self.initialpose_publisher = rospy.Publisher(
            '/initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True)
        self.initialpose_published = False

        self.cancel_route_service = rospy.ServiceProxy('/planning/cancel_route', Empty)
        self.goal_point_offset = get_distance_between_origins("ego_vehicle", "car_front")

        self.current_control = carla.VehicleControl()


    def publish_plan(self):
        """
        publish the global plan and goal points at once
        """
        # Position the ego at the route start (first waypoint) once, before the route.
        if not self.initialpose_published and self._global_plan_world_coord:
            start = self.pose_stamped_from_waypoint(self._global_plan_world_coord[0][0])
            ip = PoseWithCovarianceStamped()
            ip.header = start.header
            ip.pose.pose = start.pose
            self.initialpose_publisher.publish(ip)
            self.initialpose_published = True
            rospy.loginfo("%s - Published /initialpose at route start (%.2f, %.2f)",
                          rospy.get_name(), start.pose.position.x, start.pose.position.y)
            rospy.sleep(0.2)  # Sleep to ensure the initial pose is processed before publishing the route

        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        for wp in self._global_plan_world_coord:

            pose = self.pose_stamped_from_waypoint(wp[0])
            msg.poses.append(pose)

            # Publish goal point
            self.goal_publisher.publish(pose)

        if msg.poses:
            # Offset the final goal forward by the distance from vehicle center to car front,
            # because CARLA route waypoints are at vehicle center, but Autoware Mini goal is where car front stops
            final_pose = deepcopy(msg.poses[-1])
            heading = get_heading_from_orientation(final_pose.pose.orientation)
            final_pose.pose.position = get_point_using_heading_and_distance(final_pose.pose.position, heading, self.goal_point_offset)
            self.goal_publisher.publish(final_pose)

        self.waypoint_publisher.publish(msg)
        rospy.loginfo("%s - Publishing plan..", rospy.get_name())

    def __call__(self):
        """
        Execute the agent call, e.g. agent()
        Returns the next vehicle controls
        """
        control = self.run_step(self.sensor_interface.get_data(), GameTime.get_time())
        control.manual_gear_shift = False

        return control

    def set_global_plan(self, global_plan_gps, global_plan_world_coord):
        """
        Set the plan (route) for the agent.
        If both lists are empty, cancels the active route.
        """
        if not global_plan_world_coord:
            self._global_plan_world_coord = []
            self._global_plan = []
            self.global_plan_published = False
            try:
                self.cancel_route_service()
                rospy.loginfo("%s - Route cancelled via /planning/cancel_route", rospy.get_name())
            except rospy.ServiceException as e:
                rospy.logerr("%s - Failed to cancel route: %s", rospy.get_name(), e)
            return

        ds_ids = downsample_route(global_plan_world_coord, self.downsampling_interval)
        self._global_plan_world_coord = [(global_plan_world_coord[x][0], global_plan_world_coord[x][1])
                                         for x in ds_ids]
        self._global_plan = [global_plan_gps[x] for x in ds_ids]

    def run_step(self, input_data, timestamp):
        """
        Execute one step of navigation.
        """
        if timestamp > self.init_goal_delay and not self.global_plan_published:
            self.publish_plan()
            self.global_plan_published = True

        return self.current_control
    
    def destroy(self):
        """
        Cleanup of all ROS publishers
        """

        if self.waypoint_publisher:
            self.waypoint_publisher.unregister()
        if self.goal_publisher:
            self.goal_publisher.unregister()

        rospy.loginfo("Carla minimal agent no longer running")

    def pose_stamped_from_waypoint(self, waypoint):
        """
        Create a ROS PoseStamped from a CARLA waypoint
        """

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = waypoint.location.x
        pose.pose.position.y = -waypoint.location.y
        pose.pose.position.z = waypoint.location.z
        x, y, z, w = quaternion_from_euler(math.radians(waypoint.rotation.roll), math.radians(waypoint.rotation.pitch), -math.radians(waypoint.rotation.yaw)).tolist()
        pose.pose.orientation.x = x
        pose.pose.orientation.y = y
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w

        return pose