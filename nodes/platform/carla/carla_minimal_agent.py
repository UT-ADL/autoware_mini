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

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from localization.SimulationToUTMTransformer import SimulationToUTMTransformer

from tf.transformations import quaternion_from_euler

from srunner.autoagents.autonomous_agent import AutonomousAgent
from srunner.scenariomanager.timer import GameTime
from srunner.tools.route_manipulation import downsample_route

from helpers.geometry import  get_point_using_heading_and_distance, get_heading_from_orientation


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

        # Sim2UTM transformer used for Tartu map
        self.use_transformer = rospy.get_param("/carla_localization/use_transformer")
        use_custom_origin = rospy.get_param("/localization/use_custom_origin")
        utm_origin_lat = rospy.get_param("/localization/utm_origin_lat")
        utm_origin_lon = rospy.get_param("/localization/utm_origin_lon")

        self.sim2utm_transformer = SimulationToUTMTransformer(use_custom_origin=use_custom_origin,
                                                        origin_lat=utm_origin_lat,
                                                        origin_lon=utm_origin_lon)
        # Waypoints are used for Path visualisation in RVIZ
        self.waypoint_publisher = rospy.Publisher(
            '/carla/ego_vehicle/waypoints', Path, queue_size=1, tcp_nodelay=True, latch=True)
        self.goal_publisher = rospy.Publisher(
            '/move_base_simple/goal', PoseStamped, queue_size=0, tcp_nodelay=True, latch=True)
        
        self.current_control = carla.VehicleControl()


    def publish_plan(self):
        """
        publish the global plan and goal points at once
        """
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        for wp in self._global_plan_world_coord:

            pose = self.pose_stamped_from_waypoint(wp[0])
            msg.poses.append(pose)

            # Publish goal point
            self.goal_publisher.publish(pose)

        final_pose = deepcopy(msg.poses[-1])
        heading = get_heading_from_orientation(final_pose.pose.orientation)
        final_pose.pose.position = get_point_using_heading_and_distance(final_pose.pose.position, heading, 2)
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
        Set the plan (route) for the agent
        """
        
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
        x, y, z, w = quaternion_from_euler(math.radians(waypoint.rotation.roll), math.radians(waypoint.rotation.pitch), -math.radians(waypoint.rotation.yaw))
        pose.pose.orientation.x = x
        pose.pose.orientation.y = y
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w

        if self.use_transformer:
            pose.pose = self.sim2utm_transformer.transform_pose(pose.pose)

        return pose