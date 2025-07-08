#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation &
# Copyright (c) 2023 Autonomous Driving Lab (ADL), University of Tartu.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

"""
This module provides a ROS autonomous agent interface to control the ego vehicle via a ROS stack
"""
import math
import os
import time
import threading
import signal
import subprocess

import numpy
import carla
import rospy
from cv_bridge import CvBridge

from geometry_msgs.msg import PoseStamped, TwistWithCovariance, Accel
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Image, PointCloud2, NavSatFix, CameraInfo, Imu, PointField, NavSatFix, NavSatStatus
from sensor_msgs.point_cloud2 import create_cloud
from carla_msgs.msg import CarlaEgoVehicleInfo, CarlaEgoVehicleInfoWheel, CarlaEgoVehicleStatus, CarlaEgoVehicleControl, CarlaWorldInfo
from std_msgs.msg import String, Header

from localization.SimulationToUTMTransformer import SimulationToUTMTransformer

from tf.transformations import quaternion_from_euler

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from leaderboard.autoagents.autonomous_agent import Track, AutonomousAgent


def get_entry_point():
    return 'AutowareMiniRosAgent'


class AutowareMiniRosAgent(AutonomousAgent):

    def setup(self, path_to_conf_file):
        """
        setup agent
        """
        self.vehicle_info_published = False
        self.vehicle_info_publisher = None
        self.vehicle_status_publisher = None
        self.global_plan_published = False
        self.world_info_published = False
        self.world_info_publisher = None
        self.map_file_publisher = None
        self.odometry_publisher = None
        self.goal_waypoint_counter = 1
        self.goal_publish_time = 0
        self.rosparam_check_time = 0
        self.step_mode_possible = None
        self.current_map_name = None
        self.current_control = None
        self.latest_imu_msg = None
        self.stack_process = None
        self.timestamp = None

        self.track = Track.MAP
        topic_base = "/carla/ego_vehicle"

        # Sim2UTM transformer placeholders, these are initialized once the stack process has started
        self.use_transformer = None
        self.sim2utm_transformer = None

        # get start_script from environment
        team_code_path = os.environ['TEAM_CODE_ROOT']
        if not team_code_path or not os.path.exists(team_code_path):
            raise IOError("Path '{}' defined by TEAM_CODE_ROOT invalid".format(team_code_path))
        self.start_script = os.path.join(team_code_path, "autoware_mini/scripts/leaderboard/stack_process.sh")
        if not os.path.exists(self.start_script):
            raise IOError("File '{}' defined by TEAM_CODE_ROOT invalid".format(self.start_script))

        # set use_sim_time True before init-node
        rospy.set_param('use_sim_time', True)

        # initialize ros node
        rospy.init_node('autoware_mini_ros_agent')

        self.vehicle_control_event = threading.Event()
        self.step_mode_possible = False

        # subscribe to vehicle control command
        self.vehicle_control_subscriber = rospy.Subscriber(
            "{}/vehicle_control_cmd".format(topic_base), CarlaEgoVehicleControl, self.on_vehicle_control, queue_size=1, tcp_nodelay=True)

        # publish first clock value '0'
        self.clock_publisher = rospy.Publisher('clock', Clock, queue_size=10, tcp_nodelay=True)
        self.clock_publisher.publish(Clock(rospy.Time.from_sec(0)))

        # Waypoints are used for Path visualisation in RVIZ
        self.waypoint_publisher = rospy.Publisher(
            '{}/waypoints'.format(topic_base), Path, queue_size=1, tcp_nodelay=True, latch=True)
        self.goal_publisher = rospy.Publisher(
            '/move_base_simple/goal', PoseStamped, queue_size=1, tcp_nodelay=True, latch=True)

        self.timestamp = None
        self.current_control = carla.VehicleControl()

        # Sensor publisher map
        self.publisher_map = {}
        self.id_to_sensor_type_map = {}
        self.id_to_camera_info_map = {}

        self.cv_bridge = CvBridge()

        # setup ros publishers for sensors
        # pylint: disable=line-too-long
        for sensor in self.sensors():
            self.id_to_sensor_type_map[sensor['id']] = sensor['type']
            if sensor['type'] == 'sensor.camera.rgb':
                self.publisher_map[sensor['id']] = rospy.Publisher(
                    sensor['id'] + "/image_raw", Image, queue_size=1, tcp_nodelay=True)
                self.id_to_camera_info_map[sensor['id']] = self.build_camera_info(sensor)
                self.publisher_map[sensor['id'] + '_info'] = rospy.Publisher(
                    sensor['id'] + "/camera_info", CameraInfo, queue_size=1, tcp_nodelay=True)
            elif sensor['type'] == 'sensor.lidar.ray_cast':
                self.publisher_map[sensor['id']] = rospy.Publisher(
                    sensor['id'] +'/pointcloud', PointCloud2, queue_size=1, tcp_nodelay=True)
            elif sensor['type'] == 'sensor.other.gnss':
                self.publisher_map[sensor['id']] = rospy.Publisher(
                    sensor['id'], NavSatFix, queue_size=1, tcp_nodelay=True)
            elif sensor['type'] == 'sensor.other.imu':
                self.publisher_map[sensor['id']] = rospy.Publisher(
                    sensor['id'], Imu, queue_size=1, tcp_nodelay=True)
            elif sensor['type'] == 'sensor.opendrive_map':
                self.map_file_publisher = rospy.Publisher('/carla/map_file', String, queue_size=1, tcp_nodelay=True, latch=True)
                self.world_info_publisher = rospy.Publisher('/carla/world_info', CarlaWorldInfo, queue_size=1, tcp_nodelay=True, latch=True)
            elif sensor['type'] == 'sensor.speedometer':
                self.odometry_publisher = rospy.Publisher('{}/odometry'.format(topic_base), Odometry, queue_size=1, tcp_nodelay=True)                 
            else:
                raise TypeError("Invalid sensor type: {}".format(sensor['type']))
        
        # Can information is always published
        self.vehicle_status_publisher = rospy.Publisher(
            '{}/vehicle_status'.format(topic_base), CarlaEgoVehicleStatus, queue_size=1, tcp_nodelay=True)
        self.vehicle_info_publisher = rospy.Publisher(
            '{}/vehicle_info'.format(topic_base), CarlaEgoVehicleInfo, queue_size=1, tcp_nodelay=True, latch=True)  
        
    def sensors(self):
        """
        Define the sensor suite required by the agent
        """
        sensors = [
            {'type': 'sensor.camera.rgb', 'x': 0.777, 'y': 0.169, 'z': 1.361, 'roll': 2.3, 'pitch': 10.8, 'yaw': 14.993631,
            'width': 1280, 'height': 720, 'fov': 32.8, 'id': 'camera_fr', 'reading_frequency': 10},
            {'type': 'sensor.camera.rgb', 'x': 0.756, 'y': -0.337, 'z': 1.361, 'roll': -1.6, 'pitch': 9.4, 'yaw': -5.75,
            'width': 1280, 'height': 720, 'fov': 25.3, 'id': 'camera_fl', 'reading_frequency': 10},
            {'type': 'sensor.lidar.ray_cast', 'x': -0.375, 'y': 0.0, 'z': 2.11, 'roll': -1.6 , 'pitch': 0.01,
             'yaw': -87.892, 'id': 'lidar_center', "rotation_frequency": 10, 'reading_frequency': 10},
            {'type': 'sensor.other.gnss', 'x': 0.0, 'y': 0.0, 'z': 1.60, 'reading_frequency': 10, 'id': 'gps/fix'},
            {'type': 'sensor.opendrive_map', 'reading_frequency': 10, 'id': 'OpenDRIVE'},
            {'type': 'sensor.speedometer', 'reading_frequency': 10, 'id': 'speed'},
            {'type': 'sensor.other.imu', 'x': 0.0, 'y': 0.0, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0,
             'yaw': 0.0, 'id': 'gps/imu', 'reading_frequency': 10}
        ]
        return sensors
    
    def on_vehicle_control(self, data):
        """
        callback if a new vehicle control command is received
        """
        cmd = carla.VehicleControl()
        cmd.throttle = data.throttle
        cmd.steer = data.steer
        cmd.brake = data.brake
        cmd.hand_brake = data.hand_brake
        cmd.reverse = data.reverse
        cmd.gear = data.gear
        cmd.manual_gear_shift = data.manual_gear_shift
        self.current_control = cmd
        if not self.vehicle_control_event.is_set():
            self.vehicle_control_event.set()
        # After the first vehicle control is sent out, it is possible to use the stepping mode
        self.step_mode_possible = True

    def publish_plan(self):
        """
        publish the global plan
        """
        msg = Path()
        msg.header.frame_id = "/map"
        msg.header.stamp = rospy.Time.now()
        for wp in self._global_plan_world_coord:
            pose = PoseStamped()
            pose.pose.position.x = wp[0].location.x
            pose.pose.position.y = -wp[0].location.y
            pose.pose.position.z = wp[0].location.z
            quaternion = quaternion_from_euler(
                0, 0, -math.radians(wp[0].rotation.yaw))
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]

            if self.use_transformer:
                pose.pose = self.sim2utm_transformer.transform_pose(pose.pose)

            msg.poses.append(pose)

        self.waypoint_publisher.publish(msg)
        rospy.loginfo("Publishing Plan...")
        
    ########################################
    ############## SENSORS #################
    ########################################

    def get_header(self):
        """
        Returns ROS message header
        """
        return Header(stamp=rospy.Time.from_sec(self.timestamp))

    def build_camera_info(self, attributes):  # pylint: disable=no-self-use
        """
        Private function to compute camera info

        camera info doesn't change over time
        """
        camera_info = CameraInfo()
        # store info without header
        camera_info.header = None
        camera_info.width = int(attributes['width'])
        camera_info.height = int(attributes['height'])
        camera_info.distortion_model = 'plumb_bob'
        cx = camera_info.width / 2.0
        cy = camera_info.height / 2.0
        fx = camera_info.width / (
            2.0 * math.tan(float(attributes['fov']) * math.pi / 360.0))
        fy = fx
        camera_info.K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        camera_info.D = [0, 0, 0, 0, 0]
        camera_info.R = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
        camera_info.P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1.0, 0]
        return camera_info
    
    def publish_camera(self, sensor_id, data):
        """
        Function to publish camera data
        """
        msg = self.cv_bridge.cv2_to_imgmsg(data, encoding='bgra8')
        # the camera data is in respect to the camera's own frame
        msg.header = self.get_header()
        msg.header.frame_id = sensor_id

        cam_info = self.id_to_camera_info_map[sensor_id]
        cam_info.header = msg.header
        self.publisher_map[sensor_id + '_info'].publish(cam_info)
        self.publisher_map[sensor_id].publish(msg)
    
    def publish_gnss(self, sensor_id, data):
        """
        Function to publish gnss data
        """
        msg = NavSatFix()
        msg.header = self.get_header()
        msg.header.frame_id = 'gps'
        msg.latitude = data[0]
        msg.longitude = data[1]
        msg.altitude = data[2]
        msg.status.status = NavSatStatus.STATUS_SBAS_FIX
        # pylint: disable=line-too-long
        msg.status.service = NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS | NavSatStatus.SERVICE_COMPASS | NavSatStatus.SERVICE_GALILEO
        # pylint: enable=line-too-long
        self.publisher_map[sensor_id].publish(msg)

    def publish_imu(self, sensor_id, data):
        """
        Publish IMU data 
        """
        imu_msg = Imu()
        imu_msg.header = self.get_header()

        # Carla uses a left-handed coordinate convention (X forward, Y right, Z up).
        # Here, these measurements are converted to the right-handed ROS convention
        #  (X forward, Y left, Z up).
        imu_msg.linear_acceleration.x = data[0]
        imu_msg.linear_acceleration.y = -data[1]
        imu_msg.linear_acceleration.z = data[2]
        
        imu_msg.angular_velocity.x = -data[3]
        imu_msg.angular_velocity.y = data[4]
        imu_msg.angular_velocity.z = -data[5]
        
        imu_rotation = data[6]

        quaternion = quaternion_from_euler(0, 0, -math.radians(imu_rotation))
        imu_msg.orientation.x = quaternion[0]
        imu_msg.orientation.y = quaternion[1]
        imu_msg.orientation.z = quaternion[2]
        imu_msg.orientation.w = quaternion[3]
        self.latest_imu_msg = imu_msg

        self.publisher_map[sensor_id].publish(imu_msg)

    def publish_lidar(self, sensor_id, data):
        """
        Function to publish lidar data
        """
        header = self.get_header()
        header.frame_id = sensor_id
        lidar_data = numpy.frombuffer(data, dtype=numpy.float32)

        if lidar_data.shape[0] % 4 == 0:
            lidar_data = numpy.reshape(lidar_data, (-1, 4))
            # Permute x and y (CARLA to ROS coordinate conversion)
            lidar_data = lidar_data[..., [1, 0, 2, 3]]
            fields = [PointField('x', 0, PointField.FLOAT32, 1),
                      PointField('y', 4, PointField.FLOAT32, 1),
                      PointField('z', 8, PointField.FLOAT32, 1),
                      PointField('intensity', 12, PointField.FLOAT32, 1)]

            msg = create_cloud(header,fields, lidar_data)
            self.publisher_map[sensor_id].publish(msg)
        else:
            raise TypeError("{}: Cannot reshape LIDAR data buffer - invalid shape {}".format(rospy.get_name(), str(lidar_data.shape)))

    def publish_hd_map(self, data):
        """
        publish hd map data
        """

        # Publish Carla World Info if not published before
        if self.world_info_published:
            world_info = CarlaWorldInfo()
            world_info.map_name = self.current_map_name
            world_info.opendrive = data['opendrive']
            self.world_info_publisher.publish(world_info)
            self.world_info_published = True
        
            # Publish Opendrive map file
            self.map_file_publisher.publish(data['opendrive'])

    def speedometer_callback(self, data):
        """
        Speedometer callback of forward speed
        """

        odo_msg = Odometry()
        odo_msg.header.stamp = self.get_header().stamp
        odo_msg.header.frame_id = "map"
        odo_msg.child_frame_id = "base_link"

        ego_trans = CarlaDataProvider.get_hero_actor().get_transform()
        odo_msg.pose.pose.position.x = ego_trans.location.x
        odo_msg.pose.pose.position.y = -ego_trans.location.y
        odo_msg.pose.pose.position.z = ego_trans.location.z
        odo_msg.pose.pose.orientation.x, odo_msg.pose.pose.orientation.y, odo_msg.pose.pose.orientation.z, odo_msg.pose.pose.orientation.w = \
                quaternion_from_euler(math.radians(ego_trans.rotation.roll), math.radians(ego_trans.rotation.pitch), math.radians(-ego_trans.rotation.yaw))
        
        twist_msg = TwistWithCovariance()        
        twist_msg.twist.linear.x = data['speed']
        if self.latest_imu_msg is not None:
            twist_msg.twist.angular = self.latest_imu_msg.angular_velocity

        odo_msg.twist = twist_msg

        # Publish Odometry
        self.odometry_publisher.publish(odo_msg)

    #####################################
    ############## MISC #################
    #####################################

    def publish_can(self):
        """
        publish can_bus data
        """
        ego_actor = CarlaDataProvider.get_hero_actor()

        vehicle_status = CarlaEgoVehicleStatus(header=self.get_header())
        vehicle_status.velocity = self.get_vehicle_speed(ego_actor.get_velocity())
        vehicle_status.acceleration.linear = self.carla_acceleration_to_ros_accel(ego_actor.get_acceleration()).linear
        ego_trans = CarlaDataProvider.get_hero_actor().get_transform()
        vehicle_status.orientation.x, vehicle_status.orientation.y, vehicle_status.orientation.z, vehicle_status.orientation.w = \
            quaternion_from_euler(math.radians(ego_trans.rotation.roll), math.radians(ego_trans.rotation.pitch), math.radians(-ego_trans.rotation.yaw))
        vehicle_status.control.throttle = self.current_control.throttle
        vehicle_status.control.steer = self.current_control.steer
        vehicle_status.control.brake = self.current_control.brake
        vehicle_status.control.hand_brake = self.current_control.hand_brake
        vehicle_status.control.reverse = self.current_control.reverse
        vehicle_status.control.gear = self.current_control.gear
        vehicle_status.control.manual_gear_shift = self.current_control.manual_gear_shift
        self.vehicle_status_publisher.publish(vehicle_status)
    
        # only send vehicle once (in latched-mode)
        if not self.vehicle_info_published:
            self.vehicle_info_published = True
            vehicle_info = CarlaEgoVehicleInfo()
            vehicle_info.id = ego_actor.id
            vehicle_info.type = ego_actor.type_id
            vehicle_info.rolename = ego_actor.attributes.get('role_name')
            vehicle_physics = ego_actor.get_physics_control()

            for wheel in vehicle_physics.wheels:
                wheel_info = CarlaEgoVehicleInfoWheel()
                wheel_info.tire_friction = wheel.tire_friction
                wheel_info.damping_rate = wheel.damping_rate
                wheel_info.max_steer_angle = math.radians(wheel.max_steer_angle)
                wheel_info.radius = wheel.radius
                wheel_info.max_brake_torque = wheel.max_brake_torque
                wheel_info.max_handbrake_torque = wheel.max_handbrake_torque

                inv_T = numpy.array(ego_actor.get_transform().get_inverse_matrix(), dtype=float)
                wheel_pos_in_map = numpy.array([wheel.position.x/100.0,
                                        wheel.position.y/100.0,
                                        wheel.position.z/100.0,
                                        1.0])
                wheel_pos_in_ego_vehicle = numpy.matmul(inv_T, wheel_pos_in_map)
                wheel_info.position.x = wheel_pos_in_ego_vehicle[0]
                wheel_info.position.y = -wheel_pos_in_ego_vehicle[1]
                wheel_info.position.z = wheel_pos_in_ego_vehicle[2]
                vehicle_info.wheels.append(wheel_info)

            vehicle_info.max_rpm = vehicle_physics.max_rpm
            vehicle_info.max_rpm = vehicle_physics.max_rpm
            vehicle_info.moi = vehicle_physics.moi
            vehicle_info.damping_rate_full_throttle = vehicle_physics.damping_rate_full_throttle
            vehicle_info.damping_rate_zero_throttle_clutch_engaged = \
                vehicle_physics.damping_rate_zero_throttle_clutch_engaged
            vehicle_info.damping_rate_zero_throttle_clutch_disengaged = \
                vehicle_physics.damping_rate_zero_throttle_clutch_disengaged
            vehicle_info.use_gear_autobox = vehicle_physics.use_gear_autobox
            vehicle_info.gear_switch_time = vehicle_physics.gear_switch_time
            vehicle_info.clutch_strength = vehicle_physics.clutch_strength
            vehicle_info.mass = vehicle_physics.mass
            vehicle_info.drag_coefficient = vehicle_physics.drag_coefficient
            vehicle_info.center_of_mass.x = vehicle_physics.center_of_mass.x
            vehicle_info.center_of_mass.y = vehicle_physics.center_of_mass.y
            vehicle_info.center_of_mass.z = vehicle_physics.center_of_mass.z

            self.vehicle_info_publisher.publish(vehicle_info)

    def publish_goal_points(self):

        if self.goal_waypoint_counter < len(self._global_plan_world_coord):

            wp = self._global_plan_world_coord[self.goal_waypoint_counter]
            pose = PoseStamped()
            pose.header.frame_id = "/map"
            pose.pose.position.x = wp[0].location.x
            pose.pose.position.y = -wp[0].location.y
            pose.pose.position.z = wp[0].location.z
            quaternion = quaternion_from_euler(
                0, 0, -math.radians(wp[0].rotation.yaw))
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]

            if self.use_transformer:
                pose.pose = self.sim2utm_transformer.transform_pose(pose.pose)

            self.goal_publisher.publish(pose)
            self.goal_waypoint_counter += 1

        elif self.goal_waypoint_counter == len(self._global_plan_world_coord):

            rospy.loginfo("Goal reached! adding one more forward point so the leaderboard finishes cleanly")

            wp = self._global_plan_world_coord[self.goal_waypoint_counter-1]
            pose = PoseStamped()

            # Calculate the forward vector
            forward_vector = numpy.array([numpy.cos(math.radians(wp[0].rotation.yaw)), 0, numpy.sin(math.radians(wp[0].rotation.yaw))])
            # Normalize the vector and give a fixed forward distance of 10 meters ahead
            unit_vector = (forward_vector / numpy.linalg.norm(forward_vector)) * 10

            pose.header.frame_id = "/map"
            pose.pose.position.x = wp[0].location.x + unit_vector[0]
            pose.pose.position.y = -(wp[0].location.y + unit_vector[1])
            pose.pose.position.z = wp[0].location.z
            quaternion = quaternion_from_euler(
                0, 0, -math.radians(wp[0].rotation.yaw))
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]

            if self.use_transformer:
                pose.pose = self.sim2utm_transformer.transform_pose(pose.pose)

            self.goal_publisher.publish(pose)
            self.goal_waypoint_counter += 1

    def use_stepping_mode(self):  # pylint: disable=no-self-use
        """
        Overload this function to use stepping mode!
        """
        return False

    def run_step(self, input_data, timestamp):
        """
        Execute one step of navigation.
        """
        self.current_map_name = os.path.basename(CarlaDataProvider.get_map().name)

        # Currently, there Tartu sim environemnt and map_name are not synced so we use tartu_demo instead of tartu
        if "tartu" in self.current_map_name.lower():
            self.current_map_name = "tartu_demo"

        if self.stack_process is None and self.current_map_name is not None:
            # execute script that starts the ad stack (remains running)
            rospy.loginfo("Executing stack...")
            self.stack_process = subprocess.Popen(self.start_script + ' ' + self.current_map_name, shell=True, preexec_fn=os.setpgrp)

        self.vehicle_control_event.clear()
        self.timestamp = timestamp
        self.clock_publisher.publish(Clock(rospy.Time.from_sec(timestamp)))

        # Wait for few seconds to read ros parameters from parameter server
        if (timestamp - self.rosparam_check_time) > 2.0 and self.use_transformer is None:
            self.rosparam_check_time = self.timestamp
            try:
                self.use_transformer = rospy.get_param("/carla_localization/use_transformer")
                use_custom_origin = rospy.get_param("/localization/use_custom_origin")
                utm_origin_lat = rospy.get_param("/localization/utm_origin_lat")
                utm_origin_lon = rospy.get_param("/localization/utm_origin_lon")

                self.sim2utm_transformer = SimulationToUTMTransformer(use_custom_origin=use_custom_origin,
                                                                origin_lat=utm_origin_lat,
                                                                origin_lon=utm_origin_lon)
            except (Exception) as e:
                rospy.logerr("%s: Couldn't load parameters from ros parameter server - %s", rospy.get_name(), e) 

        # check if stack is still running
        if self.stack_process and self.stack_process.poll() is not None:
            raise RuntimeError("Stack exited with: {} {}".format(
                self.stack_process.returncode, self.stack_process.communicate()[0]))

        # publish global plan to ROS once
        if self._global_plan_world_coord and not self.global_plan_published and self.use_transformer is not None:
            self.global_plan_published = True
            self.publish_plan()

        new_data_available = False

        # publish data of all sensors
        for key, val in input_data.items():
            new_data_available = True
            sensor_type = self.id_to_sensor_type_map[key]
            if sensor_type == 'sensor.camera.rgb':
                self.publish_camera(key, val[1])
            elif sensor_type == 'sensor.lidar.ray_cast':
                self.publish_lidar(key, val[1])
            elif sensor_type == 'sensor.other.gnss':
                self.publish_gnss(key, val[1])
            elif sensor_type == 'sensor.opendrive_map':
                self.publish_hd_map(val[1])
            elif sensor_type == 'sensor.other.imu':                
                self.publish_imu(key, val[1])
            elif sensor_type == 'sensor.speedometer':
                self.speedometer_callback(val[1])
            else:
                raise TypeError("Invalid sensor type: {}".format(sensor_type))
            
        self.publish_can()

        if self._global_plan_world_coord and (self.timestamp - self.goal_publish_time) > 2.0 and self.use_transformer is not None:
            self.goal_publish_time = self.timestamp
            self.publish_goal_points()

        if self.use_stepping_mode():
            if self.step_mode_possible and new_data_available:
                self.vehicle_control_event.wait()
        # if the stepping mode is not used or active, there is no need to wait here

        return self.current_control
    
    def destroy(self):
        """
        Cleanup of all ROS publishers
        """        
        if self.stack_process:
            if self.stack_process.poll() is None:
                rospy.loginfo("Sending SIGTERM to stack...")
                os.killpg(os.getpgid(self.stack_process.pid), signal.SIGTERM)
                rospy.loginfo("Waiting for termination of stack...")
                self.stack_process.wait()
                time.sleep(5)
                rospy.loginfo("Terminated stack.")

            # cleanup stack process   
            self.stack_process = None

        rospy.loginfo("Stack is no longer running")

        # cleanup ros publishers
        for _, publisher in self.publisher_map.items():
            publisher.unregister()

        if self.odometry_publisher:
            self.odometry_publisher.unregister()
        if self.world_info_publisher:
            self.world_info_publisher.unregister()
        if self.map_file_publisher:
            self.map_file_publisher.unregister()
        if self.vehicle_status_publisher:
            self.vehicle_status_publisher.unregister()
        if self.vehicle_info_publisher:
            self.vehicle_info_publisher.unregister()
        if self.waypoint_publisher:
            self.waypoint_publisher.unregister()
        if self.goal_publisher:
            self.goal_publisher.unregister()

        rospy.loginfo("Cleanup finished")

    def _get_map_name(self, map_full_name):

        if map_full_name is None:
            return None
        name_start_index = map_full_name.rfind("/")
        if name_start_index == -1:
            name_start_index = 0
        else:
            name_start_index = name_start_index + 1        

        return map_full_name[name_start_index:len(map_full_name)]
    
    @staticmethod
    def get_vehicle_speed(velocity_vector):
        """
        Get the absolute speed of a carla vehicle
        :param velocity_vector: the carla velocity vector
        :type velocity_vector: carla.Vector3D
        :return: speed of a carla vehicle [m/s >= 0]
        :rtype: float64
        """
        return math.sqrt(velocity_vector.x**2 + \
            velocity_vector.y**2 + \
            velocity_vector.z**2)
    
    @staticmethod
    def carla_acceleration_to_ros_accel(carla_acceleration):
        """
        Convert a carla acceleration to a ROS accel

        Considers the conversion from left-handed system (unreal) to right-handed
        system (ROS)
        The angular accelerations remain zero.

        :param carla_acceleration: the carla acceleration
        :type carla_acceleration: carla.Vector3D
        :return: a ROS accel
        :rtype: geometry_msgs.msg.Accel
        """
        ros_accel = Accel()
        ros_accel.linear.x = carla_acceleration.x
        ros_accel.linear.y = -carla_acceleration.y
        ros_accel.linear.z = carla_acceleration.z

        return ros_accel
    