#!/usr/bin/env python3
#
# Copyright (c) 2023 Autonomous Driving Lab (ADL), University of Tartu.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
receive autoware_msgs::VehicleCmd
        carla_msgs::CarlaEgoVehicleInfo
        carla_msgs::CarlaEgoVehicleStatus
        
publish ackermann_msgs::AckermannDrive
        autoware_msgs::VehicleStatus
        std_msgs::Float64
"""
import math
import rospy
from ackermann_msgs.msg import AckermannDrive
from autoware_msgs.msg import VehicleCmd, VehicleStatus, Gear
from carla_msgs.msg import CarlaEgoVehicleInfo, CarlaEgoVehicleStatus
from std_msgs.msg import Float64


class CarlaVehicleInterface:

    def __init__(self):

        # Node parameters
        self.max_steer_angle = math.radians(rospy.get_param("~max_steer_angle"))

        # Publishers
        self.ackerman_cmd_pub = rospy.Publisher(
            '/carla/ego_vehicle/ackermann_cmd', AckermannDrive, queue_size=1, tcp_nodelay=True)
        self.target_speed_pub = rospy.Publisher(
            '/carla/ego_vehicle/target_speed', Float64, queue_size=1, tcp_nodelay=True)
        self.vehicle_status_pub = rospy.Publisher(
            '/vehicle/vehicle_status', VehicleStatus, queue_size=1, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber('/control/vehicle_cmd', VehicleCmd,
                         self.vehicle_cmd_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/carla/ego_vehicle/vehicle_info', CarlaEgoVehicleInfo,
                         self.vehicle_info_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/carla/ego_vehicle/vehicle_status', CarlaEgoVehicleStatus,
                         self.vehicle_status_callback, queue_size=1, tcp_nodelay=True)
        

    def vehicle_cmd_callback(self, data):
        """
        callback for vehicle cmds
        """

        msg = AckermannDrive()
        msg.speed = data.ctrl_cmd.linear_velocity
        msg.steering_angle = data.ctrl_cmd.steering_angle
        msg.acceleration = data.ctrl_cmd.linear_acceleration

        # Publish ackermandrive cmd
        self.ackerman_cmd_pub.publish(msg)
        # Publish target speed (currently used only by scenario runner)
        self.target_speed_pub.publish(Float64(msg.speed))

    def vehicle_info_callback(self, data):
        """
        callback for vehicle info
        """
        # get max steering angle (use smallest non-zero value of all wheels)
        for wheel in data.wheels:
            if wheel.max_steer_angle and wheel.max_steer_angle < self.max_steer_angle:
                self.max_steering_angle = wheel.max_steer_angle

    def vehicle_status_callback(self, data):
        """
        callback for vehicle status
        """

        status = VehicleStatus()
        status.header = data.header

        status.angle = -data.control.steer * self.max_steer_angle
        status.speed = data.velocity * 3.6  # speed is expected in km/h

        if data.control.reverse:
            status.current_gear.gear = Gear.REVERSE
        else:
            status.current_gear.gear = Gear.DRIVE

        if data.control.manual_gear_shift:
            status.drivemode = VehicleStatus.MODE_MANUAL
        else:
            status.drivemode = VehicleStatus.MODE_AUTO

        self.vehicle_status_pub.publish(status)


    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('carla_vehicle_interface', log_level=rospy.INFO)
    node = CarlaVehicleInterface()
    node.run()
