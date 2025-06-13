#!/usr/bin/env python3

import rospy

from carla_msgs.msg import CarlaEgoVehicleStatus
from autoware_mini.msg import VehicleStatus

class CarlaStatusVisualizer:
    def __init__(self):
        self.carla_status_pub = rospy.Publisher('/carla/ego_vehicle/vehicle_status', CarlaEgoVehicleStatus, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/vehicle/vehicle_status', VehicleStatus, self.vehicle_status_callback, queue_size=1, tcp_nodelay=True)

    def vehicle_status_callback(self, vehicle_status):
        carla_status = CarlaEgoVehicleStatus()
        carla_status.header = vehicle_status.header
        carla_status.velocity = vehicle_status.speed / 3.6
        #carla_status.acceleration = ...
        #carla_status.orientation = ...
        carla_status.control.header = vehicle_status.header
        carla_status.control.throttle = vehicle_status.drivepedal / 1000
        carla_status.control.steer = vehicle_status.angle
        carla_status.control.brake = vehicle_status.brakepedal / 1000
        carla_status.control.hand_brake = False
        carla_status.control.reverse = False
        carla_status.control.gear = vehicle_status.current_gear.gear
        carla_status.control.manual_gear_shift = False
        self.carla_status_pub.publish(carla_status)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('carla_status_visualizer', log_level=rospy.INFO)
    node = CarlaStatusVisualizer()
    node.run()
