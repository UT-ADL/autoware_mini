#!/usr/bin/env python3

import math

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import TwistStamped
from autoware_mini.msg import VehicleCommand, VehicleStatus, LocalPath
from automotive_platform_msgs.msg import SpeedMode


class DashboardDataExtractor:
    def __init__(self):

        # Parameters
        self.publish_rate = rospy.get_param("/planning/publish_rate")

        # Local variables to store latest values
        self.current_speed = 0.0
        self.target_speed = 0.0
        self.steering_angle_cmd = 0.0
        self.acceleration = 0.0
        self.steering_angle_status = 0.0
        self.acceleration_limit = 0.0
        self.deceleration_limit = 0.0

        # Publishers
        self.current_speed_pub = rospy.Publisher('current_speed', Float32, queue_size=1)
        self.target_speed_pub = rospy.Publisher('target_speed', Float32, queue_size=1)
        self.steering_angle_cmd_pub = rospy.Publisher('steering_angle_cmd', Float32, queue_size=1)
        self.acceleration_pub = rospy.Publisher('acceleration', Float32, queue_size=1)
        self.target_object_distance_pub = rospy.Publisher('target_object_distance', Float32, queue_size=1)
        self.target_object_speed_pub = rospy.Publisher('target_object_speed', Float32, queue_size=1)
        self.steering_angle_status_pub = rospy.Publisher('steering_angle_status', Float32, queue_size=1)
        self.acceleration_limit_pub = rospy.Publisher('acceleration_limit', Float32, queue_size=1)
        self.deceleration_limit_pub = rospy.Publisher('deceleration_limit', Float32, queue_size=1)

        # Subscribers
        rospy.Subscriber('/localization/current_velocity', TwistStamped, self.current_velocity_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/control/vehicle_cmd', VehicleCommand, self.vehicle_cmd_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/planning/local_path', LocalPath, self.local_path_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/vehicle/vehicle_status', VehicleStatus, self.vehicle_status_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/ssc/arbitrated_speed_commands', SpeedMode, self.speed_mode_callback, queue_size=1, tcp_nodelay=True)

        # Timer to publish all topics at publish_rate
        rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.timer_callback)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def current_velocity_callback(self, msg):
        self.current_speed = msg.twist.linear.x * 3.6

    def vehicle_cmd_callback(self, msg):
        self.target_speed = msg.speed * 3.6
        self.steering_angle_cmd = math.degrees(msg.steering_angle)
        self.acceleration = msg.acceleration

    def local_path_callback(self, msg):
        # local_path is already published at publish_rate, no throttling needed
        self.target_object_distance_pub.publish(msg.target_object_distance)
        self.target_object_speed_pub.publish(msg.target_object_speed * 3.6)

    def vehicle_status_callback(self, msg):
        self.steering_angle_status = math.degrees(msg.angle)

    def speed_mode_callback(self, msg):
        self.acceleration_limit = msg.acceleration_limit
        self.deceleration_limit = msg.deceleration_limit

    def timer_callback(self, event):
        self.current_speed_pub.publish(self.current_speed)
        self.target_speed_pub.publish(self.target_speed)
        self.steering_angle_cmd_pub.publish(self.steering_angle_cmd)
        self.acceleration_pub.publish(self.acceleration)
        self.steering_angle_status_pub.publish(self.steering_angle_status)
        self.acceleration_limit_pub.publish(self.acceleration_limit)
        self.deceleration_limit_pub.publish(self.deceleration_limit)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('dashboard_data_extractor', log_level=rospy.INFO)
    node = DashboardDataExtractor()
    node.run()
