#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from autoware_msgs.msg import Lane, VehicleCmd
from geometry_msgs.msg import TwistStamped

class SpeedObstacleVisualizer:
    def __init__(self):

        # Publishers
        self.current_speed_pub = rospy.Publisher('current_speed', Float32, queue_size=1)
        self.target_speed_pub = rospy.Publisher('target_speed', Float32, queue_size=1)
        self.closest_object_distance_pub = rospy.Publisher('closest_object_distance', Float32, queue_size=1)
        self.closest_object_speed_pub = rospy.Publisher('closest_object_speed', Float32, queue_size=1)

        # Subscribers
        rospy.Subscriber('/planning/local_path', Lane, self.local_path_callback, queue_size=1)
        rospy.Subscriber('/control/vehicle_cmd', VehicleCmd, self.vehicle_cmd_callback, queue_size=1)
        rospy.Subscriber('/localization/current_velocity', TwistStamped, self.current_velocity_callback, queue_size=1)

    def local_path_callback(self, msg):
        self.closest_object_distance_pub.publish(Float32(msg.closest_object_distance))
        self.closest_object_speed_pub.publish(Float32(msg.closest_object_velocity))

    def vehicle_cmd_callback(self, msg):
        self.target_speed_pub.publish(Float32(msg.ctrl_cmd.linear_velocity * 3.6))

    def current_velocity_callback(self, msg):
        self.current_speed_pub.publish(Float32(msg.twist.linear.x * 3.6))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('speed_obstacle_visualizer', log_level=rospy.INFO)
    node = SpeedObstacleVisualizer()
    node.run()