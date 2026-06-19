#!/usr/bin/env python3

import math
import rospy
import numpy as np
from autoware_mini.msg import VehicleStatus, VehicleCommand
from jsk_rviz_plugins.msg import OverlayText
from sensor_msgs.msg import CompressedImage
import cv2


class VehicleStateVisualizer:
    def __init__(self):

        # Parameters
        self.image_path = rospy.get_param('~image_path')
        self.steer_ratio = rospy.get_param('/vehicle/steer_ratio')

        self.turn_signal = VehicleCommand.TURN_STRAIGHT

        # load images for steering wheel
        wheel_autonomous_img = cv2.imread(self.image_path + "wheel_auto.png")
        if wheel_autonomous_img is None:
            rospy.logfatal("%s - Error loading autonomous mode wheel image: %s", rospy.get_name(), self.image_path + "wheel_auto.png")
            rospy.signal_shutdown("Error loading autonomous mode wheel image")
        wheel_manual_img = cv2.imread(self.image_path + "wheel_hands.png")
        if wheel_manual_img is None:
            rospy.logfatal("%s - Error loading manual mode wheel image: %s", rospy.get_name(), self.image_path + "wheel_hands.png")
            rospy.signal_shutdown("Error loading manual mode wheel image")

        # Pre-render rotated and compressed wheel images for every degree
        self.wheel_autonomous_msgs = self.prerender_wheel_images(wheel_autonomous_img)
        self.wheel_manual_msgs = self.prerender_wheel_images(wheel_manual_img)

        # Publishers
        self.steering_wheel_pub = rospy.Publisher('steering_wheel/compressed', CompressedImage, queue_size=1)
        self.right_turn_command_pub = rospy.Publisher('right_turn_command', OverlayText, queue_size=1)
        self.right_turn_status_pub = rospy.Publisher('right_turn_status', OverlayText, queue_size=1)
        self.left_turn_command_pub = rospy.Publisher('left_turn_command', OverlayText, queue_size=1)
        self.left_turn_status_pub = rospy.Publisher('left_turn_status', OverlayText, queue_size=1)

        # Subscribers
        rospy.Subscriber('/vehicle/vehicle_status', VehicleStatus, self.vehicle_status_callback, queue_size=1)
        rospy.Subscriber('/control/vehicle_cmd', VehicleCommand, self.vehicle_cmd_callback, queue_size=1)


    @staticmethod
    def prerender_wheel_images(wheel_img):
        height, width = wheel_img.shape[:2]
        center = (width / 2, height / 2)
        msgs = []
        for angle_deg in range(360):
            rotation_matrix = cv2.getRotationMatrix2D(center, angle_deg, 1)
            rotated_img = cv2.warpAffine(wheel_img, rotation_matrix, (width, height))
            _, png_data = cv2.imencode('.png', rotated_img)
            msg = CompressedImage()
            msg.format = "png"
            msg.data = np.array(png_data).tobytes()
            msgs.append(msg)
        return msgs


    def vehicle_cmd_callback(self, msg):
        self.turn_signal = msg.turn_signal


    def vehicle_status_callback(self, msg):

        # Steering angle - pick pre-rendered wheel image
        steering_wheel_angle = msg.angle * self.steer_ratio * 180 / math.pi
        angle_index = round(steering_wheel_angle) % 360

        if msg.drivemode:
            wheel_msg = self.wheel_autonomous_msgs[angle_index]
        else:
            wheel_msg = self.wheel_manual_msgs[angle_index]

        wheel_msg.header.stamp = msg.header.stamp
        self.steering_wheel_pub.publish(wheel_msg)


        # Turn signals
        left_turn_command_msg = OverlayText()
        if self.turn_signal == VehicleCommand.TURN_LEFT or self.turn_signal == VehicleCommand.TURN_HAZARD:
            left_turn_command_msg.text = "<div style='text-align: right; color: rgb(100, 150, 255);'>&#11013;</div>"
        else:
            left_turn_command_msg.text = "<div style='text-align: right; color: transparent;'>&#11013;</div>"
        self.left_turn_command_pub.publish(left_turn_command_msg)

        left_turn_status_msg = OverlayText()
        if msg.turn_signal == VehicleStatus.TURN_LEFT or msg.turn_signal == VehicleStatus.TURN_HAZARD:
            left_turn_status_msg.text = "<div style='text-align: right; color: yellow;'>&#11013;</div>"
        else:
            left_turn_status_msg.text = "<div style='text-align: right; color: transparent;'>&#11013;</div>"
        self.left_turn_status_pub.publish(left_turn_status_msg)

        right_turn_command_msg = OverlayText()
        if self.turn_signal == VehicleCommand.TURN_RIGHT or self.turn_signal == VehicleCommand.TURN_HAZARD:
            right_turn_command_msg.text = "<div style='text-align: left; color: rgb(100, 150, 255);'>&#10145;</div>"
        else:
            right_turn_command_msg.text = "<div style='text-align: left; color: transparent;'>&#10145;</div>"
        self.right_turn_command_pub.publish(right_turn_command_msg)

        right_turn_status_msg = OverlayText()
        if msg.turn_signal == VehicleStatus.TURN_RIGHT or msg.turn_signal == VehicleStatus.TURN_HAZARD:
            right_turn_status_msg.text = "<div style='text-align: left; color: yellow;'>&#10145;</div>"
        else:
            right_turn_status_msg.text = "<div style='text-align: left; color: transparent;'>&#10145;</div>"
        self.right_turn_status_pub.publish(right_turn_status_msg)


    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('vehicle_state_visualizer', log_level=rospy.INFO)
    node = VehicleStateVisualizer()
    node.run()
