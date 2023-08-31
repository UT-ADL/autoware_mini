#!/usr/bin/env python3

import rospy
from autoware_msgs.msg import VehicleStatus, VehicleCmd
from jsk_rviz_plugins.msg import OverlayText
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA
import cv2
from cv_bridge import CvBridge

BLACK = ColorRGBA(0.0, 0.0, 0.0, 0.8)
WHITE = ColorRGBA(1.0, 1.0, 1.0, 1.0)

# VehicleStatus.lamp constants
LAMP_LEFT = 1
LAMP_RIGHT = 2
LAMP_HAZARD = 3

class VehicleStateVisualizer:
    def __init__(self):

        # Parameters
        self.image_path = rospy.get_param('~image_path')

        # Publishers
        self.vehicle_drivemode_pub = rospy.Publisher('vehicle_drivemode', OverlayText, queue_size=1)
        self.steering_wheel_pub = rospy.Publisher('steering_wheel', Image, queue_size=1)
        self.right_blinker_pub = rospy.Publisher('right_blinker', Image, queue_size=1)
        self.left_blinker_pub =  rospy.Publisher('left_blinker', Image, queue_size=1)

        # Subscribers
        rospy.Subscriber('/vehicle/vehicle_status', VehicleStatus, self.vehicle_status_callback, queue_size=1)
        rospy.Subscriber('/control/vehicle_cmd', VehicleCmd, self.vehicle_cmd_callback, queue_size=1)


        # Internal parameters
        self.global_top = 170
        self.global_left = 10
        self.global_width = 266
        self.bridge = CvBridge()

        self.left_blinker_cmd = None
        self.right_blinker_cmd = None

        # load images for steering wheel and blinkers
        self.wheel_autonomous_img = cv2.imread(self.image_path + "wheel_s.png")
        self.wheel_manual_img = cv2.imread(self.image_path + "wheel_s_hands.png")
        self.blinker_left_img = cv2.imread(self.image_path + "left_cmd.png")
        self.blinker_right_img = cv2.imread(self.image_path + "right_cmd.png")

        # report errors on missing images
        if self.wheel_autonomous_img is None:
            rospy.logerr("%s - Error loading autonomous mode wheel image: %s", rospy.get_name(), self.image_path + "wheel_s.png")
        if self.wheel_manual_img is None:
            rospy.logerr("%s - Error loading manual mode wheel image: %s", rospy.get_name(), self.image_path + "wheel_s_hands.png")
        if self.blinker_left_img is None:
            rospy.logerr("%s - Error loading left blinker image: %s", rospy.get_name(), self.image_path + "left_cmd.png")
        if self.blinker_right_img is None:
            rospy.logerr("%s - Error loading right blinker image: %s", rospy.get_name(), self.image_path + "right_cmd.png")


    def vehicle_cmd_callback(self, msg):
        # get blinker command
        self.left_blinker_cmd = msg.lamp_cmd.l
        self.right_blinker_cmd = msg.lamp_cmd.r


    def vehicle_status_callback(self, msg):

        # Drivemode - autonomy / manual
        is_autonomous = msg.drivemode
        if is_autonomous:
            drivemode_text = "<div style='text-align: center; color: rgb(100, 150, 255);'>AUTONOMOUS</div>"
        else:
            drivemode_text = "<div style='text-align: center; color: rgb(100, 255, 100);'>MANUAL</div>"

        drivemode = OverlayText()
        drivemode.top = self.global_top
        drivemode.left = self.global_left
        drivemode.width = self.global_width
        drivemode.height = 35
        drivemode.text_size = 20
        drivemode.text = drivemode_text
        drivemode.fg_color = WHITE
        drivemode.bg_color = BLACK

        self.vehicle_drivemode_pub.publish(drivemode)


        # Steering angle - animate wheel
        # TODO convert it correctly to steering wheel angle in degrees
        steering_wheel_angle = msg.angle * 180 / 3.14

        # if one of the images is missing, don't publish anything
        if self.wheel_autonomous_img is not None and self.wheel_manual_img is not None:
            if is_autonomous:
                    wheel_img = self.wheel_autonomous_img.copy()
                    # convert to blueish color
                    wheel_img[:, :, 0] *= 255
                    wheel_img[:, :, 1] *= 150
                    wheel_img[:, :, 2] *= 100
            else:
                    wheel_img = self.wheel_manual_img.copy()
                    # make green channel 255
                    wheel_img[:, :, 0] *= 100
                    wheel_img[:, :, 1] *= 255
                    wheel_img[:, :, 2] *= 100

            # rotate image
            height, width = wheel_img.shape[:2]
            rotation_matrix = cv2.getRotationMatrix2D((width / 2, height / 2), steering_wheel_angle, 1)
            rotated_wheel_img = cv2.warpAffine(wheel_img, rotation_matrix, (width, height))

            wheel_msg = self.bridge.cv2_to_imgmsg(rotated_wheel_img, encoding="bgr8")
            self.steering_wheel_pub.publish(wheel_msg)


        # Blinkers - visualize command and state
        # if one of the images is missing, don't publish anything
        if self.blinker_left_img is not None and self.blinker_right_img is not None:

            # Left blinker
            blinker_left_img = self.blinker_left_img.copy()

            if self.left_blinker_cmd == 1:
                # make CMD Yellow
                pixels_with_value_80 = blinker_left_img[:, :, 0] == 80
                blinker_left_img[pixels_with_value_80, 0] = 0
                blinker_left_img[pixels_with_value_80, 1] = 255
                blinker_left_img[pixels_with_value_80, 2] = 255
            if msg.lamp == LAMP_LEFT or msg.lamp == LAMP_HAZARD:
                # make arrow Yellow
                pixels_with_value_70 = blinker_left_img[:, :, 0] == 70
                blinker_left_img[pixels_with_value_70, 0] = 0
                blinker_left_img[pixels_with_value_70, 1] = 255
                blinker_left_img[pixels_with_value_70, 2] = 255

            left_msg = self.bridge.cv2_to_imgmsg(blinker_left_img, encoding="bgr8")
            self.left_blinker_pub.publish(left_msg)

            # Right blinker
            blinker_right_img = self.blinker_right_img.copy()

            if self.right_blinker_cmd == 1:
                # make CMD Yellow
                pixels_with_value_80 = blinker_right_img[:, :, 0] == 80
                blinker_right_img[pixels_with_value_80, 0] = 0
                blinker_right_img[pixels_with_value_80, 1] = 255
                blinker_right_img[pixels_with_value_80, 2] = 255
            if msg.lamp == LAMP_RIGHT or msg.lamp == LAMP_HAZARD:
                # make arrow Yellow
                pixels_with_value_70 = blinker_right_img[:, :, 0] == 70
                blinker_right_img[pixels_with_value_70, 0] = 0
                blinker_right_img[pixels_with_value_70, 1] = 255
                blinker_right_img[pixels_with_value_70, 2] = 255

            right_msg = self.bridge.cv2_to_imgmsg(blinker_right_img, encoding="bgr8")
            self.right_blinker_pub.publish(right_msg)


    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('vehicle_state_visualizer', log_level=rospy.INFO)
    node = VehicleStateVisualizer()
    node.run()
