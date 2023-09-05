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
        self.steer_ratio = rospy.get_param('/vehicle/steer_ratio')

        # Publishers
        self.vehicle_drivemode_pub = rospy.Publisher('vehicle_drivemode', OverlayText, queue_size=1)
        self.steering_wheel_pub = rospy.Publisher('steering_wheel', Image, queue_size=1)
        self.right_blinker_cmd_pub = rospy.Publisher('right_blinker_cmd', Image, queue_size=1)
        self.right_blinker_arrow_pub = rospy.Publisher('right_blinker_arrow', Image, queue_size=1)
        self.left_blinker_cmd_pub =  rospy.Publisher('left_blinker_cmd', Image, queue_size=1)
        self.left_blinker_arrow_pub = rospy.Publisher('left_blinker_arrow', Image, queue_size=1)

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
        self.wheel_autonomous_img = cv2.imread(self.image_path + "wheel_auto.png")
        self.wheel_manual_img = cv2.imread(self.image_path + "wheel_hands.png")

        # report errors on missing wheel images
        if self.wheel_autonomous_img is None:
            rospy.logerr("%s - Error loading autonomous mode wheel image: %s", rospy.get_name(), self.image_path + "wheel_auto.png")
        if self.wheel_manual_img is None:
            rospy.logerr("%s - Error loading manual mode wheel image: %s", rospy.get_name(), self.image_path + "wheel_hands.png")


        # load images for blinkers
        blinker_left_cmd_on_img = cv2.imread(self.image_path + "left_cmd_on.png")
        if blinker_left_cmd_on_img is not None:
            self.blinker_left_cmd_on_msg = self.bridge.cv2_to_imgmsg(blinker_left_cmd_on_img, encoding="bgr8")
        else:
            rospy.logerr("%s - Error loading left blinker cmd on image: %s", rospy.get_name(), self.image_path + "left_cmd_on.png")

        blinker_left_cmd_off_img = cv2.imread(self.image_path + "left_cmd_off.png")
        if blinker_left_cmd_off_img is not None:
            self.blinker_left_cmd_off_msg = self.bridge.cv2_to_imgmsg(blinker_left_cmd_off_img, encoding="bgr8")
        else:
            rospy.logerr("%s - Error loading left blinker cmd off image: %s", rospy.get_name(), self.image_path + "left_cmd_off.png")

        blinker_left_arrow_on_img = cv2.imread(self.image_path + "left_arrow_on.png")
        if blinker_left_arrow_on_img is not None:
            self.blinker_left_arrow_on_msg = self.bridge.cv2_to_imgmsg(blinker_left_arrow_on_img, encoding="bgr8")
        else:
            rospy.logerr("%s - Error loading left blinker arrow on image: %s", rospy.get_name(), self.image_path + "left_arrow_on.png")

        blinker_left_arrow_off_img = cv2.imread(self.image_path + "left_arrow_off.png")
        if blinker_left_arrow_off_img is not None:
            self.blinker_left_arrow_off_msg = self.bridge.cv2_to_imgmsg(blinker_left_arrow_off_img, encoding="bgr8")
        else:
            rospy.logerr("%s - Error loading left blinker arrow off image: %s", rospy.get_name(), self.image_path + "left_arrow_off.png")

        blinker_right_cmd_on_img = cv2.imread(self.image_path + "right_cmd_on.png")
        if blinker_right_cmd_on_img is not None:
            self.blinker_right_cmd_on_msg = self.bridge.cv2_to_imgmsg(blinker_right_cmd_on_img, encoding="bgr8")
        else:
            rospy.logerr("%s - Error loading right blinker cmd on image: %s", rospy.get_name(), self.image_path + "right_cmd_on.png")

        blinker_right_cmd_off_img = cv2.imread(self.image_path + "right_cmd_off.png")
        if blinker_right_cmd_off_img is not None:
            self.blinker_right_cmd_off_msg = self.bridge.cv2_to_imgmsg(blinker_right_cmd_off_img, encoding="bgr8")
        else:
            rospy.logerr("%s - Error loading right blinker cmd off image: %s", rospy.get_name(), self.image_path + "right_cmd_off.png")

        blinker_right_arrow_on_img = cv2.imread(self.image_path + "right_arrow_on.png")
        if blinker_right_arrow_on_img is not None:
            self.blinker_right_arrow_on_msg = self.bridge.cv2_to_imgmsg(blinker_right_arrow_on_img, encoding="bgr8")
        else:
            rospy.logerr("%s - Error loading right blinker arrow on image: %s", rospy.get_name(), self.image_path + "right_arrow_on.png")

        blinker_right_arrow_off_img = cv2.imread(self.image_path + "right_arrow_off.png")
        if blinker_right_arrow_off_img is not None:
            self.blinker_right_arrow_off_msg = self.bridge.cv2_to_imgmsg(blinker_right_arrow_off_img, encoding="bgr8")
        else:
            rospy.logerr("%s - Error loading right blinker arrow off image: %s", rospy.get_name(), self.image_path + "right_arrow_off.png")


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
        steering_wheel_angle = msg.angle * self.steer_ratio * 180 / 3.14

        # if one of the images is missing, don't publish anything
        if self.wheel_autonomous_img is not None and self.wheel_manual_img is not None:
            if is_autonomous:
                    wheel_img = self.wheel_autonomous_img
            else:
                    wheel_img = self.wheel_manual_img

            # rotate image
            height, width = wheel_img.shape[:2]
            rotation_matrix = cv2.getRotationMatrix2D((width / 2, height / 2), steering_wheel_angle, 1)
            rotated_wheel_img = cv2.warpAffine(wheel_img, rotation_matrix, (width, height))

            wheel_msg = self.bridge.cv2_to_imgmsg(rotated_wheel_img, encoding="bgr8")
            self.steering_wheel_pub.publish(wheel_msg)


        # Blinkers
        if self.left_blinker_cmd == 1:
            self.left_blinker_cmd_pub.publish(self.blinker_left_cmd_on_msg)
        else:
            self.left_blinker_cmd_pub.publish(self.blinker_left_cmd_off_msg)

        if msg.lamp == LAMP_LEFT or msg.lamp == LAMP_HAZARD:
            self.left_blinker_arrow_pub.publish(self.blinker_left_arrow_on_msg)
        else:
            self.left_blinker_arrow_pub.publish(self.blinker_left_arrow_off_msg)

        if self.right_blinker_cmd == 1:
            self.right_blinker_cmd_pub.publish(self.blinker_right_cmd_on_msg)
        else:
            self.right_blinker_cmd_pub.publish(self.blinker_right_cmd_off_msg)

        if msg.lamp == LAMP_RIGHT or msg.lamp == LAMP_HAZARD:
            self.right_blinker_arrow_pub.publish(self.blinker_right_arrow_on_msg)
        else:
            self.right_blinker_arrow_pub.publish(self.blinker_right_arrow_off_msg)


    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('vehicle_state_visualizer', log_level=rospy.INFO)
    node = VehicleStateVisualizer()
    node.run()
