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


class VehicleStateVisualizer:
    def __init__(self):

        # Parameters
        self.image_path = rospy.get_param('~image_path')
        self.steer_ratio = rospy.get_param('/vehicle/steer_ratio')

        # Internal parameters
        self.global_top = 170
        self.global_left = 10
        self.global_width = 266

        self.left_blinker_cmd = None
        self.right_blinker_cmd = None

        self.bridge = CvBridge()

        # load images for steering wheel
        self.wheel_autonomous_img = cv2.imread(self.image_path + "wheel_auto.png")
        if self.wheel_autonomous_img is None:
            rospy.logfatal("%s - Error loading autonomous mode wheel image: %s", rospy.get_name(), self.image_path + "wheel_auto.png")
            rospy.signal_shutdown("Error loading autonomous mode wheel image")
        self.wheel_manual_img = cv2.imread(self.image_path + "wheel_hands.png")
        if self.wheel_manual_img is None:
            rospy.logfatal("%s - Error loading manual mode wheel image: %s", rospy.get_name(), self.image_path + "wheel_hands.png")
            rospy.signal_shutdown("Error loading manual mode wheel image")

        # Publishers
        self.vehicle_drivemode_pub = rospy.Publisher('vehicle_drivemode', OverlayText, queue_size=1)
        self.steering_wheel_pub = rospy.Publisher('steering_wheel', Image, queue_size=1)
        self.right_blinker_cmd_pub = rospy.Publisher('right_blinker_cmd', OverlayText, queue_size=1)
        self.right_blinker_arrow_pub = rospy.Publisher('right_blinker_arrow', OverlayText, queue_size=1)
        self.left_blinker_cmd_pub =  rospy.Publisher('left_blinker_cmd', OverlayText, queue_size=1)
        self.left_blinker_arrow_pub = rospy.Publisher('left_blinker_arrow', OverlayText, queue_size=1)

        # Subscribers
        rospy.Subscriber('/vehicle/vehicle_status', VehicleStatus, self.vehicle_status_callback, queue_size=1)
        rospy.Subscriber('/control/vehicle_cmd', VehicleCmd, self.vehicle_cmd_callback, queue_size=1)


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
        left_blinker_cmd_msg = OverlayText()
        if self.left_blinker_cmd == 1:
            left_blinker_cmd_msg.text = "<div style='text-align: right; color: rgb(100, 150, 255);'>&#11013;</div>"
        else:
            left_blinker_cmd_msg.text = "<div style='text-align: right; color: transparent;'>&#11013;</div>"
        self.left_blinker_cmd_pub.publish(left_blinker_cmd_msg)

        left_blinker_arrow_msg = OverlayText()
        if msg.lamp == VehicleStatus.LAMP_LEFT or msg.lamp == VehicleStatus.LAMP_HAZARD:
            left_blinker_arrow_msg.text = "<div style='text-align: right; color: yellow;'>&#11013;</div>"
        else:
            left_blinker_arrow_msg.text = "<div style='text-align: right; color: transparent;'>&#11013;</div>"
        self.left_blinker_arrow_pub.publish(left_blinker_arrow_msg)

        right_blinker_cmd_msg = OverlayText()
        if self.right_blinker_cmd == 1:
            right_blinker_cmd_msg.text = "<div style='text-align: left; color: rgb(100, 150, 255);'>&#10145;</div>"
        else:
            right_blinker_cmd_msg.text = "<div style='text-align: left; color: transparent;'>&#10145;</div>"
        self.right_blinker_cmd_pub.publish(right_blinker_cmd_msg)

        right_blinker_arrow_msg = OverlayText()
        if msg.lamp == VehicleStatus.LAMP_RIGHT or msg.lamp == VehicleStatus.LAMP_HAZARD:
            right_blinker_arrow_msg.text = "<div style='text-align: left; color: yellow;'>&#10145;</div>"
        else:
            right_blinker_arrow_msg.text = "<div style='text-align: left; color: transparent;'>&#10145;</div>"
        self.right_blinker_arrow_pub.publish(right_blinker_arrow_msg)


    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('vehicle_state_visualizer', log_level=rospy.INFO)
    node = VehicleStateVisualizer()
    node.run()
