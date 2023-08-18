#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import ColorRGBA, Bool
from automotive_navigation_msgs.msg import ModuleState
from pacmod3_msgs.msg import AllSystemStatuses
from jsk_rviz_plugins.msg import OverlayText
from sensor_msgs.msg import Image
from autoware_msgs.msg import VehicleCmd
import cv2
from cv_bridge import CvBridge


BLUE = ColorRGBA(0.4, 0.6, 1.0, 1.0)
GREEN = ColorRGBA(0.0, 1.0, 0.0, 1.0)
YELLOW = ColorRGBA(1.0, 1.0, 0.0, 1.0)
RED = ColorRGBA(1.0, 0.0, 0.0, 1.0)
BLACK = ColorRGBA(0.0, 0.0, 0.0, 0.8)
WHITE = ColorRGBA(1.0, 1.0, 1.0, 1.0)

# Ket values in /pacmod/all_system_statuses
ACCELERATOR = 0
BRAKES = 1
STEERING = 6
TURN_SIGNALS = 7

class PacmodStateVisualizer:
    def __init__(self):

        # Parameters
        self.image_path = rospy.get_param('~image_path')

        # Publishers
        self.pacmod_state_pub = rospy.Publisher('pacmod_state', OverlayText, queue_size=1)
        self.ssc_state_pub = rospy.Publisher('ssc_state', OverlayText, queue_size=1)
        self.autonomy_pub = rospy.Publisher('autonomy', OverlayText, queue_size=1)
        self.steering_wheel_pub = rospy.Publisher('steering_wheel', Image, queue_size=1)
        self.right_blinker_pub = rospy.Publisher('right_blinker', Image, queue_size=1)
        self.left_blinker_pub =  rospy.Publisher('left_blinker', Image, queue_size=1)

        # Subscribers
        rospy.Subscriber('/ssc/module_states', ModuleState, self.ssc_module_states_callback, queue_size=1)
        rospy.Subscriber('/pacmod/all_system_statuses', AllSystemStatuses, self.pacmod_all_system_statuses, queue_size=1)
        rospy.Subscriber('/pacmod/enabled', Bool, self.pacmod_enabled, queue_size=1)
        rospy.Subscriber('/control/vehicle_cmd', VehicleCmd, self.vehicle_cmd_callback, queue_size=1)

        # Internal parameters
        self.global_top = 330
        self.global_left = 10
        self.global_width = 250

        self.ssc_states={}
        self.bridge = CvBridge()
        self.is_autonomous = False
        

    def pacmod_enabled(self, msg):

        self.is_autonomous = msg.data
        autonomous = self.is_autonomous

        enabled = OverlayText()
        enabled.top = self.global_top
        enabled.left = self.global_left
        enabled.width = self.global_width
        enabled.height = 35
        enabled.text_size = 22
        enabled.bg_color = BLACK
        if autonomous:
            enabled.text = "AUTONOMOUS"
            enabled.fg_color = BLUE
        else:
            enabled.text = "MANUAL"
            enabled.fg_color = GREEN

        self.autonomy_pub.publish(enabled)


    def vehicle_cmd_callback(self, msg):

        autonomous = self.is_autonomous

        # Wheel animation
        wheel_img = cv2.imread(self.image_path + "wheel_s.png")
        if wheel_img is not None:

            height, width = wheel_img.shape[:2]
            # TODO: covert correctly to steering wheel angle
            angle = math.degrees(msg.ctrl_cmd.steering_angle) * 10
            rotation_matrix = cv2.getRotationMatrix2D((width / 2, height / 2), angle, 1)
            rotated_wheel_img = cv2.warpAffine(wheel_img, rotation_matrix, (width, height))

            try:
                if autonomous:
                    # make Blue channel 255, convert to blueish color
                    rotated_wheel_img[:, :, 0] *= 255
                    rotated_wheel_img[:, :, 1] *= 150
                    rotated_wheel_img[:, :, 2] *= 100
                else:
                    # make green channel 255
                    rotated_wheel_img[:, :, 1] *= 255

                wheel_msg = self.bridge.cv2_to_imgmsg(rotated_wheel_img, encoding="bgr8")
                self.steering_wheel_pub.publish(wheel_msg)
            except Exception as e:
                rospy.logerr("Dashboard: Error publishing wheel image: {}".format(str(e)))
        else:
            rospy.logerr("Dashboard: Wheel image not found or couldn't be loaded")


        # Blinkers
        right_img = cv2.imread(self.image_path + "right.png")

        if right_img is not None:
            try:
                if msg.lamp_cmd.r == 1:
                    # make Yellow
                    right_img[:, :, 1] *= 255
                    right_img[:, :, 2] *= 255

                right_msg = self.bridge.cv2_to_imgmsg(right_img, encoding="bgr8")
                self.right_blinker_pub.publish(right_msg)
            except Exception as e:
                rospy.logerr("Dashboard: Error publishing right blinker image: {}".format(str(e)))
        else:
            rospy.logerr("Dashboard: Right blinker image not found or couldn't be loaded")


        left_img = cv2.imread(self.image_path + "left.png")

        if left_img is not None:
            try:
                if msg.lamp_cmd.l == 1:
                    # make Yellow
                    left_img[:, :, 1] *= 255
                    left_img[:, :, 2] *= 255

                left_msg = self.bridge.cv2_to_imgmsg(left_img, encoding="bgr8")
                self.left_blinker_pub.publish(left_msg)
            except Exception as e:
                rospy.logerr("Dashboard: Error publishing left blinker image: {}".format(str(e)))
        else:
            rospy.logerr("Dashboard: Left blinker image not found or couldn't be loaded")


    def ssc_module_states_callback(self, msg):

        # collect all the latest states of individual modules
        self.ssc_states[msg.name] = msg.state + " " + msg.info

        ssc_status_text = ""
        for key in self.ssc_states:
            ssc_status_text += key + ": " + self.ssc_states[key] + "\n"

        fg_color = WHITE
        if 'failure' in ssc_status_text:
            fg_color = YELLOW
        if 'fatal' in ssc_status_text:
            fg_color = RED

        ssc_state = OverlayText()
        ssc_state.top = self.global_top + 115
        ssc_state.left = self.global_left
        ssc_state.width = self.global_width
        ssc_state.height = 70
        ssc_state.text_size = 10
        ssc_state.text = ssc_status_text
        ssc_state.fg_color = fg_color
        ssc_state.bg_color = BLACK

        self.ssc_state_pub.publish(ssc_state)


    def pacmod_all_system_statuses (self, msg):

        # create a string for each module to print out
        accelerator_state = self.get_state_string(ACCELERATOR, msg)
        brakes_state = self.get_state_string(BRAKES, msg)
        steering_state = self.get_state_string(STEERING, msg)
        turn_signals_state = self.get_state_string(TURN_SIGNALS, msg)

        fg_color = WHITE
        if 'Overridden' in accelerator_state:
            fg_color = YELLOW
        if 'Fault' in accelerator_state:
            fg_color = RED

        pacmod_state = OverlayText()
        pacmod_state.top = self.global_top + 185
        pacmod_state.left = self.global_left
        pacmod_state.width = self.global_width
        pacmod_state.height = 80
        pacmod_state.text_size = 10
        pacmod_state.text = "Accelerator:" + accelerator_state + "\nBrakes:" + brakes_state + "\nSteering:" + steering_state + "\nTurn signals:" + turn_signals_state
        pacmod_state.fg_color = fg_color
        pacmod_state.bg_color = BLACK

        self.pacmod_state_pub.publish(pacmod_state)


    def get_state_string(self, module, msg):
        
        state_string =""
        if msg.enabled_status[module].value == "True":
            state_string += " Enabled"
        else:
            state_string += " Disabled"

        if msg.overridden_status[module].value == "True":
            state_string += " Overridden"
        if msg.fault_status[module].value == "True":
            state_string += " Fault"

        return state_string


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('pacmod_state_visualizer', log_level=rospy.INFO)
    node = PacmodStateVisualizer()
    node.run()
