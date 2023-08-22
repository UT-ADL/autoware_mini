#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import ColorRGBA, Bool
from automotive_navigation_msgs.msg import ModuleState
from pacmod3_msgs.msg import AllSystemStatuses, SystemRptFloat, SystemRptInt
from jsk_rviz_plugins.msg import OverlayText
from sensor_msgs.msg import Image
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

# /pacmod/turn_rpt values
TURN_RIGHT = 0
TURN_LEFT = 2
STRAIGHT = 1

class PacmodStateVisualizer:
    def __init__(self):

        # Parameters
        self.image_path = rospy.get_param('~image_path')

        # Publishers
        self.pacmod_detailed_pub = rospy.Publisher('pacmod_detailed', OverlayText, queue_size=1)
        self.pacmod_general_pub = rospy.Publisher('pacmod_general', OverlayText, queue_size=1)
        self.ssc_general_pub = rospy.Publisher('ssc_general', OverlayText, queue_size=1)
        self.ssc_detailed_pub = rospy.Publisher('ssc_detailed', OverlayText, queue_size=1)
        self.autonomy_pub = rospy.Publisher('autonomy', Image, queue_size=1)
        self.steering_wheel_pub = rospy.Publisher('steering_wheel', Image, queue_size=1)
        self.right_blinker_pub = rospy.Publisher('right_blinker', Image, queue_size=1)
        self.left_blinker_pub =  rospy.Publisher('left_blinker', Image, queue_size=1)

        # Subscribers
        rospy.Subscriber('/ssc/module_states', ModuleState, self.ssc_module_states_callback, queue_size=1)
        rospy.Subscriber('/pacmod/all_system_statuses', AllSystemStatuses, self.pacmod_all_system_statuses, queue_size=1)
        rospy.Subscriber('/pacmod/enabled', Bool, self.pacmod_enabled, queue_size=1)
        rospy.Subscriber('/pacmod/steering_rpt', SystemRptFloat, self.steering_rpt_callback, queue_size=1)
        rospy.Subscriber('/pacmod/turn_rpt', SystemRptInt, self.turn_rpt_callback, queue_size=1)

        # Internal parameters
        self.global_top = 160
        self.global_left = 10
        self.global_width = 266

        self.ssc_states={}
        self.bridge = CvBridge()
        self.is_autonomous = False

        self.autonomous_img = cv2.imread(self.image_path + "autonomous.png")
        self.manual_img = cv2.imread(self.image_path + "manual.png")


    def pacmod_enabled(self, msg):
        
        # Display AUTONOMOUS / MANUAL status

        self.is_autonomous = msg.data
        autonomous = self.is_autonomous

        if autonomous:
            if self.autonomous_img is not None:
                try:
                    autonomous_msg = self.bridge.cv2_to_imgmsg(self.autonomous_img, encoding="bgr8")
                    self.autonomy_pub.publish(autonomous_msg)
                except Exception as e:
                    rospy.logerr("Dashboard: Error publishing autonomous image: {}".format(str(e)))
            else:
                rospy.logerr("Dashboard: Autonomous image not found or couldn't be loaded")
        else:
            if self.manual_img is not None:
                try:
                    manual_msg = self.bridge.cv2_to_imgmsg(self.manual_img, encoding="bgr8")
                    self.autonomy_pub.publish(manual_msg)
                except Exception as e:
                    rospy.logerr("Dashboard: Error publishing manual image: {}".format(str(e)))
            else:
                rospy.logerr("Dashboard: Manual image not found or couldn't be loaded")


    def steering_rpt_callback(self, msg):

        # Animate steering wheel in autonomous and manual mode

        autonomous = self.is_autonomous

        if autonomous:
            wheel_autonomous_img = cv2.imread(self.image_path + "wheel_s.png")
            if wheel_autonomous_img is not None:
                wheel_img = wheel_autonomous_img
                # make Blue channel 255, convert to blueish color
                wheel_img[:, :, 0] *= 255
                wheel_img[:, :, 1] *= 150
                wheel_img[:, :, 2] *= 100
            else:
                rospy.logerr("Dashboard: Steering wheel autonomous image not found or couldn't be loaded")
        else:
            wheel_manual_img = cv2.imread(self.image_path + "wheel_s_hands.png")
            if wheel_manual_img is not None:
                wheel_img = wheel_manual_img
                # make green channel 255
                wheel_img[:, :, 1] *= 255
            else:
                rospy.logerr("Dashboard: Steering wheel manual image not found or couldn't be loaded")


        height, width = wheel_img.shape[:2]
        # TODO: covert correctly to steering wheel angle
        angle = math.degrees(msg.output)
        rotation_matrix = cv2.getRotationMatrix2D((width / 2, height / 2), angle, 1)
        rotated_wheel_img = cv2.warpAffine(wheel_img, rotation_matrix, (width, height))

        try:
            wheel_msg = self.bridge.cv2_to_imgmsg(rotated_wheel_img, encoding="bgr8")
            self.steering_wheel_pub.publish(wheel_msg)
        except Exception as e:
            rospy.logerr("Dashboard: Error publishing steering wheel image: {}".format(str(e)))


    def turn_rpt_callback(self, msg):

        # Visualize blinkers:
        # CMD - command (sent by autonomy stack)
        # Arrow - actual blinker state, includes manually switched blinkers

        blinker_left_img = cv2.imread(self.image_path + "left_cmd.png")
        blinker_right_img = cv2.imread(self.image_path + "right_cmd.png")

        if blinker_left_img is not None:
            try:
                if msg.command == TURN_LEFT:
                    # make CMD Yellow
                    pixels_with_value_80 = blinker_left_img[:, :, 0] == 80
                    blinker_left_img[pixels_with_value_80, 0] = 0
                    blinker_left_img[pixels_with_value_80, 1] = 255
                    blinker_left_img[pixels_with_value_80, 2] = 255

                if msg.output == TURN_LEFT:
                    # make arrow Yellow
                    pixels_with_value_70 = blinker_left_img[:, :, 0] == 70
                    blinker_left_img[pixels_with_value_70, 0] = 0
                    blinker_left_img[pixels_with_value_70, 1] = 255
                    blinker_left_img[pixels_with_value_70, 2] = 255

                left_msg = self.bridge.cv2_to_imgmsg(blinker_left_img, encoding="bgr8")
                self.left_blinker_pub.publish(left_msg)
            except Exception as e:
                rospy.logerr("Dashboard: Error publishing left blinker image: {}".format(str(e)))
        else:
            rospy.logerr("Dashboard: Left blinker image not found or couldn't be loaded")

        if blinker_right_img is not None:
            try:
                if msg.command == TURN_RIGHT:
                    # make CMD Yellow
                    pixels_with_value_80 = blinker_right_img[:, :, 1] == 80
                    blinker_right_img[pixels_with_value_80, 0] = 0
                    blinker_right_img[pixels_with_value_80, 1] = 255
                    blinker_right_img[pixels_with_value_80, 2] = 255

                if msg.output == TURN_RIGHT:
                    # make arrow Yellow
                    pixels_with_value_70 = blinker_right_img[:, :, 1] == 70
                    blinker_right_img[pixels_with_value_70, 0] = 0
                    blinker_right_img[pixels_with_value_70, 1] = 255
                    blinker_right_img[pixels_with_value_70, 2] = 255

                right_msg = self.bridge.cv2_to_imgmsg(blinker_right_img, encoding="bgr8")
                self.right_blinker_pub.publish(right_msg)
            except Exception as e:
                rospy.logerr("Dashboard: Error publishing right blinker image: {}".format(str(e)))
        else:
            rospy.logerr("Dashboard: Right blinker image not found or couldn't be loaded")


    def ssc_module_states_callback(self, msg):

        # Display SSC status - publish general and detailed status separately

        # collect all the latest states of individual modules
        self.ssc_states[msg.name] = msg.state + " " + msg.info

        ssc_status_text = "SSC:\n"
        for key in self.ssc_states:
            ssc_status_text += key + ": " + self.ssc_states[key] + "\n"

        fg_color = WHITE
        ssc_general_status = ""
        if 'ready' in ssc_status_text:
            ssc_general_status = "Ready"
        if 'active' in ssc_status_text:
            ssc_general_status = "Active"
        if 'not_ready' in ssc_status_text:
            ssc_general_status = "Not Ready"
            fg_color = YELLOW
        if 'failure' in ssc_status_text:
            ssc_general_status = "Failure"
            fg_color = YELLOW
        if 'fatal' in ssc_status_text:
            ssc_general_status = "Fatal"
            fg_color = RED

        ssc_general = OverlayText()
        ssc_general.top = self.global_top + 115
        ssc_general.left = self.global_left
        ssc_general.width = self.global_width
        ssc_general.height = 20
        ssc_general.text_size = 11
        ssc_general.text = "SSC: " + ssc_general_status
        ssc_general.fg_color = fg_color
        ssc_general.bg_color = BLACK

        self.ssc_general_pub.publish(ssc_general)

        ssc_detailed = OverlayText()
        ssc_detailed.top = 430
        ssc_detailed.left = self.global_left
        ssc_detailed.width = self.global_width
        ssc_detailed.height = 70
        ssc_detailed.text_size = 9
        ssc_detailed.text = ssc_status_text
        ssc_detailed.fg_color = fg_color
        ssc_detailed.bg_color = BLACK

        self.ssc_detailed_pub.publish(ssc_detailed)


    def pacmod_all_system_statuses (self, msg):

        # Publish Pacmod general and detailed status separately

        # create a string for each module to print out
        accelerator_state = self.get_state_string(ACCELERATOR, msg)
        brakes_state = self.get_state_string(BRAKES, msg)
        steering_state = self.get_state_string(STEERING, msg)
        turn_signals_state = self.get_state_string(TURN_SIGNALS, msg)

        pacmod_status_text = "Pacmod:\nAccelerator:" + accelerator_state + "\nBrakes:" + brakes_state + "\nSteering:" + steering_state + "\nTurn signals:" + turn_signals_state

        fg_color = WHITE
        pacmod_general_status = "Enabled"
        if 'Disabled' in pacmod_status_text:
            pacmod_general_status = "Disabled"
            fg_color = WHITE
        if 'Overridden' in pacmod_status_text:
            pacmod_general_status = "Overridden"
            fg_color = YELLOW
        if 'Fault' in pacmod_status_text:
            pacmod_general_status = "Fault"
            fg_color = RED

        pacmod_general = OverlayText()
        pacmod_general.top = self.global_top + 135
        pacmod_general.left = self.global_left
        pacmod_general.width = self.global_width
        pacmod_general.height = 20
        pacmod_general.text_size = 11
        pacmod_general.text = "Pacmod: " + pacmod_general_status
        pacmod_general.fg_color = fg_color
        pacmod_general.bg_color = BLACK

        self.pacmod_general_pub.publish(pacmod_general)


        pacmod_detailed = OverlayText()
        pacmod_detailed.top = 500
        pacmod_detailed.left = self.global_left
        pacmod_detailed.width = self.global_width
        pacmod_detailed.height = 80
        pacmod_detailed.text_size = 9
        pacmod_detailed.text = pacmod_status_text
        pacmod_detailed.fg_color = fg_color
        pacmod_detailed.bg_color = BLACK

        self.pacmod_detailed_pub.publish(pacmod_detailed)


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
