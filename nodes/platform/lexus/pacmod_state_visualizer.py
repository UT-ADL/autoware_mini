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


BLACK = ColorRGBA(0.0, 0.0, 0.0, 0.8)
WHITE = ColorRGBA(1.0, 1.0, 1.0, 1.0)
GRAY = ColorRGBA(0.5, 0.5, 0.5, 1.0)

# Ket values in /pacmod/all_system_statuses
ACCELERATOR = 0
BRAKES = 1
STEERING = 6
TURN_SIGNALS = 7

# /pacmod/turn_rpt values
TURN_RIGHT = 0
TURN_LEFT = 2
STRAIGHT = 1

SSC_MODULE_NAMES = {
    '/ssc/veh_controller': 'Vehicle Controller',
    '/ssc/speed_model': 'Speed Model',
    '/ssc/steering_model': 'Steering Model',
}

class PacmodStateVisualizer:
    def __init__(self):

        # Parameters
        self.image_path = rospy.get_param('~image_path')

        # Publishers
        self.pacmod_detailed_pub = rospy.Publisher('pacmod_detailed', OverlayText, queue_size=1)
        self.pacmod_general_pub = rospy.Publisher('pacmod_general', OverlayText, queue_size=1)
        self.ssc_general_pub = rospy.Publisher('ssc_general', OverlayText, queue_size=1)
        self.ssc_detailed_pub = rospy.Publisher('ssc_detailed', OverlayText, queue_size=1)
        self.steering_wheel_pub = rospy.Publisher('steering_wheel', Image, queue_size=1)
        self.right_blinker_pub = rospy.Publisher('right_blinker', Image, queue_size=1)
        self.left_blinker_pub =  rospy.Publisher('left_blinker', Image, queue_size=1)
        self.autonomy_text_pub = rospy.Publisher('autonomy_text', OverlayText, queue_size=1)

        # Subscribers
        rospy.Subscriber('/ssc/module_states', ModuleState, self.ssc_module_states_callback, queue_size=1)
        rospy.Subscriber('/pacmod/all_system_statuses', AllSystemStatuses, self.pacmod_all_system_statuses, queue_size=1)
        rospy.Subscriber('/pacmod/enabled', Bool, self.pacmod_enabled, queue_size=1)
        rospy.Subscriber('/pacmod/steering_rpt', SystemRptFloat, self.steering_rpt_callback, queue_size=1)
        rospy.Subscriber('/pacmod/turn_rpt', SystemRptInt, self.turn_rpt_callback, queue_size=1)

        # Internal parameters
        self.global_top = 170
        self.global_left = 10
        self.global_width = 266

        self.ssc_states={}
        self.bridge = CvBridge()
        self.is_autonomous = False


    def pacmod_enabled(self, msg):
        
        # Display AUTONOMOUS / MANUAL status

        self.is_autonomous = msg.data
        autonomous = self.is_autonomous

        if autonomous:
            pacmod_enabled_text = "<div style=\"text-align: center; color: blue;\">AUTONOMOUS</div>"
        else:
            pacmod_enabled_text = "<div style=\"text-align: center; color: green;\">MANUAL</div>"

        text = OverlayText()
        text.top = self.global_top
        text.left = self.global_left
        text.width = self.global_width
        text.height = 35
        text.text_size = 20
        text.text = pacmod_enabled_text
        text.fg_color = WHITE
        text.bg_color = BLACK

        self.autonomy_text_pub.publish(text)


    def steering_rpt_callback(self, msg):

        # Animate steering wheel in autonomous and manual mode

        autonomous = self.is_autonomous

        if autonomous:
            wheel_autonomous_img = cv2.imread(self.image_path + "wheel_s.png")
            if wheel_autonomous_img is not None:
                wheel_img = wheel_autonomous_img
                # convert to blueish color
                wheel_img[:, :, 0] *= 255
                wheel_img[:, :, 1] *= 0
                wheel_img[:, :, 2] *= 0
            else:
                rospy.logerr("Dashboard: Steering wheel autonomous image not found or couldn't be loaded")
        else:
            wheel_manual_img = cv2.imread(self.image_path + "wheel_s_hands.png")
            if wheel_manual_img is not None:
                wheel_img = wheel_manual_img
                # make green channel 255
                wheel_img[:, :, 0] *= 0
                wheel_img[:, :, 1] *= 255
                wheel_img[:, :, 2] *= 0
            else:
                rospy.logerr("Dashboard: Steering wheel manual image not found or couldn't be loaded")


        height, width = wheel_img.shape[:2]
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
        self.ssc_states[SSC_MODULE_NAMES[msg.name]] = msg.state + " " + msg.info

        ssc_detailed_group_title_text = "<span style=\"font-style: bold; color: white;\">SSC</span>:\n"
        vehicle_controller_status_text = create_ssc_status_string("Vehicle Controller", self.ssc_states)
        speed_model_status_text = create_ssc_status_string("Speed Model", self.ssc_states)
        steering_model_status_text = create_ssc_status_string("Steering Model", self.ssc_states)

        ssc_detailed_status_text = ssc_detailed_group_title_text + vehicle_controller_status_text + speed_model_status_text + steering_model_status_text

        if "Ready" in ssc_detailed_status_text:
            ssc_general_status = "<span style=\"color: {};\">Ready</span>".format("lightgreen")
        if "Engaged" in ssc_detailed_status_text:
            ssc_general_status = "<span style=\"color: {};\">Engaged</span>".format("lightblue")
        if "Active" in ssc_detailed_status_text:
            ssc_general_status = "<span style=\"color: {};\">Active</span>".format("lightblue")
        if "Not_ready" in ssc_detailed_status_text:
            ssc_general_status = "<span style=\"color: {};\">Not Ready</span>".format("yellow")
        if "Failure" in ssc_detailed_status_text:
            ssc_general_status = "<span style=\"color: {};\">Failure</span>".format("yellow")
        if "Fatal" in ssc_detailed_status_text:
            ssc_general_status = "<span style=\"color: {};\">Fatal</span>".format("red")

        ssc_general = OverlayText()
        ssc_general.top = self.global_top + 115
        ssc_general.left = self.global_left
        ssc_general.width = self.global_width
        ssc_general.height = 20
        ssc_general.text_size = 11
        ssc_general.text = "<span style=\"color: white;\">SSC</span>: " + ssc_general_status
        ssc_general.fg_color = GRAY
        ssc_general.bg_color = BLACK

        self.ssc_general_pub.publish(ssc_general)

        ssc_detailed = OverlayText()
        ssc_detailed.top = self.global_top + 285
        ssc_detailed.left = self.global_left
        ssc_detailed.width = self.global_width
        ssc_detailed.height = 70
        ssc_detailed.text_size = 9
        ssc_detailed.text = ssc_detailed_status_text
        ssc_detailed.fg_color = GRAY
        ssc_detailed.bg_color = BLACK

        self.ssc_detailed_pub.publish(ssc_detailed)


    def pacmod_all_system_statuses (self, msg):

        # Publish Pacmod general and detailed status separately

        # create a string for each module to print out
        accelerator_state = create_pacmod_status_string(ACCELERATOR, msg)
        brakes_state = create_pacmod_status_string(BRAKES, msg)
        steering_state = create_pacmod_status_string(STEERING, msg)
        turn_signals_state = create_pacmod_status_string(TURN_SIGNALS, msg)

        pacmod_status_text = "<span style=\"font-style: bold; color: white;\">PACMOD</span>:\n" + \
                                "Accelerator: " + accelerator_state + "\n" + \
                                "Brakes: " + brakes_state + "\n" + \
                                "Steering: " + steering_state + "\n" + \
                                "Turn signals: " + turn_signals_state

        
        if "Enabled" in pacmod_status_text:
            pacmod_general_status = "<span style=\"color: {};\">{}</span>\n".format("lightblue","Enabled ")
        if "Disabled" in pacmod_status_text:
            pacmod_general_status = "<span style=\"color: {};\">{}</span>\n".format("lightgreen","Disabled ")
        if "Overridden" in pacmod_status_text:
            pacmod_general_status = "<span style=\"color: {};\">{}</span>\n".format("yellow","Overridden ")
        if "Fault" in pacmod_status_text:
            pacmod_general_status = "<span style=\"color: {};\">{}</span>\n".format("red","Fault ")

        pacmod_general = OverlayText()
        pacmod_general.top = self.global_top + 135
        pacmod_general.left = self.global_left
        pacmod_general.width = self.global_width
        pacmod_general.height = 20
        pacmod_general.text_size = 11
        pacmod_general.text = "<span style=\"color: white;\">PACMOD</span>: " + pacmod_general_status
        pacmod_general.fg_color = GRAY
        pacmod_general.bg_color = BLACK

        self.pacmod_general_pub.publish(pacmod_general)


        pacmod_detailed = OverlayText()
        pacmod_detailed.top = self.global_top + 355
        pacmod_detailed.left = self.global_left
        pacmod_detailed.width = self.global_width
        pacmod_detailed.height = 80
        pacmod_detailed.text_size = 9
        pacmod_detailed.text = pacmod_status_text
        pacmod_detailed.fg_color = GRAY
        pacmod_detailed.bg_color = BLACK

        self.pacmod_detailed_pub.publish(pacmod_detailed)


    def run(self):
        rospy.spin()


def create_pacmod_status_string(module, msg):

    state_string =""
    if msg.enabled_status[module].value == "True":
        state_string += "<span style=\"color: {};\">{}</span>".format("lightblue", "Enabled ")
    else:
        state_string += "<span style=\"color: {};\">{}</span>".format("lightgreen", "Disabled ")

    if msg.overridden_status[module].value == "True":
        state_string += "<span style=\"color: {};\">{}</span>".format("yellow", "Overridden ")

    if msg.fault_status[module].value == "True":
        state_string += "<span style=\"color: {};\">{}</span>".format("red", "Fault ")

    return state_string


def create_ssc_status_string(module_name, ssc_states):

    status_string = module_name + ": "
   
    if module_name in ssc_states:
        # check if ssc_states[module_name] starts with Ready or Active
        if ssc_states[module_name].startswith("ready"):
            status_string += "<span style=\"color: {};\">{}</span>\n".format("lightgreen", ssc_states[module_name].capitalize())
        elif ssc_states[module_name].startswith("active") or ssc_states[module_name].startswith("engaged"):
            status_string += "<span style=\"color: {};\">{}</span>\n".format("lightblue", ssc_states[module_name].capitalize())
        elif ssc_states[module_name].startswith("not_ready") or ssc_states[module_name].startswith("failure"):
            status_string += "<span style=\"color: {};\">{}</span>\n".format("yellow", ssc_states[module_name].capitalize())
        else:
            status_string += "<span style=\"color: {};\">{}</span>\n".format("red", ssc_states[module_name].capitalize())
    else:
        status_string += "<span style=\"color: {};\">{}</span>\n".format("red", "Not Available")
    
    return status_string



if __name__ == '__main__':
    rospy.init_node('pacmod_state_visualizer', log_level=rospy.INFO)
    node = PacmodStateVisualizer()
    node.run()
