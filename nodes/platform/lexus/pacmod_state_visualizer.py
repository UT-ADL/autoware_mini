#!/usr/bin/env python3

import rospy
from std_msgs.msg import ColorRGBA
from automotive_navigation_msgs.msg import ModuleState
from pacmod3_msgs.msg import AllSystemStatuses
from jsk_rviz_plugins.msg import OverlayText

# Overlaytext colors
BLACK = ColorRGBA(0.0, 0.0, 0.0, 0.8)
WHITE = ColorRGBA(1.0, 1.0, 1.0, 1.0)
GRAY = ColorRGBA(0.5, 0.5, 0.5, 1.0)

# Get values in /pacmod/all_system_statuses
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

        # Publishers
        self.pacmod_detailed_pub = rospy.Publisher('pacmod_detailed', OverlayText, queue_size=1)
        self.pacmod_general_pub = rospy.Publisher('pacmod_general', OverlayText, queue_size=1)
        self.ssc_general_pub = rospy.Publisher('ssc_general', OverlayText, queue_size=1)
        self.ssc_detailed_pub = rospy.Publisher('ssc_detailed', OverlayText, queue_size=1)

        # Subscribers
        rospy.Subscriber('/ssc/module_states', ModuleState, self.ssc_module_states_callback, queue_size=1)
        rospy.Subscriber('/pacmod/all_system_statuses', AllSystemStatuses, self.pacmod_all_system_statuses, queue_size=1)

        # Internal parameters
        self.global_top = 285
        self.global_left = 10
        self.global_width = 266

        self.ssc_states={}

    def ssc_module_states_callback(self, msg):

        # Display SSC status - publish general and detailed status separately

        ssc_general_statuses = {
            "Ready": "gray",
            "Engaged": "white",
            "Active": "white",
            "Not_ready": "yellow",
            "Failure": "yellow",
            "Fatal": "red"
            }

        # collect all the latest states of individual modules
        self.ssc_states[SSC_MODULE_NAMES[msg.name]] = msg.state + " " + msg.info

        ssc_detailed_group_title_text = "<span style='font-style: bold; color: white;'>SSC</span>:\n"
        vehicle_controller_status_text = create_ssc_status_string("Vehicle Controller", self.ssc_states)
        speed_model_status_text = create_ssc_status_string("Speed Model", self.ssc_states)
        steering_model_status_text = create_ssc_status_string("Steering Model", self.ssc_states)

        ssc_detailed_status_text = ssc_detailed_group_title_text + vehicle_controller_status_text + speed_model_status_text + steering_model_status_text

        ssc_general_status_text = next(
            (f'<span style="color: {color};">{status}</span>'
            for status, color in ssc_general_statuses.items()
            if status in ssc_detailed_status_text), "")

        ssc_general = OverlayText()
        ssc_general.top = self.global_top
        ssc_general.left = self.global_left
        ssc_general.width = self.global_width
        ssc_general.height = 20
        ssc_general.text_size = 11
        ssc_general.text = "<span style='color: gray;'>SSC</span>: " + ssc_general_status_text
        ssc_general.fg_color = GRAY
        ssc_general.bg_color = BLACK

        self.ssc_general_pub.publish(ssc_general)

        ssc_detailed = OverlayText()
        ssc_detailed.top = self.global_top + 170
        ssc_detailed.left = self.global_left
        ssc_detailed.width = self.global_width
        ssc_detailed.height = 70
        ssc_detailed.text_size = 9
        ssc_detailed.text = "<span style='white-space:nowrap;'>" + ssc_detailed_status_text + "</span>"
        ssc_detailed.fg_color = GRAY
        ssc_detailed.bg_color = BLACK

        self.ssc_detailed_pub.publish(ssc_detailed)


    def pacmod_all_system_statuses (self, msg):

        # Publish Pacmod general and detailed status separately

        pacmod_general_statuses = {
            "Enabled": "white",
            "Disabled": "gray",
            "Overridden": "yellow",
            "Fault": "red",
            "Unavailable": "red"
            }
        
        # create a string for each module to print out
        accelerator_state = create_pacmod_status_string(ACCELERATOR, msg)
        brakes_state = create_pacmod_status_string(BRAKES, msg)
        steering_state = create_pacmod_status_string(STEERING, msg)
        turn_signals_state = create_pacmod_status_string(TURN_SIGNALS, msg)

        pacmod_status_text = "<span style='font-style: bold; color: white;'>PACMOD</span>:\n" + \
                                "Accelerator: " + accelerator_state + "\n" + \
                                "Brakes: " + brakes_state + "\n" + \
                                "Steering: " + steering_state + "\n" + \
                                "Turn signals: " + turn_signals_state

        status = next((status for status in pacmod_general_statuses if status in pacmod_status_text), "")
        pacmod_general_status_text = f'<span style="color: {pacmod_general_statuses[status]};">{status}</span>\n'

        pacmod_general = OverlayText()
        pacmod_general.top = self.global_top + 20
        pacmod_general.left = self.global_left
        pacmod_general.width = self.global_width
        pacmod_general.height = 20
        pacmod_general.text_size = 11
        pacmod_general.text = "<span style='color: gray;'>PACMOD</span>: " + pacmod_general_status_text
        pacmod_general.fg_color = GRAY
        pacmod_general.bg_color = BLACK

        self.pacmod_general_pub.publish(pacmod_general)

        pacmod_detailed = OverlayText()
        pacmod_detailed.top = self.global_top + 240
        pacmod_detailed.left = self.global_left
        pacmod_detailed.width = self.global_width
        pacmod_detailed.height = 80
        pacmod_detailed.text_size = 9
        pacmod_detailed.text = "<span style='white-space:nowrap;'>" + pacmod_status_text + "</span>"
        pacmod_detailed.fg_color = GRAY
        pacmod_detailed.bg_color = BLACK

        self.pacmod_detailed_pub.publish(pacmod_detailed)


    def run(self):
        rospy.spin()


def create_pacmod_status_string(module, msg):

    state_string =""
    if module < len(msg.enabled_status):
        if msg.enabled_status[module].value == "True":
            state_string += "<span style='color: rgb(100, 150, 255);'>Enabled </span>"
        else:
            state_string += "<span style='color: rgb(100, 255, 100);'>Disabled </span>"
    else:
        state_string += "<span style='color: red;'>Unavailable </span>"

    if module < len(msg.overridden_status):
        if msg.overridden_status[module].value == "True":
            state_string += "<span style='color: yellow;'>Overridden </span>"
    else:
        state_string += "<span style='color: red;'>Unavailable </span>"

    if module < len(msg.fault_status):
        if msg.fault_status[module].value == "True":
            state_string += "<span style='color: red;'>Fault </span>"
    else:
        state_string += "<span style='color: red;'>Unavailable </span>"

    return state_string


def create_ssc_status_string(module_name, ssc_states):

    status_string = module_name + ": "
   
    if module_name in ssc_states:
        # check if ssc_states[module_name] starts with Ready or Active
        if ssc_states[module_name].startswith("ready"):
            status_string += "<span style='color: rgb(100, 255, 100);'>{}</span>\n".format(ssc_states[module_name].capitalize())
        elif ssc_states[module_name].startswith("active") or ssc_states[module_name].startswith("engaged"):
            status_string += "<span style='color: rgb(100, 150, 255);'>{}</span>\n".format(ssc_states[module_name].capitalize())
        elif ssc_states[module_name].startswith("not_ready") or ssc_states[module_name].startswith("failure"):
            status_string += "<span style='color: yellow;'>{}</span>\n".format(ssc_states[module_name].capitalize())
        else:
            status_string += "<span style='color: red;'>{}</span>\n".format(ssc_states[module_name].capitalize())
    else:
        status_string += "<span style='color: red;'>Not Available</span>\n"
    
    return status_string


if __name__ == '__main__':
    rospy.init_node('pacmod_state_visualizer', log_level=rospy.INFO)
    node = PacmodStateVisualizer()
    node.run()
