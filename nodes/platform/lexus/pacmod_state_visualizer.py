#!/usr/bin/env python3

import rospy
from std_msgs.msg import ColorRGBA
from automotive_navigation_msgs.msg import ModuleState
from pacmod3_msgs.msg import GlobalRpt
from jsk_rviz_plugins.msg import OverlayText

# Overlaytext colors
BLACK = ColorRGBA(0.0, 0.0, 0.0, 0.8)
WHITE = ColorRGBA(1.0, 1.0, 1.0, 1.0)
GRAY = ColorRGBA(0.5, 0.5, 0.5, 1.0)


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
        rospy.Subscriber('/pacmod/global_rpt', GlobalRpt, self.pacmod_global_report_callback, queue_size=1)

        # Internal parameters
        self.global_top = 285
        self.global_left = 10
        self.global_width = 266

        self.ssc_states={}

    def ssc_module_states_callback(self, msg):

        # Display SSC status - publish general and detailed status separately

        ssc_general_statuses = {
            "Ready": "white",
            "Engaged": "white",
            "Active": "white",
            "Not_ready": "yellow",
            "Failure": "yellow",
            "Fatal": "red"
            }

        # collect all the latest states of individual modules
        self.ssc_states[SSC_MODULE_NAMES[msg.name]] = msg.state + " " + msg.info

        ssc_detailed_group_title_text = "<span style='font-style: bold; color: white;'>SSC:</span>\n"
        vehicle_controller_status_text = create_ssc_status_string("Vehicle Controller", self.ssc_states)
        speed_model_status_text = create_ssc_status_string("Speed Model", self.ssc_states)
        steering_model_status_text = create_ssc_status_string("Steering Model", self.ssc_states)

        ssc_detailed_status_text = ssc_detailed_group_title_text + vehicle_controller_status_text + speed_model_status_text + steering_model_status_text

        ssc_general_status_text = ""

        for status, color in reversed(ssc_general_statuses.items()):
            if status in ssc_detailed_status_text:
                ssc_general_status_text = f'<span style="color: {color};">{status}</span>'
                break

        ssc_general = OverlayText()
        ssc_general.top = self.global_top
        ssc_general.left = self.global_left
        ssc_general.width = self.global_width
        ssc_general.height = 20
        ssc_general.text_size = 11
        ssc_general.text = "<span style='color: gray;'>SSC:</span> " + ssc_general_status_text
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


    def pacmod_global_report_callback (self, msg):

        # Pacmod general status text
        if msg.enabled:
            pacmod_general_status_text = "<span style='color: white;'>Enabled</span>"
        elif msg.config_fault_active:
            pacmod_general_status_text = "<span style='color: red;'>Disabled, Config Fault</span>"
        elif msg.pacmod_sys_fault_active:
            pacmod_general_status_text = "<span style='color: red;'>Disabled, System Fault</span>"
        elif msg.user_can_timeout or msg.steering_can_timeout or msg.brake_can_timeout or msg.subsystem_can_timeout or msg.vehicle_can_timeout:
            pacmod_general_status_text = "<span style='color: yellow;'>Disabled, CAN Timeout</span>"
        elif msg.override_active:
            pacmod_general_status_text = "<span style='color: yellow;'>Disabled, Overridden</span>"
        else:
            pacmod_general_status_text = "<span style='color: white;'>Ready</span>"

        # Pacmod detailed status text
        pacmod_detailed_group_title_text = "<span style='font-style: bold; color: white;'>PACMOD:</span>\n"
        pacmod_detailed_status_text = ""
        if msg.pacmod_sys_fault_active:
            pacmod_detailed_status_text += "System Fault: <span style='color: red;'>True</span>\n"
        else:
            pacmod_detailed_status_text += "System Fault: <span style='color: white;'>False</span>\n"
        if msg.config_fault_active:
            pacmod_detailed_status_text += "Config Fault: <span style='color: red;'>True</span>\n"
        else:
            pacmod_detailed_status_text += "Config Fault: <span style='color: white;'>False</span>\n"
        if msg.user_can_timeout:
            pacmod_detailed_status_text += "User CAN Timeout: <span style='color: red;'>True</span>\n"
        else:
            pacmod_detailed_status_text += "User CAN Timeout: <span style='color: white;'>False</span>\n"
        if msg.steering_can_timeout:
            pacmod_detailed_status_text += "Steering CAN Timeout: <span style='color: red;'>True</span>\n"
        else:
            pacmod_detailed_status_text += "Steering CAN Timeout: <span style='color: white;'>False</span>\n"
        if msg.brake_can_timeout:
            pacmod_detailed_status_text += "Brake CAN Timeout: <span style='color: red;'>True</span>\n"
        else:
            pacmod_detailed_status_text += "Brake CAN Timeout: <span style='color: white;'>False</span>\n"
        if msg.subsystem_can_timeout:
            pacmod_detailed_status_text += "Subsystem CAN Timeout: <span style='color: red;'>True</span>\n"
        else:
            pacmod_detailed_status_text += "Subsystem CAN Timeout: <span style='color: white;'>False</span>\n"
        if msg.vehicle_can_timeout:
            pacmod_detailed_status_text += "Vehicle CAN Timeout: <span style='color: red;'>True</span>\n"
        else:
            pacmod_detailed_status_text += "Vehicle CAN Timeout: <span style='color: white;'>False</span>\n"


        pacmod_general = OverlayText()
        pacmod_general.top = self.global_top + 20
        pacmod_general.left = self.global_left
        pacmod_general.width = self.global_width
        pacmod_general.height = 20
        pacmod_general.text_size = 11
        pacmod_general.text = "<span style='color: gray;'>PACMOD:</span> " + "<span style='white-space:nowrap;'>" + pacmod_general_status_text + "</span>"
        pacmod_general.fg_color = GRAY
        pacmod_general.bg_color = BLACK

        self.pacmod_general_pub.publish(pacmod_general)

        pacmod_detailed = OverlayText()
        pacmod_detailed.top = self.global_top + 240
        pacmod_detailed.left = self.global_left
        pacmod_detailed.width = self.global_width
        pacmod_detailed.height = 130
        pacmod_detailed.text_size = 9
        pacmod_detailed.text = pacmod_detailed_group_title_text + "<span style='white-space:nowrap;'>" + pacmod_detailed_status_text + "</span>"
        pacmod_detailed.fg_color = GRAY
        pacmod_detailed.bg_color = BLACK

        self.pacmod_detailed_pub.publish(pacmod_detailed)


    def run(self):
        rospy.spin()


def create_ssc_status_string(module_name, ssc_states):

    status_string = module_name + ": "
   
    if module_name in ssc_states:
        # check if ssc_states[module_name] starts with Ready or Active
        if ssc_states[module_name].startswith("ready"):
            status_string += "<span style='color: white;'>{}</span>\n".format(ssc_states[module_name].capitalize())
        elif ssc_states[module_name].startswith("active") or ssc_states[module_name].startswith("engaged"):
            status_string += "<span style='color: white;'>{}</span>\n".format(ssc_states[module_name].capitalize())
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
