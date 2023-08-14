#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import ColorRGBA, Bool
from automotive_navigation_msgs.msg import ModuleState
from pacmod3_msgs.msg import AllSystemStatuses
from jsk_rviz_plugins.msg import OverlayText

BLUE = ColorRGBA(0.6, 0.4, 1.0, 0.9)
GREEN = ColorRGBA(0.2, 1.0, 0.2, 0.9)
YELLOW = ColorRGBA(1.0, 1.0, 0.0, 0.9)
RED = ColorRGBA(1.0, 0.0, 0.0, 0.9)
BLACK = ColorRGBA(0.0, 0.0, 0.0, 0.5)
WHITE = ColorRGBA(1.0, 1.0, 1.0, 1.0)

# Ket values in /pacmod/all_system_statuses
ACCELERATOR = 0
BRAKES = 1
STEERING = 6
TURN_SIGNALS = 7

class PacmodStateVisualizer:
    def __init__(self):

        # Publishers
        self.pacmod_state_pub = rospy.Publisher('pacmod_state', OverlayText, queue_size=1)
        self.ssc_state_pub = rospy.Publisher('ssc_state', OverlayText, queue_size=1)
        self.autonomy_pub = rospy.Publisher('autonomy', OverlayText, queue_size=1)

        # Subscribers
        rospy.Subscriber('/ssc/module_states', ModuleState, self.ssc_module_states_callback, queue_size=1)
        rospy.Subscriber('/pacmod/all_system_statuses', AllSystemStatuses, self.pacmod_all_system_statuses, queue_size=1)
        rospy.Subscriber('/pacmod/enabled', Bool, self.pacmod_enabled, queue_size=1)

        # Parameters
        self.global_top = 200
        self.global_left = 10
        self.global_width = 250

        self.ssc_states={}

    def pacmod_enabled(self, msg):

        enabled = OverlayText()
        enabled.top = self.global_top
        enabled.left = self.global_left
        enabled.width = self.global_width
        enabled.height = 40
        enabled.text_size = 20
        if msg.data:
            enabled.text = "AUTONOMOUS"
            enabled.fg_color = BLUE
        else:
            enabled.text = "MANUAL"
            enabled.fg_color = GREEN

        self.autonomy_pub.publish(enabled)


    def ssc_module_states_callback(self, msg):

        # collect all the latest states of individual modules
        self.ssc_states[msg.name] = msg.state # + " " + msg.info

        ssc_status_text = ""
        for key in self.ssc_states:
            ssc_status_text += key + ": " + self.ssc_states[key] + "\n"

        ssc_state = OverlayText()
        ssc_state.top = self.global_top + 50
        ssc_state.left = self.global_left
        ssc_state.width = self.global_width
        ssc_state.height = 50
        ssc_state.text_size = 10
        ssc_state.text = ssc_status_text
        ssc_state.fg_color = WHITE
        ssc_state.bg_color = BLACK

        self.ssc_state_pub.publish(ssc_state)


    def pacmod_all_system_statuses (self, msg):

        # create a string for each module to print out
        accelerator_state = self.get_state_string(ACCELERATOR, msg)
        brakes_state = self.get_state_string(BRAKES, msg)
        steering_state = self.get_state_string(STEERING, msg)
        turn_signals_state = self.get_state_string(TURN_SIGNALS, msg)

        pacmod_state = OverlayText()
        pacmod_state.top = self.global_top + 100
        pacmod_state.left = self.global_left
        pacmod_state.width = self.global_width
        pacmod_state.height = 60
        pacmod_state.text_size = 10
        pacmod_state.text = "Accelerator:" + accelerator_state + "\nBrakes:" + brakes_state + "\nSteering:" + steering_state + "\nTurn signals:" + turn_signals_state
        pacmod_state.fg_color = WHITE
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
