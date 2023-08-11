#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import ColorRGBA
from automotive_navigation_msgs.msg import ModuleState
from pacmod3_msgs.msg import AllSystemStatuses
from jsk_rviz_plugins.msg import OverlayText

BLUE = ColorRGBA(0.1, 0.3, 1.0, 0.5)
GREEN = ColorRGBA(0.2, 1.0, 0.2, 0.5)
YELLOW = ColorRGBA(1.0, 1.0, 0.0, 0.5)
RED = ColorRGBA(1.0, 0.0, 0.0, 0.4)
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

        # Parameters
        self.global_top = 340
        self.global_left = 10
        self.global_width = 250
        self.text_size = 10

        self.ssc_states={}


    def ssc_module_states_callback(self, msg):

        # collect all the latest states of individual modules
        self.ssc_states[msg.name] = msg.state # + " " + msg.info

        ssc_status_text = ""
        for key in self.ssc_states:
            ssc_status_text += key + ": " + self.ssc_states[key] + "\n"

        # TODO - this is a hack to get the autonomy state
        # does not work properly and needs rethinking

        bg_color = GREEN
        text = "Ready"
        if "/ssc/veh_controller" in self.ssc_states:
            if self.ssc_states["/ssc/veh_controller"] == "active" or self.ssc_states["/ssc/veh_controller"] == "engaged":
                bg_color = BLUE
                text = "Autonomous"
            elif self.ssc_states["/ssc/veh_controller"] == "failure":
                bg_color = RED
                text = "Failure"
            elif self.ssc_states["/ssc/veh_controller"] == "ready":
                bg_color = GREEN
                text = "Manual"

        autonomy = OverlayText()
        autonomy.text = text
        autonomy.top = self.global_top
        autonomy.left = self.global_left
        autonomy.width = self.global_width
        autonomy.height = 40
        autonomy.text_size = 20
        autonomy.fg_color = WHITE
        autonomy.bg_color = bg_color

        self.autonomy_pub.publish(autonomy)


        ssc_state = OverlayText()
        ssc_state.top = self.global_top + 50
        ssc_state.left = self.global_left
        ssc_state.width = self.global_width
        ssc_state.height = 50
        ssc_state.text_size = 9
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
        pacmod_state.text_size = 9
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
            state_string += " Overriden"
        if msg.fault_status[module].value == "True":
            state_string += " Fault"

        return state_string


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('pacmod_state_visualizer', log_level=rospy.INFO)
    node = PacmodStateVisualizer()
    node.run()
