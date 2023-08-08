#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import ColorRGBA
from automotive_navigation_msgs.msg import ModuleState
from jsk_rviz_plugins.msg import OverlayText

BLUE = ColorRGBA(0.1, 0.3, 1.0, 0.5)
GREEN = ColorRGBA(0.2, 1.0, 0.2, 0.5)
YELLOW = ColorRGBA(1.0, 1.0, 0.0, 0.5)
RED = ColorRGBA(1.0, 0.0, 0.0, 0.4)

WHITE = ColorRGBA(1.0, 1.0, 1.0, 1.0)

class PacmodStateVisualizer:
    def __init__(self):

        # Publishers
        self.pacmod_state_pub = rospy.Publisher('pacmod_state', OverlayText, queue_size=1)

        # Subscribers
        rospy.Subscriber('/ssc/module_states', ModuleState, self.module_states_callback, queue_size=1)

        # Parameters
        self.global_top = 340
        self.global_left = 10
        self.global_width = 150
        self.global_height = 25
        self.text_size = 15
        self.line_width = 2

    def module_states_callback(self, msg):

        if msg.state == "ready":
            bg_color = GREEN
            text = "Manual"
        elif msg.state == "active" or msg.state == "engaged":
            bg_color = BLUE
            text = "Autonomous"
        elif msg.state == "fatal":
            bg_color = RED
            text = "Fatal"
        else:
            # 'not_ready', 'failure'
            bg_color = YELLOW
            text = msg.state

        pacmod_state = OverlayText()
        pacmod_state.text = text
        pacmod_state.top = self.global_top
        pacmod_state.left = self.global_left
        pacmod_state.width = self.global_width
        pacmod_state.height = self.global_height
        pacmod_state.text_size = self.text_size
        pacmod_state.line_width = self.line_width
        pacmod_state.fg_color = WHITE
        pacmod_state.bg_color = bg_color

        self.pacmod_state_pub.publish(pacmod_state)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('pacmod_state_visualizer', log_level=rospy.INFO)
    node = PacmodStateVisualizer()
    node.run()

