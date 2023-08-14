#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import ColorRGBA, Bool
from automotive_navigation_msgs.msg import ModuleState
from pacmod3_msgs.msg import AllSystemStatuses
from jsk_rviz_plugins.msg import OverlayText

BLUE = ColorRGBA(0.4, 0.8, 1.0, 0.9)
GREEN = ColorRGBA(0.2, 1.0, 0.2, 0.9)

class AutonomyVisualizer:
    def __init__(self):

        # Publishers
        self.autonomy_pub = rospy.Publisher('autonomy', OverlayText, queue_size=1)

        # Subscribers
        rospy.Subscriber('/pacmod/enabled', Bool, self.pacmod_enabled, queue_size=1)

        # Parameters

    def pacmod_enabled(self, msg):

        enabled = OverlayText()
        enabled.top = 200
        enabled.left = 10
        enabled.width = 200
        enabled.height = 40
        enabled.text_size = 20
        if msg.data:
            enabled.text = "AUTONOMOUS"
            enabled.fg_color = BLUE
        else:
            enabled.text = "MANUAL"
            enabled.fg_color = GREEN

        self.autonomy_pub.publish(enabled)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('autonomy_visualizer', log_level=rospy.INFO)
    node = AutonomyVisualizer()
    node.run()
