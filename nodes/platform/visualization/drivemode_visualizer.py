#!/usr/bin/env python3

import rospy

from std_msgs.msg import Bool
from autoware_mini.msg import VehicleStatus, StopLineStatusArray, StopLineStatus
from jsk_rviz_plugins.msg import OverlayText


class DrivemodeVisualizer:
    def __init__(self):

        # Variables
        self.is_autonomous = False
        self.teleop_enabled = False
        self.assistance_enabled = False
        self.yield_manual_needs_confirmation = False

        # Publishers
        self.drivemode_text_pub = rospy.Publisher('vehicle_drivemode', OverlayText, queue_size=1, tcp_nodelay=True, latch=True)

        # Subscribers
        rospy.Subscriber('/vehicle/vehicle_status', VehicleStatus, self.vehicle_status_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/vehicle/teleop_enabled', Bool, self.teleop_enabled_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/planning/assistance_enabled', Bool, self.assistance_enabled_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/planning/stop_line_status', StopLineStatusArray, self.stop_line_status_callback, queue_size=1, tcp_nodelay=True)

        # Timer for flash animation
        rospy.Timer(rospy.Duration(0.5), self.flash_callback)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def vehicle_status_callback(self, msg):
        if msg.drivemode != self.is_autonomous:
            self.is_autonomous = msg.drivemode
            self.publish_drivemode_text()

    def teleop_enabled_callback(self, msg):
        if msg.data != self.teleop_enabled:
            self.teleop_enabled = msg.data
            self.publish_drivemode_text()

    def assistance_enabled_callback(self, msg):
        if msg.data != self.assistance_enabled:
            self.assistance_enabled = msg.data
            self.publish_drivemode_text()

    def stop_line_status_callback(self, msg):
        if msg.type == StopLineStatusArray.YIELD_MANUAL:
            needs = msg.statuses and msg.statuses[0].status == StopLineStatus.STATUS_STOP
            if needs != self.yield_manual_needs_confirmation:
                self.yield_manual_needs_confirmation = needs
                self.publish_drivemode_text()

    def flash_callback(self, event):
        try:
            if self.yield_manual_needs_confirmation:
                self.publish_drivemode_text()
        except rospy.ROSTimeMovedBackwardsException:
            pass

    def publish_drivemode_text(self):
        # Hierarchical priority: manual > teleop > assistance > autonomous
        if not self.is_autonomous:
            text, base_color = "MANUAL", "rgb(100, 255, 100)"
        elif self.teleop_enabled:
            text, base_color = "TELEOP", "rgb(230, 60, 60)"
        elif self.assistance_enabled:
            text, base_color = "ASSISTANCE", "rgb(255, 179, 50)"
        else:
            text, base_color = "AUTONOMOUS", "rgb(100, 150, 255)"

        # Apply flash for stop line confirmation
        if self.yield_manual_needs_confirmation:
            flash_on = int(rospy.Time.now().to_sec() / 0.5) % 2 == 0
            color = "rgb(255, 80, 80)" if flash_on else base_color
        else:
            color = base_color

        html = f"<div style='text-align: center; color: {color};'>{text}</div>"
        self.drivemode_text_pub.publish(OverlayText(text=html))

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('drivemode_visualizer')
    node = DrivemodeVisualizer()
    node.run()
