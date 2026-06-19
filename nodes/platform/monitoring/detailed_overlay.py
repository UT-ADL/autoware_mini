#!/usr/bin/env python3

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from jsk_rviz_plugins.msg import OverlayText

STATUS_TO_COLOR = {
    DiagnosticStatus.OK: "white",
    DiagnosticStatus.WARN: "yellow",
    DiagnosticStatus.ERROR: "red",
    DiagnosticStatus.STALE: "lightgray"
}

STATUS_TO_TEXT = {
    DiagnosticStatus.OK: "OK",
    DiagnosticStatus.WARN: "WARN",
    DiagnosticStatus.ERROR: "ERROR",
    DiagnosticStatus.STALE: "STALE"
}

class DetailedOverlay:
    def __init__(self):
        # Parameters
        self.monitored_components = rospy.get_param('~components')

        # Publisher initializations
        self.display_publishers = {
            k: rospy.Publisher(dashboard_topic, OverlayText, queue_size=1) for k, dashboard_topic in self.monitored_components.items()
        }

        # Subscribers
        rospy.Subscriber('/diagnostics', DiagnosticArray, self.diagnostics_callback, queue_size=1)
    
    def diagnostics_callback(self, msg):
        for message in msg.status:
            name = message.name.partition(": ")[2]
            if name in self.monitored_components:
                display_publisher = self.display_publishers[name]
                if display_publisher.get_num_connections() > 0:
                    overlay_msg = OverlayText()
                    overlay_msg.text = f"{name}: <span style='color:{STATUS_TO_COLOR[message.level]};'>{STATUS_TO_TEXT[message.level]}</span>"
                    for kv in message.values:
                        overlay_msg.text += f"\n<span style='color: gray;'>{kv.key}</span>: {kv.value}"
                    display_publisher.publish(overlay_msg)
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('detailed_overlay')
    overlay = DetailedOverlay()
    overlay.run()