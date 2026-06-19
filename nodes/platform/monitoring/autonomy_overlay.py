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

class AutonomyOverlay:
    def __init__(self):
        # Parameters
        self.monitored_components = rospy.get_param('~components')

        # Initialize component status
        self.component_statuses = {comp: [DiagnosticStatus.STALE, {"Frequency": "-", "Delay": "-"}] for comp in self.monitored_components}

        # Publisher
        self.overlay_pub = rospy.Publisher('/dashboard/autonomy_overlay', OverlayText, queue_size=1)
        self.publish_overlay_msg()  # Publish initial message

        # Subscriber
        rospy.Subscriber('/diagnostics', DiagnosticArray, self.diagnostics_callback, queue_size=1)

    def publish_overlay_msg(self):
        # HTML table header with increased horizontal gaps (padding)
        overlay_html = "<table width='100%'>"
        for comp in self.monitored_components:
            status, stats = self.component_statuses[comp]
            freq = stats["Frequency"]
            delay = stats["Delay"]
            overlay_html += (
                "<tr>"
                    f"<td style='color:{STATUS_TO_COLOR[status]};'>{comp}</td>"
                    f"<td style='text-align:right;'>{freq}</td>"
                    f"<td style='text-align:right;'>{delay}</td>"
                "</tr>"
            )
        overlay_html += "</table>"
        overlay_msg = OverlayText()
        overlay_msg.text = overlay_html
        self.overlay_pub.publish(overlay_msg)

    def diagnostics_callback(self, msg):
        for message in msg.status:
            name = message.name.partition(": ")[2]
            if name in self.component_statuses:
                self.component_statuses[name][0] = message.level
                for kv in message.values:
                    self.component_statuses[name][1][kv.key] = kv.value
        self.publish_overlay_msg()
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('autonomy_overlay')
    overlay = AutonomyOverlay()
    overlay.run()