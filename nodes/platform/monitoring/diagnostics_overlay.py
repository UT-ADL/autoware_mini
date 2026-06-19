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

class DiagnosticsOverlay:
    def __init__(self):
        # Parameters
        self.monitored_components = rospy.get_param('~components')

        # Other initializations
        self.component_statuses = {k: {"display_name": v, "status": DiagnosticStatus.STALE} for k, v in self.monitored_components.items()}

        # Publishers
        self.general_diagnostics_overlay_pub = rospy.Publisher('/dashboard/diagnostics_overlay', OverlayText, queue_size=1)

        # Subscribers
        rospy.Subscriber('/diagnostics', DiagnosticArray, self.diagnostics_callback, queue_size=1)

    def publish_overlay_msg(self):
        overlay_html = "<table width='100%'>"
        for comp in self.component_statuses.keys():
            display_name = self.component_statuses[comp]["display_name"]
            status = self.component_statuses[comp]["status"]
            overlay_html += (
                "<tr>"
                    f"<td style='color:{STATUS_TO_COLOR[status]};'>{display_name}</td>"
                    f"<td style='color:{STATUS_TO_COLOR[status]};'>{STATUS_TO_TEXT[status]}</td>"
                "</tr>"
            )
        overlay_html += "</table>"
        overlay_msg = OverlayText()
        overlay_msg.text = overlay_html
        self.general_diagnostics_overlay_pub.publish(overlay_msg)

    def diagnostics_callback(self, msg):
        for message in msg.status:
            for name_substr in self.monitored_components.keys():
                if name_substr in message.name:
                    self.component_statuses[name_substr]["status"] = message.level
                    break
        self.publish_overlay_msg()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('diagnostics_overlay')
    overlay = DiagnosticsOverlay()
    overlay.run()