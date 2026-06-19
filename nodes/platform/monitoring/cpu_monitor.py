#!/usr/bin/env python3

import traceback
import rospy
import diagnostic_updater
import os
from diagnostic_msgs.msg import DiagnosticStatus

class CPUMonitor:
    def __init__(self):

        # Parameters
        self.warning_load_average = rospy.get_param('~warning_load_average')
        if self.warning_load_average == 'auto':
            self.warning_load_average = os.cpu_count()
        if self.warning_load_average is None:
            raise ValueError("Number of CPUs is undetermined!")

        # Publishers
        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID("cpu")
        self.updater.add("CPU", self.update_diagnostics)

        rospy.Timer(rospy.Duration(1), self.call_update)
        
    def update_diagnostics(self, stat):
        
        # Get the shortest interval for the CPU load average
        load_avg = os.getloadavg()[0]
        if load_avg > self.warning_load_average:
            stat.summary(DiagnosticStatus.WARN, f"High CPU load average: {load_avg:.2f}")
        else:
            stat.summary(DiagnosticStatus.OK, f"CPU load average: {load_avg:.2f}")

        stat.add("Load Average", f"{load_avg:.2f}")

        return stat

    def call_update(self, timer_event):
        try:
            self.updater.update()
        except Exception as e:
            rospy.logerr_throttle(10, "%s - Exception in callback: %s", rospy.get_name(), traceback.format_exc())

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('cpu_monitor', log_level=rospy.INFO)
    node = CPUMonitor()
    node.run()
