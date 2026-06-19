#!/usr/bin/env python3

import os
import traceback
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import simpleaudio as sa

# Sound file paths
BEEP_WARN_PATH = os.path.join(os.path.dirname(__file__), 'beep_warn.wav')
BEEP_ERROR_PATH = os.path.join(os.path.dirname(__file__), 'beep_error.wav')

class DiagnosticsPlayer:
    def __init__(self):
        # Parameters
        self.monitored_components = rospy.get_param('~components')

        self.sound_loop_rate = rospy.get_param('~sound_loop_rate')
        if self.sound_loop_rate <= 0:
            raise ValueError("Sound loop rate must be positive")

        # Other initializations
        self.component_statuses = {}
        for component_dict in self.monitored_components:
            for k in component_dict.keys():
                self.component_statuses[k] = DiagnosticStatus.STALE

        self.warn_wave = sa.WaveObject.from_wave_file(BEEP_WARN_PATH)
        self.error_wave = sa.WaveObject.from_wave_file(BEEP_ERROR_PATH)

        self.sound_loop_period_ns = int(1.0 / self.sound_loop_rate * 1e9)
        self.overall_status = DiagnosticStatus.STALE

        self.play_obj = None
        self.previous_status = DiagnosticStatus.STALE
        rospy.Timer(rospy.Duration(nsecs=self.sound_loop_period_ns), self.play_sound)

        # Subscribers
        rospy.Subscriber('/diagnostics', DiagnosticArray, self.diagnostics_callback, queue_size=1)
    
    def diagnostics_callback(self, msg):
        for message in msg.status:
            for name_substr in self.component_statuses.keys():
                if name_substr in message.name:
                    self.component_statuses[name_substr] = message.level
                    break

        # Update self.overall_status to the MAX of all monitored components
        max_status = DiagnosticStatus.OK
        for status in self.component_statuses.values():
            if status != DiagnosticStatus.STALE and status > max_status:
                max_status = status
        self.overall_status = max_status
    
    def play_sound(self, timer_event):
        """
        Play sound according to self.overall_status.
        OK: Play nothing
        WARN: Play beep_warn.wav repeatedly
        ERROR: Play beep_error.wav repeatedly
        """
        try:
            # Always check for status change
            current_status = self.overall_status
            # Stop any running sound thread if status changed or if status is OK
            if (self.play_obj is not None and
                (current_status != self.previous_status or current_status in [DiagnosticStatus.OK, DiagnosticStatus.STALE])):
                # Stop playing sound
                self.play_obj.stop()
            # Play sound if status is WARN or ERROR or STALE
            if current_status == DiagnosticStatus.WARN:
                if self.play_obj is None or not self.play_obj.is_playing():
                    # Play warning sound
                    self.play_obj = self.warn_wave.play()
            elif current_status == DiagnosticStatus.ERROR:
                if self.play_obj is None or not self.play_obj.is_playing():
                    # Play error sound
                    self.play_obj = self.error_wave.play()
            self.previous_status = current_status
        except Exception as e:
            rospy.logerr_throttle(10, "%s - Exception in callback: %s", rospy.get_name(), traceback.format_exc())


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('diagnostics_player')
    player = DiagnosticsPlayer()
    player.run()