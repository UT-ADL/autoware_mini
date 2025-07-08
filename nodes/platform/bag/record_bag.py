#!/usr/bin/env python3

import os
import rospy
import subprocess
from datetime import datetime

from jsk_rviz_plugins.msg import RecordCommand, OverlayText

from std_srvs.srv import Empty, EmptyResponse
from autoware_mini.msg import Log

RED = (255, 0, 0, 255)
GRAY = (128, 128, 128, 128)

class RecordBag:
    def __init__(self):

        # Parameters
        self.blacklist_file = rospy.get_param("~blacklist_file")
        self.recorded_bags_dir = rospy.get_param("~recorded_bags_dir")
        self.bag_name = rospy.get_param("~bag_name")

        os.makedirs(self.recorded_bags_dir, exist_ok=True)
        self.recording_process = None
        self.output_file = None

        # Publishers
        self.recording_symbol_pub = rospy.Publisher('/dashboard/recording_symbol', OverlayText, queue_size=1, latch=True)
        self.log_message_pub = rospy.Publisher('/dashboard/log_message', Log, queue_size=5, tcp_nodelay=True)
        self.record_command_pub = rospy.Publisher('/record_command', RecordCommand, queue_size=1, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber('/record_command', RecordCommand, self.record_bag_callback, queue_size=1, tcp_nodelay=True)

        # Services
        rospy.Service('/dashboard/start_record', Empty, self.start_record_callback)
        rospy.Service('/dashboard/stop_record', Empty, self.stop_record_callback)

        self.publish_recording_symbol(GRAY)

    def record_bag_callback(self, msg):
        if msg.command == RecordCommand.RECORD and self.recording_process is None:
            timestamp = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
            self.output_file = f"{timestamp}_{msg.target}"

            # Blacklist file command
            blacklist_cmd = f"grep -v -P '^#(.*)' {self.blacklist_file} | xargs | sed -e 's/ /|/g'"

            # Full command
            cmd = f"rosbag record -a -O {self.output_file} -x \"$({blacklist_cmd})\""

            self.recording_process = subprocess.Popen(cmd, shell=True, executable="/bin/bash", cwd=self.recorded_bags_dir)
            rospy.loginfo(f"Started recording {self.output_file}")
            self.publish_recording_symbol(RED)
            self.log_message_pub.publish(Log(message = f"Started recording {self.output_file}", color = "white"))

        elif msg.command == RecordCommand.RECORD_STOP and self.recording_process is not None:
            self.recording_process.terminate()
            self.recording_process.wait()
            self.recording_process = None

            rospy.loginfo(f"Stopped recording {self.output_file}")
            self.publish_recording_symbol(GRAY)
            self.log_message_pub.publish(Log(message = f"Stopped recording {self.output_file}", color = "white"))

    def publish_recording_symbol(self, color):
        recording_symbol = OverlayText()
        recording_symbol.text = f"<div style='text-align: center; color: rgba{color};'>REC</div>"
        self.recording_symbol_pub.publish(recording_symbol)

    def start_record_callback(self, msg):
        # Don't block the service, just publish the command
        record_cmd = RecordCommand()
        record_cmd.command = RecordCommand.RECORD
        record_cmd.target = self.bag_name
        self.record_command_pub.publish(record_cmd)
        return EmptyResponse()

    def stop_record_callback(self, msg):
        # Don't block the service, just publish the command
        record_cmd = RecordCommand()
        record_cmd.command = RecordCommand.RECORD_STOP
        record_cmd.target = self.bag_name
        self.record_command_pub.publish(record_cmd)
        return EmptyResponse()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('record_bag')
    node = RecordBag()
    node.run()