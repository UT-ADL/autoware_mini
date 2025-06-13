#!/usr/bin/env python3

import rospy

from autoware_mini.msg import Log
from jsk_rviz_plugins.msg import OverlayText

class Logger:
    def __init__(self):

        # Parameters
        self.throttle = rospy.get_param("~throttle")
        self.history_length = rospy.get_param("~history_length")

        # Variables
        self.previous_message = None
        self.previous_message_time = None
        self.message_list = []

        # Publishers
        self.log_text_pub = rospy.Publisher('log_text', OverlayText, queue_size=1, tcp_nodelay=True, latch=True)
        self.log_text_pub.publish(OverlayText(text=""))  # Initialize with empty text

        # Subscribers
        rospy.Subscriber('log_message', Log, self.log_message_callback, queue_size=5, tcp_nodelay=True)

    def log_message_callback(self, msg):

        if self.previous_message_time == None or msg.message != self.previous_message or (rospy.Time.now() - self.previous_message_time).to_sec() >= self.throttle:
            self.update_log_text(msg.message, msg.color, msg.duration)
            self.previous_message = msg.message
            self.previous_message_time = rospy.Time.now()

    def update_log_text(self, message, color, duration):

        # create table row
        if duration != 0:
            row_text = f"<tr><td style='color: {color};'>{message}</td><td style='text-align: right; color: {color};'>{duration:.1f}s</td></tr>"
        else:
            row_text = f"<tr><td style='color: {color};' colspan='2'>{message}</td></tr>"
        # add as first and keep only up to history_length messages
        self.message_list = [row_text] + self.message_list
        self.message_list = self.message_list[:self.history_length]
        text = "<table width='100%'>" + "".join(self.message_list) + "</table>"

        log_text = OverlayText(text = text)
        self.log_text_pub.publish(log_text)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('logger', log_level=rospy.INFO)
    node = Logger()
    node.run()
