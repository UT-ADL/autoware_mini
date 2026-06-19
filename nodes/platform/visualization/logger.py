#!/usr/bin/env python3

import rospy

from autoware_mini.msg import Log
from jsk_rviz_plugins.msg import OverlayText

class Logger:
    def __init__(self):

        # Parameters
        self.instant_message_throttle = rospy.get_param("~instant_message_throttle")
        self.history_length = rospy.get_param("~history_length")

        # Variables
        self.previous_message = None
        self.previous_message_time = None
        self.previous_instant_message = None
        self.previous_instant_message_time = None
        self.last_callback_time = None
        self.message_list = []

        # Publishers
        self.log_text_pub = rospy.Publisher('log_text', OverlayText, queue_size=1, tcp_nodelay=True, latch=True)
        self.log_text_pub.publish(OverlayText(text=""))  # Initialize with empty text

        # Subscribers
        rospy.Subscriber('log_message', Log, self.log_message_callback, queue_size=5, tcp_nodelay=True)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def log_message_callback(self, msg):
        current_time = rospy.Time.now()

        # Throttle instant messages if repeated within the throttle window
        if msg.instant:
            if (self.previous_instant_message == msg.message and self.previous_instant_message_time
                    and (current_time - self.previous_instant_message_time).to_sec() < self.instant_message_throttle):
                return
            self.previous_instant_message = msg.message
            self.previous_instant_message_time = current_time

        is_new_message = (self.previous_message is None or self.previous_message.message != msg.message)

        if is_new_message:
            self.previous_message = msg
            # Use last callback time so first display shows ~0.1s, fall back to current_time for the very first message
            self.previous_message_time = self.last_callback_time if self.last_callback_time is not None else current_time

        duration = 0.0 if self.previous_message_time is None else (current_time - self.previous_message_time).to_sec()
        self.last_callback_time = current_time

        if msg.instant:
            row = f"<tr><td style='color: {msg.color};' colspan='2'>{msg.message}</td></tr>"
        else:
            row = f"<tr><td style='color: {msg.color};'>{msg.message}</td><td style='text-align: right; color: {msg.color};'>{duration:.1f}s</td></tr>"

        if is_new_message:
            self.message_list.insert(0, row)
        else:
            self.message_list[0] = row

        self.message_list = self.message_list[:self.history_length]
        self.log_text_pub.publish(OverlayText(text="<table width='100%'>" + "".join(self.message_list) + "</table>"))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('logger', log_level=rospy.INFO)
    node = Logger()
    node.run()
