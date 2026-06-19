#!/usr/bin/env python3

import rospy

from std_msgs.msg import Bool
from can_msgs.msg import Frame


TELEOP_STATUS_CAN_ID = 0x510
STALE_TIMEOUT = rospy.Duration(0.2)


class TeleopStatus:
    def __init__(self):
        self.teleop_active = False
        self.last_rx = rospy.Time(0)

        self.teleop_enabled_pub = rospy.Publisher('teleop_enabled', Bool, queue_size=1, latch=True, tcp_nodelay=True)

        rospy.Subscriber('/pacmod/can_tx', Frame, self.can_callback, queue_size=200, tcp_nodelay=True)

        rospy.Timer(rospy.Duration(0.1), self.tick_callback)

        self.teleop_enabled_pub.publish(Bool(False))

        rospy.loginfo("%s - initialized", rospy.get_name())

    def can_callback(self, msg):
        if msg.id != TELEOP_STATUS_CAN_ID or msg.dlc < 1:
            return
        active = bool(msg.data[0] & 0x01)
        self.last_rx = rospy.Time.now()
        if active != self.teleop_active:
            self.teleop_active = active
            self.teleop_enabled_pub.publish(Bool(active))

    def tick_callback(self, event):
        try:
            if self.teleop_active and (rospy.Time.now() - self.last_rx) > STALE_TIMEOUT:
                self.teleop_active = False
                self.teleop_enabled_pub.publish(Bool(False))
        except rospy.ROSTimeMovedBackwardsException:
            pass

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('teleop_status')
    node = TeleopStatus()
    node.run()
