#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

class ButtonPanelNode:
    def __init__(self):
        self.pose_msg = None
        self.marker_id = 0
        self.time_engage = 0

        self.cooldown = rospy.get_param("~cooldown")

        self.engage_pub = rospy.Publisher("engage", Bool, queue_size=10)
        self.marker_pub = rospy.Publisher("/log/markers", Marker, queue_size=10)

        rospy.Subscriber("/localization/current_pose", PoseStamped, self.pose_callback)
        rospy.Subscriber("joy", Joy, self.joy_callback)

        rospy.loginfo("button_panel - node started")

    def pose_callback(self, msg):
        rospy.logdebug("button_panel - got pose (%d, %d)", msg.pose.position.x, msg.pose.position.y)
        self.pose_msg = msg

    def joy_callback(self, msg):
        rospy.logdebug("button_panel - got joy (%d, %d, %d, %d, %d, %d)", msg.buttons[0], msg.buttons[1], msg.buttons[2], msg.buttons[3], msg.buttons[4], msg.buttons[5])
        if msg.buttons[0] == 1:
            if rospy.get_time() - self.time_engage > self.cooldown:
                self.engage_pub.publish(Bool(data=True))
                self.time_engage = rospy.get_time()
                rospy.logdebug("button_panel - published engage")
            else:
                rospy.logwarn("button_panel - did not publish engage, in cooldown")
        if msg.buttons[1] == 1 or msg.buttons[2] == 0 or msg.buttons[3] == 1 or msg.buttons[4] == 1 or msg.buttons[5] == 1:
            if self.pose_msg is not None:
                marker = Marker()
                marker.header.frame_id = self.pose_msg.header.frame_id
                marker.id = self.marker_id
                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD
                marker.pose = self.pose_msg.pose
                marker.scale.x = 1
                marker.scale.y = 1
                marker.scale.z = 2
                if msg.buttons[1] == 1:
                    marker.color.r = 1
                    marker.color.g = 1
                    marker.color.b = 1
                elif msg.buttons[2] == 0:
                    marker.color.r = 1
                    marker.color.g = 0
                    marker.color.b = 0
                elif msg.buttons[3] == 1:
                    marker.color.r = 0
                    marker.color.g = 1
                    marker.color.b = 0
                elif msg.buttons[4] == 1:
                    marker.color.r = 0
                    marker.color.g = 0
                    marker.color.b = 1
                elif msg.buttons[5] == 1:
                    marker.color.r = 1
                    marker.color.g = 1
                    marker.color.b = 0
                marker.color.a = 1
                self.marker_id += 1
                self.marker_pub.publish(marker)
                rospy.logdebug("button_panel - published marker (%d, %d)", marker.pose.position.x, marker.pose.position.y)
            else:
                rospy.logwarn("button_panel - did not publish marker, no current pose yet")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('button_panel', log_level=rospy.INFO)
    node = ButtonPanelNode()
    node.run()
