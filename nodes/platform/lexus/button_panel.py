#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_srvs.srv import Empty

class ButtonPanelNode:
    def __init__(self):
        self.pose_msg = None
        self.marker_id = 0
        self.time_engage = 0

        self.cooldown = rospy.get_param("~cooldown")
        self.enabled = False

        self.engage_pub = rospy.Publisher("engage", Bool, queue_size=10, tcp_nodelay=True)
        self.marker_pub = rospy.Publisher("/log/markers", Marker, queue_size=10, tcp_nodelay=True)

        self.service_lets_go = rospy.ServiceProxy('/planning/service_lets_go', Empty)
        self.service_cancel_route = rospy.ServiceProxy('/planning/cancel_route', Empty)
        self.service_cancel_pose = rospy.ServiceProxy('/localization/cancel_pose', Empty)
        self.service_start_record = rospy.ServiceProxy('/dashboard/start_record', Empty)
        self.service_stop_record = rospy.ServiceProxy('/dashboard/stop_record', Empty)

        rospy.Subscriber("/localization/current_pose", PoseStamped, self.pose_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber("/pacmod/enabled", Bool, self.enabled_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber("joy", Joy, self.joy_callback, queue_size=None, tcp_nodelay=True)

        rospy.loginfo("%s - node started", rospy.get_name())

    def enabled_callback(self, msg):
        self.enabled = msg.data

    def pose_callback(self, msg):
        rospy.logdebug("%s - got pose (%d, %d)", rospy.get_name(), msg.pose.position.x, msg.pose.position.y)
        self.pose_msg = msg

    def joy_callback(self, msg):
        rospy.logdebug("%s - got joy (%d, %d, %d, %d, %d, %d)", rospy.get_name(), msg.buttons[0], msg.buttons[1], msg.buttons[2], msg.buttons[3], msg.buttons[4], msg.buttons[5])
        if msg.buttons[0] == 1:
            if not self.enabled:
                if rospy.get_time() - self.time_engage > self.cooldown:
                    self.engage_pub.publish(Bool(data=True))
                    self.time_engage = rospy.get_time()
                    rospy.logdebug("%s - published engage", rospy.get_name())
                else:
                    rospy.logwarn("%s - did not publish engage, in cooldown", rospy.get_name())
            else:
                # Disables forced stop on the stop lines
                try:
                    response = self.service_lets_go()
                except rospy.ServiceException as e:
                    rospy.logerr("%s - service_lets_go call failed: %s", rospy.get_name(), e)

        elif msg.buttons[1] == 1:
            try:
                response = self.service_cancel_route()
            except rospy.ServiceException as e:
                rospy.logerr("%s - service_cancel_route call failed: %s", rospy.get_name(), e)

        elif msg.buttons[2] == 0:
            try:
                response = self.service_cancel_pose()
            except rospy.ServiceException as e:
                rospy.logerr("%s - service_cancel_pose call failed: %s", rospy.get_name(), e)

        elif msg.buttons[3] == 1:
            try:
                response = self.service_start_record()
            except rospy.ServiceException as e:
                rospy.logerr("%s - service_start_record call failed: %s", rospy.get_name(), e)

        elif msg.buttons[4] == 1:
            try:
                response = self.service_stop_record()
            except rospy.ServiceException as e:
                rospy.logerr("%s - service_stop_record call failed: %s", rospy.get_name(), e)

        elif msg.buttons[5] == 1:
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
                marker.color.r = 1
                marker.color.g = 1
                marker.color.b = 1
                marker.color.a = 1
                self.marker_id += 1
                self.marker_pub.publish(marker)
                rospy.logdebug("%s - published marker (%d, %d)", rospy.get_name(), marker.pose.position.x, marker.pose.position.y)
            else:
                rospy.logwarn("%s - did not publish marker, no current pose yet", rospy.get_name())

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('button_panel', log_level=rospy.INFO)
    node = ButtonPanelNode()
    node.run()
