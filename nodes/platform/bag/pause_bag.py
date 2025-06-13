#!/usr/bin/env python3

import rospy

from std_srvs.srv import SetBool
from carla_msgs.msg import CarlaControl, CarlaStatus


class PauseBag:

    def __init__(self):

        # Publishers 
        self.carla_status_pub = rospy.Publisher('/carla/status', CarlaStatus, queue_size=10, latch=True)

        # Subscribers
        rospy.Subscriber('/carla/control', CarlaControl, self.carla_control_callback, queue_size=None)

        # Services
        self.pause_playback = rospy.ServiceProxy('/player/pause_playback', SetBool)

    def carla_control_callback(self, msg):
        if msg.command == CarlaControl.PLAY:
            response = self.pause_playback(False)
            if response.success:
                self.publish_carla_status(True)
                rospy.loginfo(response.message)
            else:
                rospy.logerr(response.message)

        elif msg.command == CarlaControl.PAUSE:
            response = self.pause_playback(True)
            if response.success:
                self.publish_carla_status(False)
                rospy.loginfo(response.message)
            else:
                rospy.logerr(response.message)

        elif msg.command == CarlaControl.STEP_ONCE:
            # pause play, ignore error when already paused
            response = self.pause_playback(True)
            self.publish_carla_status(False)
            # play for 0.1 seconds
            response = self.pause_playback(False)
            if not response.success:
                rospy.logerr(response.message)
                return
            self.publish_carla_status(True)
            rospy.sleep(0.1)
            # pause again
            response = self.pause_playback(True)
            if not response.success:
                rospy.logerr(response.message)
                return
            self.publish_carla_status(False)
            rospy.loginfo("Stepped 0.1 seconds")

    def publish_carla_status(self, running):
        carla_status = CarlaStatus()
        carla_status.frame = 0
        carla_status.fixed_delta_seconds = 0.0
        carla_status.synchronous_mode = True
        carla_status.synchronous_mode_running = running
        self.carla_status_pub.publish(carla_status)

    def run(self):
        self.publish_carla_status(True)
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('pause_bag')
    node = PauseBag()
    node.run()