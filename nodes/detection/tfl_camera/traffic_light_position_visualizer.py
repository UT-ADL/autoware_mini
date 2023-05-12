#!/usr/bin/env python3

import rospy
import cv2

from autoware_msgs.msg import Signals
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

SIGNAL_TYPE_TO_COLOR = {
    1: (255, 0, 0),   # Red
    2: (0, 255, 0),   # Green
    3: (255, 255, 0), # Yellow
    4: (0, 0, 255)    # Unknown
}


class TrafficLightPositionVisualizer:
    def __init__(self):

        # Internal parameters
        self.signals = []

        # Initialize the CvBridge for converting ROS messages to OpenCV images
        self.bridge = CvBridge()

        # Publishers
        self.tfl_positions_pub = rospy.Publisher('tfl_positions_visualization', Image, queue_size=1)

        # Subscribers
        # TODO create time synced subscriber? once the timestamps have been fixed downstream
        rospy.Subscriber('signals', Signals, self.signals_callback, queue_size=1)
        rospy.Subscriber('/camera_fl/image_raw', Image, self.image_callback, queue_size=1)

    def image_callback(self, msg):
        # Convert the ROS message to OpenCV format using a CvBridge
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except CvBridgeError as e:
            rospy.logerr("traffic_light_position_visualizer - ", e)

        if len(self.signals) > 0:
            for signal in self.signals:
                # Draw the signals on the image
                image = cv2.circle(image, 
                                   (signal.u, signal.v),
                                   # TODO, replace with radius 
                                   radius = signal.radius, 
                                   color = SIGNAL_TYPE_TO_COLOR[signal.type], 
                                   thickness = 2)
            
            # publish only if there are signals
            try:
                self.tfl_positions_pub.publish(self.bridge.cv2_to_imgmsg(image, encoding='rgb8'))
            except CvBridgeError as e:
                rospy.logerr("traffic_light_position_visualizer - ", e)

    def signals_callback(self, msg):
        self.signals = msg.Signals


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('traffic_light_position_visualizer', log_level=rospy.INFO)
    node = TrafficLightPositionVisualizer()
    node.run()