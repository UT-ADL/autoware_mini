#!/usr/bin/env python3

import rospy
import message_filters
import cv2

from camera_tfl_helpers import convert_signals_to_rois

from sensor_msgs.msg import Image, CameraInfo
from autoware_msgs.msg import Signals

from cv_bridge import CvBridge, CvBridgeError



class TrafficLightRoiVisualizer:
    def __init__(self):

        # Node parameters
        self.radius_to_roi_factor = rospy.get_param("/detection/tfl_camera/radius_to_roi_factor")

        # Publishers
        self.tfl_roi_pub = rospy.Publisher('tfl_roi_visualization', Image, queue_size=1)

        # Subscribers
        signals_sub = message_filters.Subscriber('signals', Signals)
        camera_img_sub = message_filters.Subscriber('/camera_fl/image_raw', Image)
        # TODO: add sub to results of the classifier

        # Synchronizer
        ts = message_filters.ApproximateTimeSynchronizer([signals_sub, camera_img_sub], queue_size = 10, slop = 0.2)
        ts.registerCallback(self.data_callback)


        self.bridge = CvBridge()

    def data_callback(self, signals_msg, camera_img_msg):

        try:
            image = self.bridge.imgmsg_to_cv2(camera_img_msg,  desired_encoding='rgb8')
        except CvBridgeError as e:
            rospy.logerr("traffic_light_roi_visualizer - ", e)

        if len(signals_msg.Signals) > 0:

            rois = convert_signals_to_rois(signals_msg.Signals, image.shape[1], image.shape[0], self.radius_to_roi_factor)
            for roi in rois:
                start_point = (int(roi[2]), int(roi[4]))    # top left
                end_point = (int(roi[3]), int(roi[5]))      # bottom right
                # TODO use classifier results to set color and label for roi
                color = (0, 255, 0)
                thickness = 2

                cv2.rectangle(image, start_point, end_point, color, thickness)

        try:
            self.tfl_roi_pub.publish(self.bridge.cv2_to_imgmsg(image, encoding='rgb8'))
        except CvBridgeError as e:
            rospy.logerr("traffic_light_roi_visualizer - ", e)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('traffic_light_roi_visualizer', log_level=rospy.INFO)
    node = TrafficLightRoiVisualizer()
    node.run()