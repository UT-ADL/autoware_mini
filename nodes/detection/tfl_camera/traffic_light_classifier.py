#!/usr/bin/env python3

import rospy
import message_filters
import cv2

from camera_tfl_helpers import convert_signals_to_rois

from sensor_msgs.msg import CompressedImage, CameraInfo
from autoware_msgs.msg import Signals

from cv_bridge import CvBridge, CvBridgeError



class TrafficLightClassifier:
    def __init__(self):

        # Node parameters
        self.radius_to_roi_factor = rospy.get_param("/detection/tfl_camera/radius_to_roi_factor")

        # Publishers

        # Subscribers
        signals_sub = message_filters.Subscriber('signals', Signals)
        camera_img_sub = message_filters.Subscriber('/camera_fl/image_raw/compressed', CompressedImage)
        camera_info_sub = message_filters.Subscriber('/camera_fl/camera_info', CameraInfo)

        # Synchronizer
        ts = message_filters.ApproximateTimeSynchronizer([signals_sub, camera_img_sub, camera_info_sub], queue_size = 10, slop = 0.2)
        ts.registerCallback(self.data_callback)


        cv2.namedWindow('Image window', cv2.WINDOW_NORMAL)
        self.bridge = CvBridge()

    def data_callback(self, signals_msg, camera_img_msg, camera_info_msg):
        print("traffic_light_classifier - data_callback")
        rospy.loginfo("traffic_light_classifier - data_callback")

        if len(signals_msg.Signals) > 0:

            rois = convert_signals_to_rois(signals_msg.Signals, camera_info_msg.width, camera_info_msg.height, self.radius_to_roi_factor)

            # Extract roi from image
            # Send to classifier
            # Publish result to TrafficLightResultArray


            try:
                image = self.bridge.compressed_imgmsg_to_cv2(camera_img_msg,  desired_encoding='rgb8')
            except CvBridgeError as e:
                rospy.logerr("traffic_light_position_visualizer - ", e)

            for roi in rois:
                start_point = (int(roi[2]), int(roi[4]))
                end_point = (int(roi[3]), int(roi[5]))
                color = (0, 255, 0)
                thickness = 2
                image = cv2.rectangle(image, start_point, end_point, color, thickness)


            # cv2.imshow('Image window', image)
            # cv2.waitKey(1)



    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('traffic_light_classifier', log_level=rospy.INFO)
    node = TrafficLightClassifier()
    node.run()