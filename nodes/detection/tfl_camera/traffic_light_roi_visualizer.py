#!/usr/bin/env python3

import rospy
import message_filters
import cv2

from camera_tfl_helpers import convert_signals_to_rois

from sensor_msgs.msg import Image
from autoware_msgs.msg import Signals
from autoware_msgs.msg import TrafficLightResultArray

from cv_bridge import CvBridge, CvBridgeError

RECKOGNITION_RESULT_STR_TO_COLOR = {
    "red": (255, 0, 0),
    "yellow": (255, 255, 0),
    "green": (0, 255, 0),
    "unkown": (0, 0, 0)
}

class TrafficLightRoiVisualizer:
    def __init__(self):

        # Node parameters
        self.radius_to_roi_factor = rospy.get_param("/detection/tfl_camera/radius_to_roi_factor")

        # Publishers
        self.tfl_roi_pub = rospy.Publisher('tfl_roi_visualization', Image, queue_size=1)

        # Subscribers
        signals_sub = message_filters.Subscriber('signals', Signals)
        camera_img_sub = message_filters.Subscriber('/camera_fl/image_raw', Image)
        classifier_sub = message_filters.Subscriber('traffic_light_status', TrafficLightResultArray)
        # TODO: add sub to results of the classifier

        # Synchronizer
        ts = message_filters.ApproximateTimeSynchronizer([signals_sub, camera_img_sub, classifier_sub], queue_size = 10, slop = 0.2)
        ts.registerCallback(self.data_callback)


        self.bridge = CvBridge()

    def data_callback(self, signals_msg, camera_img_msg, classifier_msg):

        try:
            image = self.bridge.imgmsg_to_cv2(camera_img_msg,  desired_encoding='rgb8')
        except CvBridgeError as e:
            rospy.logerr("traffic_light_roi_visualizer - ", e)

        # read in classifier results and put into dictionary
        classifier_results = {}
        for result in classifier_msg.results:
            classifier_results[result.light_id] = result.recognition_result_str

        if len(signals_msg.Signals) > 0:

            rois = convert_signals_to_rois(signals_msg.Signals, image.shape[1], image.shape[0], self.radius_to_roi_factor)
            for roi in rois:
                start_point = (int(roi[2]), int(roi[4]))    # top left
                end_point = (int(roi[3]), int(roi[5]))      # bottom right
                # TODO use classifier results to set color and label for roi
                color = RECKOGNITION_RESULT_STR_TO_COLOR[classifier_results[roi[0]]]

                cv2.rectangle(image, start_point, end_point, color, thickness = 3)

                cv2.putText(image,
                            classifier_results[roi[0]],
                            org = (int(roi[2]) + 5, int(roi[5]) - 5),
                            fontFace = cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale = 1,
                            color = color, 
                            thickness = 2)

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