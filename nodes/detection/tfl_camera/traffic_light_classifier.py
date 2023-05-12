#!/usr/bin/env python3

import rospy
import message_filters
import cv2
import onnxruntime
import numpy as np

from camera_tfl_helpers import convert_signals_to_rois

from sensor_msgs.msg import Image
from autoware_msgs.msg import Signals
from autoware_msgs.msg import TrafficLightResult, TrafficLightResultArray

from cv_bridge import CvBridge, CvBridgeError

# TODO - review the classifier class codes
CLASSIFIER_RESULT_TO_STRING = {
    0: "red",
    3: "yellow",      # not sure ? - Almost never outputted?????
    1: "green",
    2: "unkown"
}

class TrafficLightClassifier:
    def __init__(self):

        # Node parameters
        self.radius_to_roi_factor = rospy.get_param("/detection/tfl_camera/radius_to_roi_factor")
        self.onnx_path = rospy.get_param("/detection/tfl_camera/onnx_path", "/home/edgar/workspaces/autoware_mini_ws/src/autoware_mini/config/traffic_lights/tlr_model.onnx")

        # Publishers
        self.tfl_status_pub = rospy.Publisher('traffic_light_status', TrafficLightResultArray, queue_size=1)

        # Subscribers
        signals_sub = message_filters.Subscriber('signals', Signals)
        camera_img_sub = message_filters.Subscriber('/camera_fl/image_raw', Image)

        # Synchronizer
        ts = message_filters.ApproximateTimeSynchronizer([signals_sub, camera_img_sub], queue_size = 10, slop = 0.2)
        ts.registerCallback(self.data_callback)

        self.bridge = CvBridge()

        self.model = self.load_model(self.onnx_path)


    def load_model(self, onnx_path):
        return onnxruntime.InferenceSession(onnx_path)


    def data_callback(self, signals_msg, camera_img_msg):

        tfl_status = TrafficLightResultArray()
        tfl_status.header.stamp = camera_img_msg.header.stamp

        if len(signals_msg.Signals) > 0:

            try:
                image = self.bridge.imgmsg_to_cv2(camera_img_msg,  desired_encoding='rgb8')
            except CvBridgeError as e:
                rospy.logerr("traffic_light_position_visualizer - ", e)

            rois = convert_signals_to_rois(signals_msg.Signals, image.shape[1], image.shape[0], self.radius_to_roi_factor)

            roi_images = []
            for roi in rois:
                #                   start_row:end_row,       start_col:end_col
                roi_image = image[int(roi[4]):int(roi[5]), int(roi[2]):int(roi[3])]
                roi_image = self.process_image(roi_image)
                roi_images.append(roi_image)

            # if there only one roi, expand dims to make it a batch of 1 else stack them
            if len(roi_images) > 1:
                input = np.stack(roi_images, axis=0)
            else:
                input = np.expand_dims(roi_images[0], axis=0)

            # run model and do prediction
            prediction = self.model.run(None, {'conv2d_1_input': input})

            for i, pred in enumerate(prediction[0]):
                result = np.argmax(pred)

                tfl_result = TrafficLightResult()
                tfl_result.light_id = int(rois[i][0])
                tfl_result.lane_id = int(rois[i][1])
                tfl_result.recognition_result = result
                tfl_result.recognition_result_str = CLASSIFIER_RESULT_TO_STRING[result]

                tfl_status.results.append(tfl_result)

        self.tfl_status_pub.publish(tfl_status)


    def process_image(self, image):
        #image = ros_numpy.numpify(image)
        image = cv2.resize(image, (128, 128), interpolation=cv2.INTER_LINEAR)
        # convert image float and normalize
        image = image.astype(np.float32) / 255.0

        return image


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('traffic_light_classifier', log_level=rospy.INFO)
    node = TrafficLightClassifier()
    node.run()