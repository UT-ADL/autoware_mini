#!/usr/bin/env python3

import rospy
import message_filters
import cv2
import onnxruntime
import ros_numpy
import numpy as np

import matplotlib.pyplot as plt

from camera_tfl_helpers import convert_signals_to_rois

from sensor_msgs.msg import Image, CameraInfo
from autoware_msgs.msg import Signals

from cv_bridge import CvBridge, CvBridgeError



class TrafficLightClassifier:
    def __init__(self):

        # Node parameters
        self.radius_to_roi_factor = rospy.get_param("/detection/tfl_camera/radius_to_roi_factor")
        self.onnx_path = rospy.get_param("/detection/tfl_camera/onnx_path", "/home/edgar/workspaces/autoware_mini_ws/src/autoware_mini/config/traffic_lights/tlr_model.onnx")

        # Publishers

        # Subscribers
        signals_sub = message_filters.Subscriber('signals', Signals)
        camera_img_sub = message_filters.Subscriber('/camera_fl/image_raw', Image)
        camera_info_sub = message_filters.Subscriber('/camera_fl/camera_info', CameraInfo)

        # Synchronizer
        ts = message_filters.ApproximateTimeSynchronizer([signals_sub, camera_img_sub, camera_info_sub], queue_size = 10, slop = 0.2)
        ts.registerCallback(self.data_callback)


        cv2.namedWindow('Image window', cv2.WINDOW_NORMAL)
        self.bridge = CvBridge()

        self.model = self.load_model(self.onnx_path)


    def load_model(self, onnx_path):
        return onnxruntime.InferenceSession(onnx_path)


    def data_callback(self, signals_msg, camera_img_msg, camera_info_msg):

        if len(signals_msg.Signals) > 0:

            rois = convert_signals_to_rois(signals_msg.Signals, camera_info_msg.width, camera_info_msg.height, self.radius_to_roi_factor)

            # TODO decide what to use numpify or cv_bridge
            try:
                image = self.bridge.imgmsg_to_cv2(camera_img_msg,  desired_encoding='rgb8')
            except CvBridgeError as e:
                rospy.logerr("traffic_light_position_visualizer - ", e)

            # img = ros_numpy.numpify(camera_img_msg)
            print(image.shape)
            # plt.imshow(img)
            # plt.show()

            for roi in rois:

                # # Numpy
                # roi_img = img[int(roi[4]):int(roi[5]), int(roi[2]):int(roi[3])]
                # print(roi_img.shape)
                # roi_img = self.process_image(roi_img)

                #                   start_row:end_row,       start_col:end_col
                roi_image = image[int(roi[4]):int(roi[5]), int(roi[2]):int(roi[3])]
                print("bef", roi_image.shape)
                roi_image = self.process_image(roi_image)
                print("aft", roi_image.shape)
                

                # plt.clf()
                # plt.imshow(roi_img)
                # plt.show()

                # cv2.imshow('Image window', roi_image)
                # cv2.waitKey(1)

                roi_image = np.expand_dims(roi_image, axis=0)
                print(roi_image.shape)
                output = self.model.run(None, {'conv2d_1_input': roi_image})
                print(output)
                

            #if roi_images 


    def process_image(self, image):
        #image = ros_numpy.numpify(image)
        image = cv2.resize(image, (128, 128), interpolation=cv2.INTER_LINEAR)
        print(image.shape)
        # convert image float and normalize
        image = image.astype(np.float32) / 255.0

        return image


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('traffic_light_classifier', log_level=rospy.INFO)
    node = TrafficLightClassifier()
    node.run()