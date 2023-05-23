#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import traceback
import tf2_ros
import onnxruntime
import message_filters

from sklearn.neighbors import RadiusNeighborsClassifier
import warnings
from image_geometry import PinholeCameraModel

from lanelet2.io import Origin, load
from lanelet2.projection import UtmProjector

from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from autoware_msgs.msg import TrafficLightResult, TrafficLightResultArray
from autoware_msgs.msg import Lane

from cv_bridge import CvBridge, CvBridgeError

from helpers.transform import transform_point

# Classifier outputs 4 classes (LightState)
CLASSIFIER_RESULT_TO_STRING = {
    0: "green",
    1: "yellow",
    2: "red",
    3: "unknown"
}

CLASSIFIER_RESULT_TO_COLOR = {
    0: (0,255,0),
    1: (255,255,0),
    2: (255,0,0),
    3: (0,0,0)
}

CLASSIFIER_RESULT_TO_TLRESULT = {
    0: 1,   # GREEN
    1: 0,   # YELLOW
    2: 0,   # RED
    3: 2    # UNKNOWN
}

class CameraTrafficLightDetector:
    def __init__(self):

        # Node parameters
        onnx_path = rospy.get_param("~onnx_path")

        self.output_roi_image = rospy.get_param("~output_roi_image")
        self.rectify_image = rospy.get_param('~rectify_image')
        self.traffic_light_bulb_radius = rospy.get_param("~traffic_light_bulb_radius")
        self.radius_to_roi_multiplier = rospy.get_param("~radius_to_roi_multiplier")
        self.waypoint_interval = rospy.get_param("/planning/path_smoothing/waypoint_interval")

        coordinate_transformer = rospy.get_param("/localization/coordinate_transformer")
        use_custom_origin = rospy.get_param("/localization/use_custom_origin")
        utm_origin_lat = rospy.get_param("/localization/utm_origin_lat")
        utm_origin_lon = rospy.get_param("/localization/utm_origin_lon")
        lanelet2_map_name = rospy.get_param("~lanelet2_map_name")


        # Load the map using Lanelet2
        if coordinate_transformer == "utm":
            projector = UtmProjector(Origin(utm_origin_lat, utm_origin_lon), use_custom_origin, False)
        else:
            rospy.logfatal("%s - only utm currently supported for lanelet2 map loading", rospy.get_name())
            exit(1)
        self.lanelet2_map = load(lanelet2_map_name, projector)

        # Etract all stop lines and signals from the map
        stoplines=[]
        self.traffic_lights = {}

        for reg_el in self.lanelet2_map.regulatoryElementLayer:
            if reg_el.attributes["subtype"] == "traffic_light":
                # ref_line is the stop line and there is only 1 stopline per traffic light reg_el
                linkId = reg_el.parameters["ref_line"][0].id
                stoplines.extend([[linkId, point.x, point.y] for point in reg_el.parameters["ref_line"][0]])

                for bulbs in reg_el.parameters["light_bulbs"]:
                    plId = bulbs.id
                    bulb_data = [[bulb.id, bulb.attributes["color"], bulb.x, bulb.y, bulb.z] for bulb in bulbs]
                    self.traffic_lights.setdefault(linkId, {}).setdefault(plId, []).extend(bulb_data)

        stoplines = np.array(stoplines)

        # Disable the warnings from sklearn
        warnings.filterwarnings("ignore", category=UserWarning, module="sklearn")
        # Create the classifier
        self.classifier = RadiusNeighborsClassifier(radius=self.waypoint_interval / 2, outlier_label=0).fit(stoplines[:,1:], stoplines[:,0])

        self.bridge = CvBridge()
        self.model = onnxruntime.InferenceSession(onnx_path, providers=['CUDAExecutionProvider'])

        # Publishers
        self.tfl_status_pub = rospy.Publisher('traffic_light_status', TrafficLightResultArray, queue_size=1)
        if self.output_roi_image:
            self.tfl_roi_pub = rospy.Publisher('traffic_light_roi', Image, queue_size=1)

        # Camera model
        self.camera_model = None
        rospy.Subscriber('camera_info', CameraInfo, self.camera_info_callback, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Subscribers
        local_path_sub = message_filters.Subscriber('/planning/local_path', Lane)
        camera_image_sub = message_filters.Subscriber('image_raw', Image)
        self.ts = message_filters.ApproximateTimeSynchronizer([local_path_sub, camera_image_sub], queue_size=5, slop=0.1)
        self.ts.registerCallback(self.local_path_camera_image_callback)

    def camera_info_callback(self, camera_info_msg):
        if self.camera_model is None:
            self.camera_model = PinholeCameraModel()
            self.camera_model.fromCameraInfo(camera_info_msg)

    def local_path_camera_image_callback(self, local_path_msg, camera_image_msg):

        try:
            if self.camera_model is None:
                rospy.logwarn_throttle(10, "%s - No camera model received, skipping image", rospy.get_name())
                return

            image_time_stamp = camera_image_msg.header.stamp
            transform_to_frame = camera_image_msg.header.frame_id
            transform_from_frame = local_path_msg.header.frame_id

            tfl_status = TrafficLightResultArray()
            tfl_status.header.stamp = image_time_stamp

            stoplines_on_path = []
            rois = []
            classes = []
            scores = []

            if len(local_path_msg.waypoints) > 0:
                stoplines_on_path = self.get_stoplines_on_path(local_path_msg.waypoints)

            # extract image
            try:
                image = self.bridge.imgmsg_to_cv2(camera_image_msg,  desired_encoding='rgb8')
            except CvBridgeError as e:
                rospy.logerr("%s - %s", rospy.get_name(), e)
                return

            if self.rectify_image:
                self.camera_model.rectifyImage(image, image)

            if len(stoplines_on_path) > 0:

                # extract transform
                try:
                    transform = self.tf_buffer.lookup_transform(transform_to_frame, transform_from_frame, image_time_stamp)
                except tf2_ros.TransformException as e:
                    rospy.logwarn("%s - %s", rospy.get_name(), e)
                    return

                rois = self.calculate_roi_coordinates(stoplines_on_path, transform)

                if len(rois) > 0:
                    roi_images = self.create_roi_images(image, rois)

                    # run model and do prediction
                    predictions = self.model.run(None, {'conv2d_1_input': roi_images})[0]

                    assert len(predictions) == len(rois), "Number of predictions (%d) does not match number of rois (%d)" % (len(predictions), len(rois))

                    classes = np.argmax(predictions, axis=1)
                    scores = np.max(predictions, axis=1)
                    
                    # extract results in sync with rois
                    for cl, (linkId, plId, _, _, _, _) in zip(classes, rois):

                        tfl_result = TrafficLightResult()
                        tfl_result.light_id = plId
                        tfl_result.lane_id = linkId
                        tfl_result.recognition_result = CLASSIFIER_RESULT_TO_TLRESULT[cl]
                        tfl_result.recognition_result_str = CLASSIFIER_RESULT_TO_STRING[cl]

                        tfl_status.results.append(tfl_result)

            self.tfl_status_pub.publish(tfl_status)

            if self.output_roi_image:
                self.publish_roi_images(image, rois, classes, scores, image_time_stamp)
                
        except Exception as e:
            rospy.logerr_throttle(10, "%s - Exception in callback: %s", rospy.get_name(), traceback.format_exc())


    def get_stoplines_on_path(self, waypoints):

        # Extract waypoints and create array - xy coordinates only
        waypoints_xy = np.array([(wp.pose.pose.position.x, wp.pose.pose.position.y) for wp in waypoints])
        stopline_idx = np.unique(self.classifier.predict(waypoints_xy))
        # remove zeros
        stopline_idx = stopline_idx[stopline_idx != 0]

        return stopline_idx

    def calculate_roi_coordinates(self, stoplines_on_path, transform):

        rois = []

        for linkId in stoplines_on_path:
            for plId, signals in self.traffic_lights[linkId].items():

                us = []
                vs = []
                for _, _, x, y, z in signals:
                    point_map = Point()
                    point_map.x = float(x)
                    point_map.y = float(y)
                    point_map.z = float(z)

                    # transform point to camera frame and then to image frame
                    point_camera = transform_point(point_map, transform)
                    u, v = self.camera_model.project3dToPixel((point_camera.x, point_camera.y, point_camera.z))

                    # check with image limits using the camera model
                    if u < 0 or u >= self.camera_model.width or v < 0 or v >= self.camera_model.height:
                        break

                    # calculate radius of the bulb in pixels
                    d = np.linalg.norm([point_camera.x, point_camera.y, point_camera.z])
                    radius = self.camera_model.fx() * self.traffic_light_bulb_radius / d

                    # calc extent for every signal then generate roi using min/max and rounding
                    extent = radius * self.radius_to_roi_multiplier
                    us.extend([u + extent, u - extent])
                    vs.extend([v + extent, v - extent])

                # not all signals were in image, take next traffic light
                if len(us) < 6:
                    continue
                # round and clip against image limits
                us = np.clip(np.round(np.array(us)), 0, self.camera_model.width)
                vs = np.clip(np.round(np.array(vs)), 0, self.camera_model.height)
                # extract one roi per traffic light
                rois.append([int(linkId), plId, int(np.min(us)), int(np.max(us)), int(np.min(vs)), int(np.max(vs))])

        return rois

    def create_roi_images(self, image, rois):

            roi_images = []

            for _, _, min_u, max_u, min_v, max_v in rois:

                roi_image = image[min_v:max_v, min_u:max_u, :]
                roi_image = cv2.resize(roi_image, (128, 128), interpolation=cv2.INTER_LINEAR)
                roi_images.append(roi_image.astype(np.float32))

            return np.stack(roi_images, axis=0) / 255.0


    def publish_roi_images(self, image, rois, classes, scores, image_time_stamp):
        
        # add rois to image
        if len(rois) > 0:
            for cl, score, (_, _, min_u, max_u, min_v, max_v) in zip(classes, scores, rois):
                
                text_orig_u = int(min_u + (max_u - min_u) / 2 - (cv2.getTextSize(CLASSIFIER_RESULT_TO_STRING[cl] + " " + str(score)[:4], cv2.FONT_HERSHEY_SIMPLEX, 1, 2))[0][0] / 2)

                start_point = (min_u, min_v)
                end_point = (max_u, max_v)
                cv2.rectangle(image, start_point, end_point, CLASSIFIER_RESULT_TO_COLOR[cl] , thickness = 3)
                cv2.putText(image,
                    CLASSIFIER_RESULT_TO_STRING[cl] + " " + str(score)[:4],
                    org = (text_orig_u, max_v + 24),
                    fontFace = cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale = 1,
                    color = CLASSIFIER_RESULT_TO_COLOR[cl], 
                    thickness = 2)

        try:
            img_msg = self.bridge.cv2_to_imgmsg(image, encoding='rgb8')
        except CvBridgeError as e:
            rospy.logerr("%s - %s", rospy.get_name(), e)
        
        img_msg.header.stamp = image_time_stamp
        self.tfl_roi_pub.publish(img_msg)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('camera_traffic_light_detector', log_level=rospy.INFO)
    node = CameraTrafficLightDetector()
    node.run()