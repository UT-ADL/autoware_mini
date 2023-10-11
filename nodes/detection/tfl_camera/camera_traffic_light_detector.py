#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import threading
import tf2_ros
import onnxruntime

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

        self.rectify_image = rospy.get_param('~rectify_image')
        self.traffic_light_bulb_radius = rospy.get_param("~traffic_light_bulb_radius")
        self.radius_to_roi_multiplier = rospy.get_param("~radius_to_roi_multiplier")
        self.min_roi_width = rospy.get_param("~min_roi_width")
        self.transform_timeout = rospy.get_param("~transform_timeout")
        self.waypoint_interval = rospy.get_param("/planning/waypoint_interval")
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
        lanelet2_map = load(lanelet2_map_name, projector)

        # Extract all stop lines and signals from the map
        self.signals = self.get_signals_from_map(lanelet2_map)

        self.bridge = CvBridge()
        self.model = onnxruntime.InferenceSession(onnx_path, providers=['CUDAExecutionProvider'])

        # Publishers
        self.tfl_status_pub = rospy.Publisher('traffic_light_status', TrafficLightResultArray, queue_size=1, tcp_nodelay=True)
        self.tfl_roi_pub = rospy.Publisher('traffic_light_roi', Image, queue_size=1, tcp_nodelay=True)

        # Camera model
        self.camera_model = None
        rospy.Subscriber('camera_info', CameraInfo, self.camera_info_callback, queue_size=1, tcp_nodelay=True)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Subscribers
        self.stoplines_on_path = None
        self.lock = threading.Lock()
        rospy.Subscriber('/planning/local_path', Lane, self.local_path_callback, queue_size=1, buff_size=2**20, tcp_nodelay=True)
        rospy.Subscriber('image_raw', Image, self.camera_image_callback, queue_size=1, buff_size=2**26, tcp_nodelay=True)

    def camera_info_callback(self, camera_info_msg):
        if self.camera_model is None:
            camera_model = PinholeCameraModel()
            camera_model.fromCameraInfo(camera_info_msg)
            # assignment is atomic in python
            self.camera_model = camera_model

    def local_path_callback(self, local_path_msg):

        stopline_idx = []

        # If there is a local path collect allt the stop line id's on the path
        if len(local_path_msg.waypoints) > 0:
            for wp in local_path_msg.waypoints:
                if wp.stop_line_id > 0:
                    # check if there is a signal available for that stop line
                    if wp.stop_line_id in self.signals:
                        stopline_idx.append(wp.stop_line_id)

        with self.lock:
            self.stoplines_on_path = stopline_idx
            self.transform_from_frame = local_path_msg.header.frame_id

    def camera_image_callback(self, camera_image_msg):

        if self.camera_model is None:
            rospy.logwarn_throttle(10, "%s - No camera model received, skipping image", rospy.get_name())
            return

        with self.lock:
            if self.stoplines_on_path is None:
                rospy.logwarn_throttle(10, "%s - No path received, skipping image", rospy.get_name())
                return
            stoplines_on_path = self.stoplines_on_path
            transform_from_frame = self.transform_from_frame

        image_time_stamp = camera_image_msg.header.stamp
        transform_to_frame = camera_image_msg.header.frame_id

        tfl_status = TrafficLightResultArray()
        tfl_status.header.stamp = image_time_stamp

        rois = []
        classes = []
        scores = []

        # extract image
        image = self.bridge.imgmsg_to_cv2(camera_image_msg,  desired_encoding='rgb8')

        # rectify image
        if self.rectify_image:
            self.camera_model.rectifyImage(image, image)

        if len(stoplines_on_path) > 0:

            # extract transform
            try:
                transform = self.tf_buffer.lookup_transform(transform_to_frame, transform_from_frame, image_time_stamp, rospy.Duration(self.transform_timeout))
            except (tf2_ros.TransformException, rospy.ROSTimeMovedBackwardsException) as e:
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

        if self.tfl_roi_pub.get_num_connections() > 0:
            self.publish_roi_images(image, rois, classes, scores, image_time_stamp)
    
    def get_signals_from_map(self, lanelet2_map):

        signals = {}

        for reg_el in lanelet2_map.regulatoryElementLayer:
            if reg_el.attributes["subtype"] == "traffic_light":
                # ref_line is the stop line and there is only 1 stopline per traffic light reg_el
                linkId = reg_el.parameters["ref_line"][0].id

                for bulbs in reg_el.parameters["light_bulbs"]:
                    # plId represents the traffic light (pole), one stop line can be associated with multiple traffic lights
                    plId = bulbs.id
                    # one traffic light has red, yellow and green bulbs
                    bulb_data = [[bulb.id, bulb.attributes["color"], bulb.x, bulb.y, bulb.z] for bulb in bulbs]
                    # signals is a dictionary indexed by stopline id and contains dictionary of traffic lights indexed by pole id
                    # which in turn contains a list of bulbs
                    signals.setdefault(linkId, {}).setdefault(plId, []).extend(bulb_data)

        return signals

    def calculate_roi_coordinates(self, stoplines_on_path, transform):

        rois = []

        for linkId in stoplines_on_path:
            for plId, bulbs in self.signals[linkId].items():

                us = []
                vs = []
                for _, _, x, y, z in bulbs:
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
                us = np.clip(np.round(np.array(us)), 0, self.camera_model.width - 1)
                vs = np.clip(np.round(np.array(vs)), 0, self.camera_model.height - 1)
                # extract one roi per traffic light
                min_u = int(np.min(us))
                max_u = int(np.max(us))
                min_v = int(np.min(vs))
                max_v = int(np.max(vs))
                # check if roi is too small
                if max_u - min_u < self.min_roi_width:
                    continue
                rois.append([int(linkId), plId, min_u, max_u, min_v, max_v])

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

                text_string = "%s %.2f" % (CLASSIFIER_RESULT_TO_STRING[cl], score)
                text_width, text_height = cv2.getTextSize(text_string, cv2.FONT_HERSHEY_SIMPLEX, 1.5, 2)[0]
                text_orig_u = int(min_u + (max_u - min_u) / 2 - text_width / 2)
                text_orig_v = max_v + text_height + 3

                start_point = (min_u, min_v)
                end_point = (max_u, max_v)
                cv2.rectangle(image, start_point, end_point, color=CLASSIFIER_RESULT_TO_COLOR[cl] , thickness=3)
                cv2.putText(image,
                    text_string,
                    org=(text_orig_u, text_orig_v),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=1.5,
                    color=CLASSIFIER_RESULT_TO_COLOR[cl], 
                    thickness=2)

        image = cv2.resize(image, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_LINEAR)
        img_msg = self.bridge.cv2_to_imgmsg(image, encoding='rgb8')
        
        img_msg.header.stamp = image_time_stamp
        self.tfl_roi_pub.publish(img_msg)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('camera_traffic_light_detector', log_level=rospy.INFO)
    node = CameraTrafficLightDetector()
    node.run()