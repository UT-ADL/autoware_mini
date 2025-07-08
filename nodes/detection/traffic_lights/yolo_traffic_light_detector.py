#!/usr/bin/env python3

import rospy
import numpy as np
import shapely
import cv2
import threading
import tf2_ros

from image_geometry import PinholeCameraModel

from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from autoware_mini.msg import Path, TrafficLightResult, TrafficLightResultArray

from cv_bridge import CvBridge

from autoware_mini.transform import transform_point
from autoware_mini.lanelet2 import get_traffic_light_stop_lines, get_stoplines_trafficlights, load_lanelet2_map
from autoware_mini.detection import calculate_iou
from autoware_mini.yolo import YoloModel

# Classifier outputs 4 classes (LightState)
CLASSIFIER_RESULT_TO_STRING = {
    0: "red",
    1: "green",
    2: "yellow",
    3: "unknown"
}

CLASSIFIER_RESULT_TO_COLOR = {
    0: (255,0,0),
    1: (0,255,0),
    2: (255,255,0),
    3: (0,0,0)
}

CLASSIFIER_RESULT_TO_LIGHT_COLOR = {
    0: (255,153,153),
    1: (204,255,153),
    2: (255,255,153),
    3: (192,192,192)
}

CLASSIFIER_RESULT_TO_TLRESULT = {
    0: 0,   # RED
    1: 1,   # GREEN
    2: 0,   # YELLOW
    3: 2    # UNKNOWN
}

class YoloTrafficLightDetector:
    def __init__(self):

        # Node parameters
        onnx_path = rospy.get_param("~onnx_path")

        self.rectify_image = rospy.get_param('~rectify_image')
        self.roi_width_extent = rospy.get_param("~roi_width_extent")
        self.roi_height_extent = rospy.get_param("~roi_height_extent")
        self.min_roi_width = rospy.get_param("~min_roi_width")
        self.transform_timeout = rospy.get_param("~transform_timeout")
        lanelet2_map_path = rospy.get_param("~lanelet2_map_path")
        self.iou_threshold = rospy.get_param("~iou_threshold")
        self.camera_delay_compensation = rospy.get_param("~camera_delay_compensation")

        # Extract all stop lines and traffic lights from the lanelet2 map
        lanelet2_map = load_lanelet2_map(lanelet2_map_path)
        self.stoplines = get_traffic_light_stop_lines(lanelet2_map)
        self.trafficlights = get_stoplines_trafficlights(lanelet2_map)

        # Remove stoplines that have no traffic lights. If stopline_id is not in self.trafficlights then it has no traffic lights
        self.stoplines = {k: v for k, v in self.stoplines.items() if k in self.trafficlights}

        self.bridge = CvBridge()
        self.yolo_model = YoloModel(onnx_path)

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
        rospy.Subscriber('/planning/local_path', Path, self.local_path_callback, queue_size=1, buff_size=2**20, tcp_nodelay=True)
        rospy.Subscriber('image_raw', Image, self.camera_image_callback, queue_size=1, buff_size=2**26, tcp_nodelay=True)

    def camera_info_callback(self, camera_info_msg):
        if self.camera_model is None:
            self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(camera_info_msg)

    def local_path_callback(self, local_path_msg):

        # used in calculate_roi_coordinates to filter out only relevant traffic lights
        stoplines_on_path = []

        # if there is a local path collect allt the stop line id's on the path
        if len(local_path_msg.waypoints) > 0:
            local_path = shapely.LineString([(wp.position.x, wp.position.y) for wp in local_path_msg.waypoints])

            for linkId, stopline in self.stoplines.items():
                # check if stopline intersects with local path
                if local_path.intersects(stopline):
                    stoplines_on_path.append(linkId)

        with self.lock:
            self.stoplines_on_path = stoplines_on_path
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

        image_time_stamp = camera_image_msg.header.stamp  - rospy.Duration.from_sec(self.camera_delay_compensation)
        transform_to_frame = camera_image_msg.header.frame_id

        tfl_status = TrafficLightResultArray()
        tfl_status.header.stamp = image_time_stamp

        map_rois = []

        yolo_rois = []
        classes = []
        scores = []

        match_dict = {}

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

            map_rois = self.calculate_roi_coordinates(stoplines_on_path, transform)

            if len(map_rois) > 0:
                # get yolo predictions
                yolo_rois, classes, scores = self.yolo_model.predict(image)

                # match yolo predictions with map ROIs
                tfl_results, match_dict = self.match_map_and_yolo_rois(map_rois, yolo_rois, classes, scores)
                tfl_status.results.extend(tfl_results)

        self.tfl_status_pub.publish(tfl_status)

        if self.tfl_roi_pub.get_num_connections() > 0:
            self.publish_roi_images(image, map_rois, yolo_rois, match_dict, image_time_stamp)

    def calculate_roi_coordinates(self, stoplines_on_path, transform):
        rois = []

        for linkId in stoplines_on_path:
            for plId, traffic_lights in self.trafficlights[linkId].items():
                us = []
                vs = []

                for x, y, z in traffic_lights.values():
                    point_map = Point(float(x), float(y), float(z))

                    # transform point to camera frame and then to image frame
                    point_camera = transform_point(point_map, transform)
                    u, v = self.camera_model.project3dToPixel((point_camera.x, point_camera.y, point_camera.z))

                    # check with image limits using the camera model and points's z coordinate w.r.t camera
                    if u < 0 or u >= self.camera_model.width or v < 0 or v >= self.camera_model.height or point_camera.z < 0:
                        break
                    
                    # convert the extent in meters to extent in pixels
                    extent_x_px = self.camera_model.fx() * self.roi_width_extent / point_camera.z
                    extent_y_px = self.camera_model.fy() * self.roi_height_extent / point_camera.z

                    us.extend([u + extent_x_px, u - extent_x_px])
                    vs.extend([v + extent_y_px, v - extent_y_px])

                # not all traffic lights were in image, take next traffic light
                if len(us) < 8:
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
    
    def match_map_and_yolo_rois(self, map_rois, yolo_rois, yolo_classes, yolo_scores):
        tfl_results = []
        match_dict = {}
        
        # for every map roi
        for linkId, plId, x1_map, x2_map, y1_map, y2_map in map_rois:
            iou_max = -2
            matched_roi = None

            # for every yolo class and box
            for idx, cls, score, yolo_roi in zip(range(len(yolo_rois)), yolo_classes, yolo_scores, yolo_rois):
                iou_score = calculate_iou(np.array([[x1_map, y1_map, x2_map, y2_map]]), yolo_roi[np.newaxis, :])[0][0]
                # if iou over threshold use max iou for association
                if iou_score > self.iou_threshold:
                    if iou_score > iou_max:
                        matched_roi = [cls, score, yolo_roi, idx]
                        iou_max = iou_score
                else:
                    continue

            tfl_result = TrafficLightResult()
            tfl_result.light_id = plId
            tfl_result.stopline_id = linkId

            if matched_roi is None:
                # no match for map ROI - traffic light status is unknown
                tfl_result.recognition_result = 2
                tfl_result.recognition_result_str = "unknown"
                match_dict[plId] = None
            else:
                # yolo ROI and map ROI were matched
                tfl_result.recognition_result = CLASSIFIER_RESULT_TO_TLRESULT[matched_roi[0]]
                tfl_result.recognition_result_str = CLASSIFIER_RESULT_TO_STRING[matched_roi[0]]
                match_dict[plId] = matched_roi

            tfl_results.append(tfl_result)

        return tfl_results, match_dict

    def publish_roi_images(self, image, map_rois, yolo_rois, match_dict, image_time_stamp):
        # add rois to image
        if len(map_rois) > 0:
            matched_yolo_roi_idxs = []

            for _, plId, min_u, max_u, min_v, max_v in map_rois:
                if match_dict[plId] is None:
                    # map roi was not matched with any yolo roi
                    text_string = "%s %.2f" % ("unknown", 0)
                    color = (0,0,0)

                else:
                    # map roi was matched with a yolo roi
                    cl, score, yolo_roi, yolo_idx = match_dict[plId]

                    yolo_min_u, yolo_min_v, yolo_max_u, yolo_max_v = yolo_roi
                    matched_yolo_roi_idxs.append(yolo_idx)
                    
                    # add smaller yolo roi
                    yolo_start_point = (yolo_min_u, yolo_min_v)
                    yolo_end_point = (yolo_max_u, yolo_max_v)
                    cv2.rectangle(image, yolo_start_point, yolo_end_point, color=CLASSIFIER_RESULT_TO_LIGHT_COLOR[cl], thickness=2)

                    text_string = "%s %.2f" % (CLASSIFIER_RESULT_TO_STRING[cl], score)
                    color = CLASSIFIER_RESULT_TO_COLOR[cl]

                #add bigger map roi
                text_width, text_height = cv2.getTextSize(text_string, cv2.FONT_HERSHEY_SIMPLEX, 1.5, 2)[0]
                text_orig_u = int(min_u + (max_u - min_u) / 2 - text_width / 2)
                text_orig_v = max_v + text_height + 3

                start_point = (min_u, min_v)
                end_point = (max_u, max_v)
                cv2.rectangle(image, start_point, end_point, color=color, thickness=3)
                cv2.putText(image,
                    text_string,
                    org=(text_orig_u, text_orig_v),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=1.5,
                    color=color, 
                    thickness=2)
        
            # add all yolo ROIs that were not matched to any map ROI
            for i, (yolo_min_u, yolo_min_v, yolo_max_u, yolo_max_v) in enumerate(yolo_rois):
                if i not in matched_yolo_roi_idxs:
                    yolo_start_point = (yolo_min_u, yolo_min_v)
                    yolo_end_point = (yolo_max_u, yolo_max_v)
                    cv2.rectangle(image, yolo_start_point, yolo_end_point, color=(0, 0, 255) , thickness=2)

        image = cv2.resize(image, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_LINEAR)
        img_msg = self.bridge.cv2_to_imgmsg(image, encoding='rgb8')
        
        img_msg.header.stamp = image_time_stamp
        self.tfl_roi_pub.publish(img_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('yolo_traffic_light_detector', log_level=rospy.INFO)
    node = YoloTrafficLightDetector()
    node.run()