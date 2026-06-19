#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import threading
import tf2_ros

import image_geometry

from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from autoware_mini.msg import LocalPath, StopLineStatus, StopLineStatusArray, Waypoint

import cv_bridge

from autoware_mini.transform import transform_point
from autoware_mini.lanelet2 import get_traffic_light_stop_lines, get_traffic_light_bboxes, load_lanelet2_map
from autoware_mini.detection import calculate_iou
from autoware_mini.yolo import Yolo11Model
from autoware_mini.path import PathWrapper

CLASS_TO_STRING = {
    0: "green",         1: "yellow",          2: "red",             3: "unknown",
    4: "green-left",    5: "green-right",     6: "green-straight",
    7: "yellow-left",   8: "yellow-right",    9: "yellow-straight",
    10: "red-left",     11: "red-right",      12: "red-straight",
}

CLASS_TO_TURN = {
    0: None,         1: None,          2: None,             3: None,
    4: Waypoint.TURN_LEFT,      5: Waypoint.TURN_RIGHT,       6: Waypoint.TURN_STRAIGHT,
    7: Waypoint.TURN_LEFT,      8: Waypoint.TURN_RIGHT,       9: Waypoint.TURN_STRAIGHT,
    10: Waypoint.TURN_LEFT,     11: Waypoint.TURN_RIGHT,      12: Waypoint.TURN_STRAIGHT,
}

BASE_CLASS_TO_COLOR = {
    0: (0,255,0),
    1: (255,255,0),
    2: (255,0,0),
    3: (0,0,0)
}

BASE_CLASS_TO_LIGHT_COLOR = {
    0: (204,255,153),
    1: (255,255,153),
    2: (255,153,153),
    3: (192,192,192)
}

def _base_class(cls):
    """Maps detailed class (e.g. green-left=4) to base class (e.g. green=0)."""
    return (cls - 4) // 3 if cls >= 4 else cls

BASE_CLASS_TO_TLRESULT = {
    0: StopLineStatus.STATUS_GO,      # GREEN
    1: StopLineStatus.STATUS_STOP,    # YELLOW
    2: StopLineStatus.STATUS_STOP,    # RED
    3: StopLineStatus.STATUS_UNKNOWN  # UNKNOWN
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
        enable_fp16 = rospy.get_param("~enable_fp16")
        self.turn_distance_limit = rospy.get_param("~turn_distance_limit")

        # Extract all stop lines and traffic lights from the lanelet2 map
        lanelet2_map = load_lanelet2_map(lanelet2_map_path)
        stop_lines = get_traffic_light_stop_lines(lanelet2_map)
        self.traffic_lights = get_traffic_light_bboxes(lanelet2_map)
        self.stop_lines = np.array(list(stop_lines.values()))
        self.stop_line_ids = np.array(list(stop_lines.keys()))

        self.bridge = cv_bridge.CvBridge()
        self.yolo_model = Yolo11Model(onnx_path, enable_fp16=enable_fp16)

        # Publishers
        self.tfl_status_pub = rospy.Publisher('traffic_light_status', StopLineStatusArray, queue_size=1, tcp_nodelay=True)
        self.tfl_roi_pub = rospy.Publisher('traffic_light_roi', Image, queue_size=1, tcp_nodelay=True)

        # Camera model
        self.camera_model = None
        rospy.Subscriber('camera_info', CameraInfo, self.camera_info_callback, queue_size=1, tcp_nodelay=True)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.stop_line_ids_on_path = None
        self.local_path = None
        self.lock = threading.Lock()

        # Subscribers
        rospy.Subscriber('/planning/local_path', LocalPath, self.local_path_callback, queue_size=1, buff_size=2**20, tcp_nodelay=True)
        rospy.Subscriber('image_raw', Image, self.camera_image_callback, queue_size=1, buff_size=2**26, tcp_nodelay=True)

    def camera_info_callback(self, camera_info_msg):
        if self.camera_model is None:
            self.camera_model = image_geometry.PinholeCameraModel()
            self.camera_model.fromCameraInfo(camera_info_msg)

    def local_path_callback(self, local_path_msg):

        # used in calculate_roi_coordinates to filter out only relevant traffic lights
        stop_line_ids_on_path = []
        local_path = None

        # if there is a local path collect allt the stop line id's on the path
        if local_path_msg.waypoints:
            local_path = PathWrapper(local_path_msg.waypoints)
            mask = local_path.linestring.intersects(self.stop_lines)
            stop_line_ids_on_path = self.stop_line_ids[mask].tolist()

        with self.lock:
            self.stop_line_ids_on_path = stop_line_ids_on_path
            self.transform_from_frame = local_path_msg.header.frame_id
            self.local_path = local_path

    def camera_image_callback(self, camera_image_msg):

        if self.camera_model is None:
            rospy.logwarn_throttle(10, "%s - No camera model received, skipping image", rospy.get_name())
            return

        with self.lock:
            if self.stop_line_ids_on_path is None or self.local_path is None:
                rospy.logwarn_throttle(10, "%s - No path received, skipping image", rospy.get_name())
                return
            stop_line_ids_on_path = self.stop_line_ids_on_path
            transform_from_frame = self.transform_from_frame
            local_path = self.local_path

        image_time_stamp = camera_image_msg.header.stamp  - rospy.Duration.from_sec(self.camera_delay_compensation)
        transform_to_frame = camera_image_msg.header.frame_id

        tfl_status = StopLineStatusArray()
        tfl_status.type = StopLineStatusArray.TRAFFIC_LIGHT
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

        if stop_line_ids_on_path:
            # extract transform
            try:
                transform = self.tf_buffer.lookup_transform(transform_to_frame, transform_from_frame, image_time_stamp, rospy.Duration(self.transform_timeout))
            except (tf2_ros.TransformException, rospy.ROSTimeMovedBackwardsException) as e:
                rospy.logwarn("%s - %s", rospy.get_name(), e)
                return

            map_rois = self.calculate_roi_coordinates(stop_line_ids_on_path, transform)

            if map_rois:
                # get yolo predictions
                yolo_rois, classes, scores = self.yolo_model.predict(image)

                # determine lane turn direction based on waypoint turn signals up to turn_distance_limit distance along the path
                turn = Waypoint.TURN_STRAIGHT
                path_idx = local_path.get_waypoint_index_at_distance(self.turn_distance_limit)
                turn_signals = local_path.turn_signals[:path_idx]
                turn_signals_mask = np.isin(turn_signals, [Waypoint.TURN_LEFT, Waypoint.TURN_RIGHT])

                if np.any(turn_signals_mask):
                    turn = turn_signals[np.argmax(turn_signals_mask)]

                # match yolo predictions with map ROIs
                tfl_results, match_dict = self.match_map_and_yolo_rois(map_rois, yolo_rois, classes, scores, turn)
                tfl_status.statuses.extend(tfl_results)

        self.tfl_status_pub.publish(tfl_status)

        if self.tfl_roi_pub.get_num_connections() > 0:
            self.publish_roi_images(image, map_rois, yolo_rois, match_dict, image_time_stamp)

    def calculate_roi_coordinates(self, stop_line_ids_on_path, transform):
        rois = []

        for stop_line_id in stop_line_ids_on_path:
            for traffic_light_id, traffic_light_coords in self.traffic_lights[stop_line_id].items():
                us = []
                vs = []

                for x, y, z in traffic_light_coords:
                    point_map = Point(x=x, y=y, z=z)

                    # transform point to camera frame and then to image frame
                    point_camera = transform_point(transform, point_map)
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

                rois.append([stop_line_id, traffic_light_id, min_u, max_u, min_v, max_v])

        return rois
    
    def match_map_and_yolo_rois(self, map_rois, yolo_rois, yolo_classes, yolo_scores, turn):
        tfl_results = []
        match_dict = {}

        # for every map roi
        for stop_line_id, traffic_light_id, x1_map, x2_map, y1_map, y2_map in map_rois:
            iou_max = -float('inf')
            best_priority = -1
            matched_roi = None

            # for every yolo class and box
            for idx, cls, score, yolo_roi in zip(range(len(yolo_rois)), yolo_classes, yolo_scores, yolo_rois):
                iou_score = calculate_iou(np.array([[x1_map, y1_map, x2_map, y2_map]]), yolo_roi[np.newaxis, :])[0][0]

                if iou_score <= self.iou_threshold:
                    continue
                
                # Determine priority of the match based on the class and turn signal. If the class label corresponds to the turn signal it gets highest priority, 
                # if the class is generic label it gets medium priority and if the class label contradicts the turn signal it gets lowest priority.
                cls_turn = CLASS_TO_TURN[cls]
                if cls_turn == turn:
                    priority = 2
                elif cls_turn is None:
                    priority = 1
                else:
                    priority = 0

                # prefer higher priority match, break ties by higher IOU score
                if (priority, iou_score) > (best_priority, iou_max):
                    matched_roi = [cls, score, yolo_roi, idx]
                    iou_max = iou_score
                    best_priority = priority

            tfl_result = StopLineStatus()
            tfl_result.traffic_light_id = traffic_light_id
            tfl_result.stop_line_id = stop_line_id

            if matched_roi is None:
                # no match for map ROI - traffic light status is missing
                tfl_result.status = StopLineStatus.STATUS_MISSING
                tfl_result.status_text = "missing"
                match_dict[traffic_light_id] = None
            else:
                # yolo ROI and map ROI were matched
                base_class = _base_class(matched_roi[0])
                tfl_result.status = BASE_CLASS_TO_TLRESULT[base_class]
                tfl_result.status_text = CLASS_TO_STRING[matched_roi[0]]
                match_dict[traffic_light_id] = matched_roi

            tfl_results.append(tfl_result)

        return tfl_results, match_dict

    def publish_roi_images(self, image, map_rois, yolo_rois, match_dict, image_time_stamp):
        # add rois to image
        if map_rois:
            matched_yolo_roi_idxs = []

            for _, traffic_light_id, min_u, max_u, min_v, max_v in map_rois:
                if match_dict[traffic_light_id] is None:
                    # map roi was not matched with any yolo roi
                    text_string = "%s %.2f" % ("missing", 0)
                    color = (200,200,200)

                else:
                    # map roi was matched with a yolo roi
                    cl, score, yolo_roi, yolo_idx = match_dict[traffic_light_id]

                    yolo_min_u, yolo_min_v, yolo_max_u, yolo_max_v = yolo_roi
                    matched_yolo_roi_idxs.append(yolo_idx)
                    
                    base_class = _base_class(cl)

                    # add smaller yolo roi
                    yolo_start_point = (yolo_min_u, yolo_min_v)
                    yolo_end_point = (yolo_max_u, yolo_max_v)
                    cv2.rectangle(image, yolo_start_point, yolo_end_point, color=BASE_CLASS_TO_LIGHT_COLOR[base_class], thickness=2)

                    text_string = "%s %.2f" % (CLASS_TO_STRING[cl], score)
                    color = BASE_CLASS_TO_COLOR[base_class]

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