#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import tf2_ros
import tf2_geometry_msgs
import onnxruntime
import message_filters

from sklearn.neighbors import NearestNeighbors
from image_geometry import PinholeCameraModel

from lanelet2.io import Origin, load
from lanelet2.projection import UtmProjector

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from autoware_msgs.msg import TrafficLightResult, TrafficLightResultArray
from autoware_msgs.msg import Lane

from cv_bridge import CvBridge, CvBridgeError


# TODO - review the classifier class codes
CLASSIFIER_RESULT_TO_STRING = {
    0: "red",
    3: "yellow",      # not sure ? - Almost never outputted?????
    1: "green",
    2: "unkown"
}

CLASSIFIER_RESULT_TO_COLOR = {
    0: (255,0,0),
    3: (255,255,0),      # not sure ? - Almost never outputted?????
    1: (0,255,0),
    2: (0,0,0)
}

CLASSIFIER_RESULT_TO_TLRESULT = {
    0: 0,   # 0 RED
    3: 0,   # 0 YELLOW
    1: 1,   # 1 GREEN
    2: 2    # 2 UNKNOWN
}

class CameraTrafficLightDetector:
    def __init__(self):

        # Node parameters
        onnx_path = rospy.get_param("~onnx_path")

        self.output_roi_image = rospy.get_param("~output_roi_image")
        self.rectify_image = rospy.get_param('~rectify_image')
        self.nearest_neighbor_search = rospy.get_param("~nearest_neighbor_search")
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
        stopline = []
        signal = []

        for reg_el in self.lanelet2_map.regulatoryElementLayer:
            if reg_el.attributes["subtype"] == "traffic_light":
                
                # ref_line is the stop line and there is only 1 stopline per traffic light reg_el
                linkId = reg_el.parameters["ref_line"][0].id

                # add stoplines to list
                for point in reg_el.parameters["ref_line"][0]:
                    stopline.append([linkId, point.x, point.y, point.z])

                # add signals to list
                for bulbs in reg_el.parameters["light_bulbs"]:
                    # bulb group is used for the signal group and as plId (pole id) - one traffic light
                    plId = bulbs.id
                    for bulb in bulbs:
                        #              linkId, plId, signalId, type,                    map_x,  map_y,  map_z
                        signal.append([linkId, plId, bulb.id, bulb.attributes["color"], bulb.x, bulb.y, bulb.z])
        
        self.stopline_array = np.array(stopline)
        self.signal_array = np.array(signal)

        # Spatial index for nearest neighbor search - xy coordinates only
        self.stopline_tree = NearestNeighbors(n_neighbors=1, algorithm=self.nearest_neighbor_search).fit(self.stopline_array[:,1:3])

        self.bridge = CvBridge()
        self.model = onnxruntime.InferenceSession(onnx_path)

        # Publishers
        self.tfl_status_pub = rospy.Publisher('traffic_light_status', TrafficLightResultArray, queue_size=1)
        if self.output_roi_image:
            self.tfl_roi_pub = rospy.Publisher('traffic_light_roi', Image, queue_size=1)

        # Camera model
        camera_info = rospy.wait_for_message('camera_info', CameraInfo, timeout=4)
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(camera_info)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Subscribers
        local_paths_sub = message_filters.Subscriber('/planning/local_path', Lane)
        camera_image_sub = message_filters.Subscriber('image_raw', Image)
        self.ts = message_filters.ApproximateTimeSynchronizer([local_paths_sub, camera_image_sub], 5, 0.07)
        self.ts.registerCallback(self.local_path_camera_image_callback)

    def local_path_camera_image_callback(self, local_path_msg, camera_image_msg):

        image_time_stamp = camera_image_msg.header.stamp
        transform_to_frame = camera_image_msg.header.frame_id
        transform_from_frame = local_path_msg.header.frame_id

        tfl_status = TrafficLightResultArray()
        tfl_status.header.stamp = image_time_stamp

        traffic_lights = {}
        rois = []
        prediction = []

        if len(local_path_msg.waypoints) > 0:
            traffic_lights = self.get_traffic_lights_on_path(local_path_msg.waypoints)

        # extract image
        try:
            image = self.bridge.imgmsg_to_cv2(camera_image_msg,  desired_encoding='rgb8')
        except CvBridgeError as e:
            rospy.logerr("%s - CvBridgeError: %s", rospy.get_name(), str(e))
            return

        if self.rectify_image:
            self.camera_model.rectifyImage(image, image)

        if len(traffic_lights) > 0:

            # extract transform
            try:
                transform = self.tf_buffer.lookup_transform(transform_to_frame, transform_from_frame, image_time_stamp)
            except tf2_ros.TransformException as ex:
                rospy.logerr("%s - Could not load transform from %s to %s: %s", rospy.get_name(), transform_from_frame, transform_to_frame, str(ex))
                return

            rois = self.calculate_roi_coordinates(traffic_lights, transform)
            roi_images = self.create_roi_images(image, rois)

            # run model and do prediction
            prediction = self.model.run(None, {'conv2d_1_input': roi_images})[0]

            if len(prediction) != len(rois):
                rospy.logerr("%s - Number of predictions (%d) does not match number of rois (%d)", rospy.get_name(), len(prediction[0]), len(rois))
                return

            # extract results in sync with rois
            for pred, (linkId, plId, _, _, _, _) in zip(prediction, rois):
                result = np.argmax(pred)

                tfl_result = TrafficLightResult()
                tfl_result.light_id = plId
                tfl_result.lane_id = linkId
                tfl_result.recognition_result = CLASSIFIER_RESULT_TO_TLRESULT[result]
                tfl_result.recognition_result_str = CLASSIFIER_RESULT_TO_STRING[result]

                tfl_status.results.append(tfl_result)

        self.tfl_status_pub.publish(tfl_status)

        if self.output_roi_image:
            self.publish_roi_images(image, rois, prediction, image_time_stamp)


    def get_traffic_lights_on_path(self, waypoints):
        
        traffic_lights = {}
        
        # Extract waypoints and create array - xy coordinates only
        waypoints_xy = np.array([(wp.pose.pose.position.x, wp.pose.pose.position.y) for wp in waypoints])
        _, stopline_idx = self.stopline_tree.radius_neighbors(waypoints_xy, self.waypoint_interval / 2, return_distance=True)
        # flatten the stopline_idx and remove duplicates
        stopline_idx = np.unique(np.hstack(stopline_idx))
        stoplines_on_path = [int(self.stopline_array[idx][0]) for idx in stopline_idx]
        signals_on_path = list(filter(lambda signal: int(signal[0]) in stoplines_on_path, self.signal_array))

        # reorg signals (individual bulbs) into traffic lights dict
        for signal in signals_on_path:
            plId = int(signal[1])
            if plId in traffic_lights:
                traffic_lights[plId].append(signal)
            else:
                traffic_lights[plId] = [signal]

        return traffic_lights

    def calculate_roi_coordinates(self, traffic_lights, transform):

        rois = []

        for plId, signals in traffic_lights.items():
            u = []
            v = []
            for linkId, _, _, _, x, y, z in signals:
                point_map = PointStamped()
                point_map.point.x = float(x)
                point_map.point.y = float(y)
                point_map.point.z = float(z)

                # transform point to camera frame and then to image frame
                point_camera = tf2_geometry_msgs.do_transform_point(point_map, transform).point
                point_image = self.camera_model.project3dToPixel((point_camera.x, point_camera.y, point_camera.z))

                # check with image limits using the camera model
                if point_image[0] < 0 or point_image[0] >= self.camera_model.width or point_image[1] < 0 or point_image[1] >= self.camera_model.height:
                    break

                # calculate radius of the bulb in pixels
                d = np.linalg.norm([point_camera.x, point_camera.y, point_camera.z])
                radius = self.camera_model.fx() * self.traffic_light_bulb_radius / d

                # calc extent for every signal then generate roi using min/max and rounding
                u += ([point_image[0] + radius * self.radius_to_roi_multiplier, point_image[0] - radius * self.radius_to_roi_multiplier])
                v += ([point_image[1] + radius * self.radius_to_roi_multiplier, point_image[1] - radius * self.radius_to_roi_multiplier])

            # not all signals were in image, take next traffic light
            if len(u) < 6:
                continue
            # round and clip against image limits
            u = np.clip(np.round(np.array(u)), 0, self.camera_model.width)
            v = np.clip(np.round(np.array(v)), 0, self.camera_model.height)
            # extract one roi per traffic light
            rois.append([int(linkId), plId, int(np.min(u)), int(np.max(u)), int(np.min(v)), int(np.max(v))])
        
        return rois

    def create_roi_images(self, image, rois):

            roi_images = []

            for _, _, min_u, max_u, min_v, max_v in rois:

                roi_image = image[min_v:max_v, min_u:max_u, :]
                roi_image = self.process_image(roi_image)
                roi_images.append(roi_image)

            return np.stack(roi_images, axis=0)

    def process_image(self, image):
        image = cv2.resize(image, (128, 128), interpolation=cv2.INTER_LINEAR)
        # convert image float and normalize
        image = image.astype(np.float32) / 255.0

        return image

    def publish_roi_images(self, image, rois, prediction, image_time_stamp):
        
        # add rois to image
        if len(rois) > 0:
            for pred, (_, _, min_u, max_u, min_v, max_v) in zip(prediction, rois):
                result = np.argmax(pred)
                
                start_point = (min_u, min_v)
                end_point = (max_u, max_v)
                cv2.rectangle(image, start_point, end_point, CLASSIFIER_RESULT_TO_COLOR[result] , thickness = 3)
                cv2.putText(image,
                    CLASSIFIER_RESULT_TO_STRING[result] + " " + str(pred[result])[:4],
                    org = (min_u + 5, max_v - 5),
                    fontFace = cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale = 1,
                    color = CLASSIFIER_RESULT_TO_COLOR[result], 
                    thickness = 2)

        try:
            img_msg = self.bridge.cv2_to_imgmsg(image, encoding='rgb8')
        except CvBridgeError as e:
            rospy.logerr("%s - CVBridge cant't convert image to message: %s", rospy.get_name(), str(e))
        
        img_msg.header.stamp = image_time_stamp
        self.tfl_roi_pub.publish(img_msg)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('camera_traffic_light_detector', log_level=rospy.INFO)
    node = CameraTrafficLightDetector()
    node.run()