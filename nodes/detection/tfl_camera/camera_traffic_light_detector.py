#!/usr/bin/env python3

import rospy
import numpy as np
import threading
import tf2_ros
import tf2_geometry_msgs

from sklearn.neighbors import NearestNeighbors
from helpers import get_distance_between_two_points
from image_geometry import PinholeCameraModel

from lanelet2.io import Origin, load
from lanelet2.projection import UtmProjector

from geometry_msgs.msg import PointStamped, Point
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from autoware_msgs.msg import TrafficLightResult, TrafficLightResultArray
from autoware_msgs.msg import Lane, Signals, ExtractedPosition

from cv_bridge import CvBridge, CvBridgeError


# TODO what are the lamp types? in extracted position msg?
LANELET2_STRING_TO_LAMP_TYPE = {
    "red": 1,
    "green": 2,
    "yellow": 3,
    "unknown": 4
}


class CameraTrafficLightDetector:
    def __init__(self):

        # Node parameters
        camera_info_topic = rospy.get_param('~camera_info_topic')
        camera_image_topic = rospy.get_param('~camera_image_topic')

        self.transform_from_frame = rospy.get_param('~transform_from_frame')
        self.transform_to_frame = rospy.get_param('~transform_to_frame')
        self.rectify_image = rospy.get_param('~rectify_image')
        self.nearest_neighbor_search = rospy.get_param("~nearest_neighbor_search")
        self.traffic_light_bulb_radius = rospy.get_param("~traffic_light_bulb_radius")
        self.output_roi_image = rospy.get_param("~output_roi_image")

        self.waypoint_interval = rospy.get_param("/planning/path_smoothing/waypoint_interval")
        coordinate_transformer = rospy.get_param("/localization/coordinate_transformer")
        use_custom_origin = rospy.get_param("/localization/use_custom_origin")
        utm_origin_lat = rospy.get_param("/localization/utm_origin_lat")
        utm_origin_lon = rospy.get_param("/localization/utm_origin_lon")
        lanelet2_map_name = rospy.get_param("~lanelet2_map_name")

        # internal variables
        self.lock = threading.Lock()
        self.current_pose = None

        # Load the map using Lanelet2
        if coordinate_transformer == "utm":
                projector = UtmProjector(Origin(utm_origin_lat, utm_origin_lon), use_custom_origin, False)
        else:
            rospy.logfatal("traffic_light_position_extractor - only utm currently supported for lanelet2 map loading")
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
        # TODO replace with classifier
        self.stopline_tree = NearestNeighbors(n_neighbors=1, algorithm=self.nearest_neighbor_search).fit(self.stopline_array[:,1:3])

        self.bridge = CvBridge()

        #self.model = self.load_model(self.onnx_path)

        # Publishers
        self.tfl_status_pub = rospy.Publisher('traffic_light_status', TrafficLightResultArray, queue_size=1)
        if self.output_roi_image:
            self.tfl_roi_pub = rospy.Publisher('traffic_light_roi_visualization', Image, queue_size=1)

        # Camera model
        camera_info = rospy.wait_for_message(camera_info_topic, CameraInfo, timeout=4)
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(camera_info)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Subscribers
        rospy.Subscriber('/planning/local_path', Lane, self.local_path_callback, queue_size=1)
        rospy.Subscriber(camera_image_topic, Image, self.camera_image_callback, queue_size=1)


    def local_path_callback(self, msg):

        signals_on_path = []
        current_pose = None

        if len(msg.waypoints) > 0:

            # Extract waypoints and create array - xy coordinates only
            waypoints_xy = np.array([(wp.pose.pose.position.x, wp.pose.pose.position.y) for wp in msg.waypoints])
            _, stopline_idx = self.stopline_tree.radius_neighbors(waypoints_xy, self.waypoint_interval / 2, return_distance=True)
            # flatten the stopline_idx and remove duplicates
            stopline_idx = np.unique(np.hstack(stopline_idx))
            stoplines_on_path = [int(self.stopline_array[idx][0]) for idx in stopline_idx]
            signals_on_path = list(filter(lambda signal: int(signal[0]) in stoplines_on_path, self.signal_array))

            current_pose = msg.waypoints[0].pose.pose

        with self.lock:
            self.signals_on_path = signals_on_path.copy()
            self.current_pose = current_pose


    def camera_image_callback(self, msg):

        with self.lock:
            signals_on_path = self.signals_on_path.copy()
            current_pose = self.current_pose

        image_time = msg.header.stamp
        # extract image
        try:
            image = self.bridge.imgmsg_to_cv2(msg,  desired_encoding='rgb8')
        except CvBridgeError as e:
            rospy.logerr("camera_traffic_light_detector - CvBridgeError: %s", e)
            return

        # extract transform
        try:
            t = self.tf_buffer.lookup_transform(self.transform_to_frame, self.transform_from_frame, image_time)
        except tf2_ros.TransformException as ex:
            rospy.logerr("camera_traffic_light_detector - Could not load transform from %s to %s: %s" % (self.transform_from_frame, self.transform_to_frame, ex))
            return
        
        if self.rectify_image:
            self.camera_model.rectifyImage(image, image)


        if len(signals_on_path) > 0 and current_pose is not None:

            print(signals_on_path)
            
            # transform signals to camera frame
             # iterate over signals on path
            for signal in signals_on_path:

                #print(signal)
                point_map = PointStamped()
                point_map.header.frame_id = "map"
                point_map.header.stamp = image_time
                point_map.point.x = float(signal[4])
                point_map.point.y = float(signal[5])
                point_map.point.z = float(signal[6])

                # transform point to camera frame
                point_camera = tf2_geometry_msgs.do_transform_point(point_map, t).point

                print(point_camera)
                point_image = self.camera_model.project3dToPixel((point_camera.x, point_camera.y, point_camera.z))
                print(point_image)

                




            # chekc if all 3 signals from tfl in image then create ROI

            # extract ROI from image

            # sed ROI to classifier

            # publish result

        if self.output_roi_image:
            try:
                img_msg = self.bridge.cv2_to_imgmsg(image, encoding='rgb8')
            except CvBridgeError as e:
                rospy.logerr("camera_traffic_light_detector - ", e)
            
            img_msg.header.stamp = image_time
            self.tfl_roi_pub.publish(img_msg)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('camera_traffic_light_detector', log_level=rospy.INFO)
    node = CameraTrafficLightDetector()
    node.run()