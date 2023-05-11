#!/usr/bin/env python3

import rospy
import tf
import numpy as np
import math

from sklearn.neighbors import NearestNeighbors
from helpers.timer import Timer

from lanelet2.io import Origin, load
from lanelet2.projection import UtmProjector

from helpers import get_distance_between_two_points

from image_geometry import PinholeCameraModel

from geometry_msgs.msg import PoseStamped, PointStamped, Point
from sensor_msgs.msg import CameraInfo
from autoware_msgs.msg import Lane, Signals, ExtractedPosition


# TODO what are the lamp types? in extracted position msg?
LANELET2_STRING_TO_LAMP_TYPE = {
    "red": 1,
    "green": 2,
    "yellow": 3,
    "unknown": 4
}


class TrafficLightPositionExtractor:
    def __init__(self):

        # Node parameters
        camera_info_topic = rospy.get_param('~camera_info_topic')

        self.nearest_neighbor_search = rospy.get_param("~nearest_neighbor_search")
        self.traffic_light_bulb_radius = rospy.get_param("~traffic_light_bulb_radius")

        self.waypoint_interval = rospy.get_param("/planning/path_smoothing/waypoint_interval")
        
        coordinate_transformer = rospy.get_param("/localization/coordinate_transformer")
        use_custom_origin = rospy.get_param("/localization/use_custom_origin")
        utm_origin_lat = rospy.get_param("/localization/utm_origin_lat")
        utm_origin_lon = rospy.get_param("/localization/utm_origin_lon")
        lanelet2_map_name = rospy.get_param("~lanelet2_map_name")
        
        self.image_time = None

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
        self.stopline_tree = NearestNeighbors(n_neighbors=1, algorithm=self.nearest_neighbor_search).fit(self.stopline_array[:,1:3])
        
        self.camera_model = PinholeCameraModel()
        self.tf_listener = tf.TransformListener()

        # Publishers
        self.signals_pub = rospy.Publisher('signals', Signals, queue_size=1)

        # Subscribers
        rospy.Subscriber(camera_info_topic, CameraInfo, self.camera_info_callback, queue_size=1)
        rospy.Subscriber('/planning/local_path', Lane, self.local_path_callback)

    def local_path_callback(self, msg):
        
        t2 = Timer()
        
        signals_on_image = []

        if len(msg.waypoints) > 0:

            # Extract waypoints and create array - xy coordinates only
            waypoints_xy = np.array([(wp.pose.pose.position.x, wp.pose.pose.position.y) for wp in msg.waypoints])
            _, stopline_idx = self.stopline_tree.radius_neighbors(waypoints_xy, self.waypoint_interval / 2, return_distance=True)
            # flatten the stopline_idx and remove duplicates
            stopline_idx = np.unique(np.hstack(stopline_idx))
            stoplines_on_path = [int(self.stopline_array[idx][0]) for idx in stopline_idx]
            signals_on_path = list(filter(lambda signal: int(signal[0]) in stoplines_on_path, self.signal_array))
            t2("signals_on_path")

            # TODO - currently approximation - 2m behind the cameras
            current_position = Point(x = waypoints_xy[0][0], y = waypoints_xy[0][1], z = 0.0)

            # iterate over signals on path
            for signal in signals_on_path:

                #print(signal)
                point_map = PointStamped()
                point_map.header.frame_id = "map"
                point_map.point.x = float(signal[4])
                point_map.point.y = float(signal[5])
                point_map.point.z = float(signal[6])

                # transform bulb to camera frame
                # TODO: self.image_time ? use image timestamp
                self.tf_listener.lookupTransform("map", "camera_fl", rospy.Time(0))
                point_camera = self.tf_listener.transformPoint("camera_fl", point_map).point
                point_image = self.camera_model.project3dToPixel((point_camera.x, point_camera.y, point_camera.z))

                # check with image limits using the camera model
                if point_image[0] < 0 or self.camera_model.width < point_image[0]:
                    continue
                if point_image[1] < 0 or self.camera_model.height < point_image[1]:
                    continue

                # calculate radius of the bulb in pixels
                d = get_distance_between_two_points(current_position, point_map.point)
                radius = self.camera_model.fx() * self.traffic_light_bulb_radius / d

                extracted_position = ExtractedPosition()
                extracted_position.signalId = int(signal[2])
                extracted_position.u = int(round(point_image[0]))
                extracted_position.v = int(round(point_image[1]))
                # TODO - radius
                extracted_position.radius = math.ceil(radius)
                extracted_position.x = float(signal[4])
                extracted_position.y = float(signal[5])
                extracted_position.z = float(signal[6])
                # extracted_position.hang = 0
                extracted_position.type = LANELET2_STRING_TO_LAMP_TYPE[signal[3]]
                extracted_position.linkId = int(signal[0])
                extracted_position.plId = int(signal[1])

                signals_on_image.append(extracted_position)

        # publish signals
        signals = Signals()
        # TODO: write image timestamp?
        #signals.header.stamp = rospy.Time.now()
        signals.header.frame_id = "camera_fl"
        signals.Signals = signals_on_image
        self.signals_pub.publish(signals)


        t2("local_path_callback - end")
        print(t2)


    def camera_info_callback(self, msg):
        self.camera_model.fromCameraInfo(msg)
        self.image_time = msg.header.stamp


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('traffic_light_position_extractor', log_level=rospy.INFO)
    node = TrafficLightPositionExtractor()
    node.run()