#!/usr/bin/env python3

import rospy
import numpy as np

from sklearn.neighbors import KNeighborsClassifier

from carla_msgs.msg import CarlaTrafficLightStatus, CarlaTrafficLightStatusList, CarlaTrafficLightInfoList
from autoware_msgs.msg import TrafficLightResult, TrafficLightResultArray

import ros_numpy

from localization.SimulationToUTMTransformer import SimulationToUTMTransformer
from helpers.lanelet2 import get_stoplines_center, load_lanelet2_map

# Carla to Autoware traffic light status mapping
CARLA_TO_AUTOWARE_TFL_MAP = {
    CarlaTrafficLightStatus.RED: 0,
    CarlaTrafficLightStatus.YELLOW: 0,
    CarlaTrafficLightStatus.GREEN: 1,
    CarlaTrafficLightStatus.OFF: 1,
    CarlaTrafficLightStatus.UNKNOWN: 2
}

CARLA_TO_AUTOWARE_TFL_STR = {
    CarlaTrafficLightStatus.RED: "RED",
    CarlaTrafficLightStatus.YELLOW: "YELLOW",
    CarlaTrafficLightStatus.GREEN: "GREEN",
    CarlaTrafficLightStatus.OFF: "OFF",
    CarlaTrafficLightStatus.UNKNOWN: "UNKNOWN"
}

class CarlaTrafficLightDetector:
    def __init__(self):

        # Node parameters
        self.use_offset = rospy.get_param("/carla/use_offset")
        coordinate_transformer = rospy.get_param("/localization/coordinate_transformer")
        use_custom_origin = rospy.get_param("/localization/use_custom_origin")
        utm_origin_lat = rospy.get_param("/localization/utm_origin_lat")
        utm_origin_lon = rospy.get_param("/localization/utm_origin_lon")
        lanelet2_map_name = rospy.get_param("~lanelet2_map_name")

        # Load lanelet2 map
        lanelet2_map = load_lanelet2_map(lanelet2_map_name, coordinate_transformer, use_custom_origin, utm_origin_lat, utm_origin_lon)

        # Coordinate transformer from simulation coordinates to UTM
        self.sim2utm_transformer = SimulationToUTMTransformer(use_custom_origin=use_custom_origin,
                                                              origin_lat=utm_origin_lat,
                                                              origin_lon=utm_origin_lon)
        
        # Extract stopline data (coords, light_id, lane_id) from the map
        self.stopline_data = []
        self.extractStopLines(lanelet2_map)

        # Carla tfl_id to lanelet2 stopline_id mapping
        self.light_id_to_stopline_id_map = None

        # Get stopline centers with stopline_id and corresponding light_ids mapping
        self.stopline_centers_map = get_stoplines_center(lanelet2_map)

        # Carla_light_id to stopline_id mapping
        self.light_id_to_stopline_id_map = {}

        # Publishers
        self.tfl_status_pub = rospy.Publisher('traffic_light_status', TrafficLightResultArray, queue_size=1, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber('/carla/traffic_lights/info',
                         CarlaTrafficLightInfoList, self.tfl_info_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/carla/traffic_lights/status',
                    CarlaTrafficLightStatusList, self.tfl_status_callback, queue_size=1, tcp_nodelay=True)

    def tfl_info_callback(self, msg):
        """
        callback CarlaTrafficLightInfoList
        """
        
        trigger_volume_coords = []
        light_ids = []

        for tfl in msg.traffic_lights:
            pose = tfl.transform
            if self.use_offset:
                pose = self.sim2utm_transformer.transform_pose(pose)

            # Transform trigger volume location using the transformation matrix from tfl pose
            trans_matrix = ros_numpy.numpify(pose)
            trigger_coords = np.array([tfl.trigger_volume.center.x, tfl.trigger_volume.center.y, tfl.trigger_volume.center.z, 1])
            center_x, center_y, _, _ = np.dot(trans_matrix, trigger_coords)

            trigger_volume_coords.append((center_x, center_y))
            light_ids.append((tfl.id))

        # Initialize classifier to predict the closest trigger volume
        classifier = KNeighborsClassifier(n_neighbors=1)
        classifier.fit(trigger_volume_coords, light_ids)
        
        # Predict closest trigger volume to stopline center and create carla_light_id to stopline_id mapping
        for stopline_id, ((center_x, center_y), lanelet_light_ids) in self.stopline_centers_map.items():
            try:
                carla_light_id = classifier.predict([(center_x, center_y)])[0]
                self.light_id_to_stopline_id_map[carla_light_id] = (stopline_id, lanelet_light_ids)
            except:
                rospy.logwarn_throttle(10, "%s Unable to find nearest traffic light trigger volume for stopline %d", rospy.get_name(), stopline_id)


    def tfl_status_callback(self, msg):
        """
        callback CarlaTrafficLightStatusList
        """
            
        tfl_status = TrafficLightResultArray()
        tfl_status.header.stamp = rospy.Time.now()

        for light in msg.traffic_lights:

            if light.id not in self.light_id_to_stopline_id_map:
                rospy.logwarn_throttle(10, "%s - traffic light %d not found in info", rospy.get_name(), light.id)
                continue

            stopline_id, lanelet_light_ids = self.light_id_to_stopline_id_map[light.id]

            for light_id in lanelet_light_ids:

                tfl_result = TrafficLightResult()
                tfl_result.light_id = light_id
                tfl_result.lane_id = stopline_id
                tfl_result.recognition_result = CARLA_TO_AUTOWARE_TFL_MAP[light.state]
                tfl_result.recognition_result_str = CARLA_TO_AUTOWARE_TFL_STR[light.state]
                tfl_status.results.append(tfl_result)
        
        self.tfl_status_pub.publish(tfl_status)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('carla_traffic_light_detector', log_level=rospy.INFO)
    node = CarlaTrafficLightDetector()
    node.run()
