#!/usr/bin/env python3

import rospy
import numpy as np
from lanelet2.io import Origin, load
from lanelet2.projection import UtmProjector

from sklearn.neighbors import KNeighborsClassifier

from carla_msgs.msg import CarlaTrafficLightStatus, CarlaTrafficLightStatusList, CarlaTrafficLightInfoList
from autoware_msgs.msg import TrafficLightResult, TrafficLightResultArray

from tf.transformations import quaternion_matrix

from localization.SimulationToUTMTransformer import SimulationToUTMTransformer
from helpers.lanelet2 import get_stoplines_center

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

        # Load the map using Lanelet2
        if coordinate_transformer == "utm":
                projector = UtmProjector(Origin(utm_origin_lat, utm_origin_lon), use_custom_origin, False)
        else:
            rospy.logfatal("%s - only utm and custom origin currently supported for lanelet2 map loading", rospy.get_name())
            exit(1)
        lanelet2_map = load(lanelet2_map_name, projector)

        # Coordinate transformer from simulation coordinates to UTM
        self.sim2utm_transformer = SimulationToUTMTransformer(use_custom_origin=use_custom_origin,
                                                              origin_lat=utm_origin_lat,
                                                              origin_lon=utm_origin_lon)
        
        # Extract stopline data (coords, light_id, lane_id) from the map
        self.stopline_data = []
        self.extractStopLines(lanelet2_map)

        # Carla tfl_id to lanelet2 stopline_id mapping
        self.light_id_to_stopline_id_map = None

        self.stopline_centers_map = get_stoplines_center(lanelet2_map)

        stopline_centers = np.array(list(self.stopline_centers_map.values()))[:, 0].tolist()
        stopline_ids = [(item,) for item in self.stopline_centers_map.keys()]

        # Classifier to find the closest stopline and its corresponding traffic light on map
        self.classifier = KNeighborsClassifier(n_neighbors=1)
        self.classifier.fit(stopline_centers, stopline_ids)

        # Carla tfl_id to lanelet2 stopline_id mapping
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
        
        for tfl in msg.traffic_lights:
            pose = tfl.transform
            if self.use_offset:
                pose = self.sim2utm_transformer.transform_pose(pose)
            
            rotation_matrix = quaternion_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])

            # Transform center_location using the rotation matrix
            center_rotated = np.dot(rotation_matrix, np.array([tfl.trigger_volume.center.x, tfl.trigger_volume.center.y, tfl.trigger_volume.center.z, 1]))

            # Compute the position of the trigger volume based on the center location and the pose
            pose.position.x += center_rotated[0]
            pose.position.y += center_rotated[1]
            pose.position.z += center_rotated[2]

            try:
                stopline_id = self.classifier.predict([(pose.position.x, pose.position.y)])[0]
            except ValueError:
                rospy.logdebug("%s - stopline at coordinates (%f, %f) near (traffic light: %s) not found in map", rospy.get_name(), pose.position.x, pose.position.y, tfl.id)
                continue

            self.light_id_to_stopline_id_map[tfl.id] = stopline_id

    def tfl_status_callback(self, msg):
        """
        callback CarlaTrafficLightStatusList
        """
            
        tfl_status = TrafficLightResultArray()
        tfl_status.header.stamp = rospy.Time.now()

        stopline_state = {}

        for light in msg.traffic_lights:

            if light.id not in self.light_id_to_stopline_id_map:
                rospy.logwarn_throttle(10, "%s - traffic light %d not found in info", rospy.get_name(), light.id)
                continue

            stopline_id = self.light_id_to_stopline_id_map[light.id]

            if stopline_id not in stopline_state:
                stopline_state[stopline_id] = light.state   
            elif stopline_state[stopline_id] != light.state:
                rospy.logwarn_throttle(10, "%s - multiple traffic lights for the same stopline %d with different states", rospy.get_name(), stopline_id)
                continue

            # Add same traffic light status to a stopline if there exists multiple traffic lights for the same stopline
            for light_id in self.stopline_centers_map[stopline_id][1]:
                
                tfl_result = TrafficLightResult()
                tfl_result.light_id = light_id
                tfl_result.lane_id = stopline_id
                tfl_result.recognition_result = CARLA_TO_AUTOWARE_TFL_MAP[stopline_state[stopline_id]]
                tfl_result.recognition_result_str = CARLA_TO_AUTOWARE_TFL_STR[stopline_state[stopline_id]]
                tfl_status.results.append(tfl_result)
        
        self.tfl_status_pub.publish(tfl_status)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('carla_traffic_light_detector', log_level=rospy.ERROR)
    node = CarlaTrafficLightDetector()
    node.run()
