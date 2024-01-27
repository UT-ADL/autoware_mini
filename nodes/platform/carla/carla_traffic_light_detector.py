#!/usr/bin/env python3

import rospy
import numpy as np
from lanelet2.io import Origin, load
from lanelet2.projection import UtmProjector

from sklearn.neighbors import RadiusNeighborsClassifier

from carla_msgs.msg import CarlaTrafficLightStatus, CarlaTrafficLightStatusList, CarlaTrafficLightInfoList
from autoware_msgs.msg import TrafficLightResult, TrafficLightResultArray

from tf.transformations import quaternion_matrix

from localization.SimulationToUTMTransformer import SimulationToUTMTransformer

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

        # Create a classifier to find the closest stopline and its corresponding traffic light on map
        stopline_coords, stopline_ids = self.extractStopLines(lanelet2_map)
        # Trigger volumes are quite far apart from stoplines in Carla Towns so use variable radius for classifier
        self.classifier = RadiusNeighborsClassifier(radius=4 if self.use_offset else 15)
        if len(stopline_coords) > 0:
            assert len(stopline_coords) == len(stopline_ids)
            self.classifier.fit(stopline_coords, stopline_ids)
        self.stopline_id_to_coords_map = {}

        # Publishers
        self.tfl_status_pub = rospy.Publisher('traffic_light_status', TrafficLightResultArray, queue_size=1, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber('/carla/traffic_lights/info',
                         CarlaTrafficLightInfoList, self.tfl_info_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/carla/traffic_lights/status',
                    CarlaTrafficLightStatusList, self.tfl_status_callback, queue_size=1, tcp_nodelay=True)

    def extractStopLines(self, lanelet2_map):
        stopline_coords = []
        stopline_ids = []
        for reg_el in lanelet2_map.regulatoryElementLayer:
            if reg_el.attributes["subtype"] == "traffic_light":
                for tfl in reg_el.parameters["light_bulbs"]:
                    line_points = []
                    for point in reg_el.parameters["ref_line"][0]:
                        line_points.append((point.x, point.y))
                    
                    # Extract center point from stopline
                    center_point = np.mean(line_points, axis=0)

                    stopline_coords.append((center_point[0], center_point[1]))
                    # light_id refers to the traffic light, lane_id refers to the stop line
                    stopline_ids.append((tfl.id, reg_el.parameters["ref_line"][0].id))

                    break
                        
        return stopline_coords, stopline_ids

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
            
            # Use trigger volume's location for stopline association
            self.stopline_id_to_coords_map[tfl.id] = (pose.position.x, pose.position.y)

    def tfl_status_callback(self, msg):
        """
        callback CarlaTrafficLightStatusList
        """
        tfl_status = TrafficLightResultArray()
        tfl_status.header.stamp = rospy.Time.now()
        stopline_association_dict = {}

        for light in msg.traffic_lights:
            if light.id not in self.stopline_id_to_coords_map:
                rospy.logwarn("%s - traffic light %d not found in info", rospy.get_name(), light.id)
                continue
            stopline_coords = self.stopline_id_to_coords_map[light.id]

            try:
                light_id, lane_id = self.classifier.predict([stopline_coords])[0]

                if light_id not in stopline_association_dict:
                    stopline_association_dict[light_id] = [np.linalg.norm(np.array(stopline_coords)), lane_id, light.state]
                elif np.linalg.norm(np.array(stopline_coords)) <= stopline_association_dict[light_id][0]:
                    stopline_association_dict[light_id] = [np.linalg.norm(np.array(stopline_coords)), lane_id, light.state]
                else:
                    rospy.logdebug("%s - traffic light %d at coordinates (%f, %f) is further away than the one already associated with stopline %d", rospy.get_name(), light.id, stopline_coords[0], stopline_coords[1], stopline_association_dict[light_id][1])
                    continue

            except ValueError:
                rospy.logdebug("%s - traffic light %d at coordinates (%f, %f) not found in map", rospy.get_name(), light.id, stopline_coords[0], stopline_coords[1])
                continue

        for light_id, (_, lane_id, state) in stopline_association_dict.items():
            tfl_result = TrafficLightResult()
            tfl_result.light_id = light_id
            tfl_result.lane_id = lane_id
            tfl_result.recognition_result = CARLA_TO_AUTOWARE_TFL_MAP[state]
            tfl_result.recognition_result_str = CARLA_TO_AUTOWARE_TFL_STR[state]
            tfl_status.results.append(tfl_result)
        
        self.tfl_status_pub.publish(tfl_status)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('carla_traffic_light_detector', log_level=rospy.INFO)
    node = CarlaTrafficLightDetector()
    node.run()
