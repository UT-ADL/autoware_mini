#!/usr/bin/env python3

import rospy
from lanelet2.io import Origin, load
from lanelet2.projection import UtmProjector

from sklearn.neighbors import RadiusNeighborsClassifier

from carla_msgs.msg import CarlaTrafficLightStatus, CarlaTrafficLightStatusList, CarlaTrafficLightInfoList
from autoware_msgs.msg import TrafficLightResult, TrafficLightResultArray
from visualization_msgs.msg import MarkerArray, Marker

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
    CarlaTrafficLightStatus.RED: "red",
    CarlaTrafficLightStatus.YELLOW: "yellow",
    CarlaTrafficLightStatus.GREEN: "green",
    CarlaTrafficLightStatus.OFF: "off",
    CarlaTrafficLightStatus.UNKNOWN: "unknown"
}

class CarlaTrafficLightDetector:
    def __init__(self):

        # Node parameters
        self.use_offset = rospy.get_param("~use_offset", default=True)
        coordinate_transformer = rospy.get_param("/localization/coordinate_transformer")
        use_custom_origin = rospy.get_param("/localization/use_custom_origin", True)
        utm_origin_lat = rospy.get_param("/localization/utm_origin_lat")
        utm_origin_lon = rospy.get_param("/localization/utm_origin_lon")
        lanelet2_map_name = rospy.get_param("~lanelet2_map_name")

        # Load the map using Lanelet2
        if coordinate_transformer == "utm":
                projector = UtmProjector(Origin(utm_origin_lat, utm_origin_lon), use_custom_origin, False)
        else:
            rospy.logfatal("lanelet2_global_planner - only utm and custom origin currently supported for lanelet2 map loading")
            exit(1)
        lanelet2_map = load(lanelet2_map_name, projector)

        # Coordinate transformer from simulation coordinates to UTM
        self.sim2utm_transformer = SimulationToUTMTransformer(use_custom_origin=use_custom_origin,
                                                              origin_lat=utm_origin_lat,
                                                              origin_lon=utm_origin_lon)

        # Create a classifier to find the closest traffic light on map
        tfl_coords, tfl_ids = self.extractTrafficLights(lanelet2_map)
        self.classifier = RadiusNeighborsClassifier(radius=1.0).fit(tfl_coords, tfl_ids)
        self.tlf_id_to_coords_map = {}

        # Publishers
        self.tfl_status_pub = rospy.Publisher('traffic_light_status', TrafficLightResultArray, queue_size=1)

        # Subscribers
        rospy.Subscriber('/carla/traffic_lights/info',
                         CarlaTrafficLightInfoList, self.tfl_info_callback, queue_size=1)
        rospy.Subscriber('/carla/traffic_lights/status',
                    CarlaTrafficLightStatusList, self.tfl_status_callback, queue_size=1)

    def extractTrafficLights(self, lanelet2_map):
        tfl_coords = []
        tfl_ids = []
        for reg_el in lanelet2_map.regulatoryElementLayer:
            if reg_el.attributes["subtype"] == "traffic_light":
                for tfl in reg_el.parameters["light_bulbs"]:
                    for bulb in tfl:
                        tfl_coords.append((bulb.x, bulb.y))
                        # light_id refers to the traffic light, lane_id refers to the stop line
                        tfl_ids.append((tfl.id, reg_el.parameters["ref_line"][0].id))
                        # use the parameters of the first bulb
                        break
        return tfl_coords, tfl_ids

    def tfl_info_callback(self, msg):
        """
        callback CarlaTrafficLightInfoList
        """
        for tfl in msg.traffic_lights:
            pose = tfl.transform
            if self.use_offset:
                pose = self.sim2utm_transformer.transform_pose(pose)
            self.tlf_id_to_coords_map[tfl.id] = (pose.position.x, pose.position.y)

    def tfl_status_callback(self, msg):
        """
        callback CarlaTrafficLightStatusList
        """
        tfl_status = TrafficLightResultArray()
        tfl_status.header.stamp = rospy.Time.now()
        for light in msg.traffic_lights:
            if light.id not in self.tlf_id_to_coords_map:
                rospy.logwarn("Traffic light %d not found in info", light.id)
                continue
            tfl_coords = self.tlf_id_to_coords_map[light.id]

            try:
                light_id, lane_id = self.classifier.predict([tfl_coords])[0]
            except ValueError:
                rospy.logdebug("Traffic light %d at coordinates (%f, %f) not found in map", light.id, tfl_coords[0], tfl_coords[1])
                continue

            tfl_result = TrafficLightResult()
            tfl_result.light_id = light_id
            tfl_result.lane_id = lane_id
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
