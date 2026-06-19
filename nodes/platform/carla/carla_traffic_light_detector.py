#!/usr/bin/env python3

import traceback
import rospy
import ros_numpy
import numpy as np

from sklearn.neighbors import KNeighborsClassifier

from carla_msgs.msg import CarlaTrafficLightStatus, CarlaTrafficLightStatusList, CarlaTrafficLightInfoList
from autoware_mini.msg import StopLineStatus, StopLineStatusArray

from autoware_mini.lanelet2 import get_stop_lines_center, load_lanelet2_map

# Carla to Autoware traffic light status mapping
CARLA_TO_AUTOWARE_TFL_MAP = {
    CarlaTrafficLightStatus.RED: StopLineStatus.STATUS_STOP,
    CarlaTrafficLightStatus.YELLOW: StopLineStatus.STATUS_STOP,
    CarlaTrafficLightStatus.GREEN: StopLineStatus.STATUS_GO,
    CarlaTrafficLightStatus.OFF: StopLineStatus.STATUS_GO,
    CarlaTrafficLightStatus.UNKNOWN: StopLineStatus.STATUS_UNKNOWN
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
        lanelet2_map_path = rospy.get_param("~lanelet2_map_path")
        self.publish_rate = rospy.get_param("~publish_rate")

        # Load lanelet2 map
        lanelet2_map = load_lanelet2_map(lanelet2_map_path)

        # Get stop line centers with stop_line_id and corresponding light_ids mapping
        self.stop_line_centers_map = get_stop_lines_center(lanelet2_map)

        # Carla_light_id to stop_line_id mapping
        self.light_id_to_stop_line_id_map = {}

        # Latest traffic light status from Carla (republished at fixed rate)
        self.latest_traffic_lights = None

        # Publishers
        self.tfl_status_pub = rospy.Publisher('traffic_light_status', StopLineStatusArray, queue_size=1, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber('/carla/traffic_lights/info',
                         CarlaTrafficLightInfoList, self.tfl_info_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/carla/traffic_lights/status',
                    CarlaTrafficLightStatusList, self.tfl_status_callback, queue_size=1, tcp_nodelay=True)

        # Timer
        rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.publish_timer_callback)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def tfl_info_callback(self, msg):
        """
        callback CarlaTrafficLightInfoList
        """

        trigger_volume_coords = []
        light_ids = []

        for tfl in msg.traffic_lights:
            pose = tfl.transform

            # Transform trigger volume location using the transformation matrix from tfl pose
            trans_matrix = ros_numpy.numpify(pose)
            trigger_coords = np.array([tfl.trigger_volume.center.x, tfl.trigger_volume.center.y, tfl.trigger_volume.center.z, 1])
            center_x, center_y, _, _ = np.dot(trans_matrix, trigger_coords)

            trigger_volume_coords.append((center_x, center_y))
            light_ids.append((tfl.id))

        # Initialize classifier to predict the closest trigger volume
        classifier = KNeighborsClassifier(n_neighbors=1)
        classifier.fit(trigger_volume_coords, light_ids)
        
        # Predict closest trigger volume to stop line center and create carla_light_id to stop_line_id mapping
        for stop_line_id, ((center_x, center_y), lanelet_light_ids) in self.stop_line_centers_map.items():
            try:
                carla_light_id = classifier.predict([(center_x, center_y)])[0]
                self.light_id_to_stop_line_id_map[carla_light_id] = (stop_line_id, lanelet_light_ids)
            except:
                rospy.logwarn_throttle(10, "%s Unable to find nearest traffic light trigger volume for stop line %d", rospy.get_name(), stop_line_id)


    def tfl_status_callback(self, msg):
        """
        callback CarlaTrafficLightStatusList - cache latest status, actual publishing happens on timer
        """
        self.latest_traffic_lights = msg.traffic_lights

    def publish_timer_callback(self, event):
        try:
            traffic_lights = self.latest_traffic_lights
            if traffic_lights is None:
                return

            tfl_status = StopLineStatusArray()
            tfl_status.type = StopLineStatusArray.TRAFFIC_LIGHT
            tfl_status.header.stamp = rospy.Time.now()

            for light in traffic_lights:

                if light.id not in self.light_id_to_stop_line_id_map:
                    rospy.logdebug_throttle(10, "%s - traffic light %d not found in info", rospy.get_name(), light.id)
                    continue

                stop_line_id, lanelet_light_ids = self.light_id_to_stop_line_id_map[light.id]

                for light_id in lanelet_light_ids:

                    tfl_result = StopLineStatus()
                    tfl_result.traffic_light_id = light_id
                    tfl_result.stop_line_id = stop_line_id
                    tfl_result.status = CARLA_TO_AUTOWARE_TFL_MAP[light.state]
                    tfl_result.status_text = CARLA_TO_AUTOWARE_TFL_STR[light.state]
                    tfl_status.statuses.append(tfl_result)

            self.tfl_status_pub.publish(tfl_status)
        except Exception:
            rospy.logerr_throttle(10, "%s - Exception in callback: %s", rospy.get_name(), traceback.format_exc())

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('carla_traffic_light_detector', log_level=rospy.INFO)
    node = CarlaTrafficLightDetector()
    node.run()
