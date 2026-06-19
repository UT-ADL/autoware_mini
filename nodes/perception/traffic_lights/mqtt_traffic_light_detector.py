#!/usr/bin/env python3

import rospy
import json
import struct
import time
import traceback

import paho.mqtt.client as mqtt

from geometry_msgs.msg import PoseStamped
from autoware_mini.msg import StopLineStatus, StopLineStatusArray

from autoware_mini.geometry import get_distance_between_two_points_2d
from autoware_mini.lanelet2 import load_lanelet2_map, get_stop_lines_api_id

MQTT_TO_AUTOWARE_TFL_MAP = {
    "RED": StopLineStatus.STATUS_STOP,
    "RED/AMB": StopLineStatus.STATUS_STOP,
    "REDVAMB": StopLineStatus.STATUS_STOP,
    "RED\\AMB": StopLineStatus.STATUS_STOP,
    "AMBER-RED": StopLineStatus.STATUS_STOP,
    "AMBERRED" : StopLineStatus.STATUS_STOP,
    "YELLOW": StopLineStatus.STATUS_STOP,
    "AMBER": StopLineStatus.STATUS_STOP,
    "FLASH" : StopLineStatus.STATUS_UNKNOWN,
    "AMB FLASH" : StopLineStatus.STATUS_UNKNOWN,
    "AMBER FLASH" : StopLineStatus.STATUS_UNKNOWN,
    "GREEN FLASH": StopLineStatus.STATUS_GO,
    "GREEN": StopLineStatus.STATUS_GO,
    "OFF": StopLineStatus.STATUS_UNKNOWN,
    "UNKNOWN": StopLineStatus.STATUS_UNKNOWN,
    "DARK/FLASH": StopLineStatus.STATUS_UNKNOWN
}

BINARY_MQTT_TO_STR = {
    0: "GREEN",
    1: "RED",
    2: "AMBER",
    3: "GREEN FLASH",
    4: "RED/AMB",
    5: "AMBER FLASH",
    6: "DARK"
}

BINARY_MQTT_MSG_FORMAT_1 = "<B Q B"  # Version, Timestamp, Status
BINARY_MQTT_MSG_FORMAT_2 = "<B Q B i"  # Version, Timestamp, Status, Since Change
BINARY_MQTT_MSG_FORMAT_3 = "<B Q B i i"  # Version, Timestamp, Status, Since Change, Till Change

BINARY_MSG_1_SIZE = struct.calcsize(BINARY_MQTT_MSG_FORMAT_1)
BINARY_MSG_2_SIZE = struct.calcsize(BINARY_MQTT_MSG_FORMAT_2)
BINARY_MSG_3_SIZE = struct.calcsize(BINARY_MQTT_MSG_FORMAT_3)

class MqttTrafficLightDetector:
    def __init__(self):

        # Node parameters
        self.mqtt_host = rospy.get_param('~mqtt_host')
        self.mqtt_port = rospy.get_param('~mqtt_port')
        self.mqtt_topic = rospy.get_param('~mqtt_topic')
        self.enable_automatic_subscribe = rospy.get_param('~enable_automatic_subscribe')
        self.automatic_subscription_range = rospy.get_param('~automatic_subscription_range')
        self.local_path_length = rospy.get_param("/planning/local_path_length")
        self.timeout = rospy.get_param('~timeout')
        self.id_string = rospy.get_param('~id_string')
        self.publish_rate = rospy.get_param('~publish_rate')
        lanelet2_map_path = rospy.get_param("~lanelet2_map_path")

        self.lanelet2_map = load_lanelet2_map(lanelet2_map_path)

        # MQTT traffic light status
        self.mqtt_status = {}

        # Publishers
        self.tfl_status_pub = rospy.Publisher('traffic_light_status', StopLineStatusArray, queue_size=1, tcp_nodelay=True)

        self.client = mqtt.Client()
        self.client.on_message = self.on_message
        self.client.on_disconnect = self.on_disconnect
        self.client.on_connect = self.on_connect

        self.client.tls_set("/etc/ssl/certs/ca-certificates.crt")
        self.client.connect(self.mqtt_host, self.mqtt_port, keepalive=10)
        self.client.loop_start()

        if self.enable_automatic_subscribe:
            self.stop_line_ids = {}
            self.last_fetch_location = None
            rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1, tcp_nodelay=True)
        else:
            self.stop_line_ids = get_stop_lines_api_id(self.lanelet2_map)

        # Timer
        rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.publish_timer_callback)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def current_pose_callback(self, msg):
        # fetch stopilines after ego vehicle has travelled a little less than the length of the local path
        if self.last_fetch_location is not None and get_distance_between_two_points_2d(self.last_fetch_location, msg.pose.position) < self.local_path_length:
            return

        stop_line_ids_in_range = get_stop_lines_api_id(self.lanelet2_map, msg.pose.position.x, msg.pose.position.y, 
                                                   self.automatic_subscription_range + self.local_path_length)

        seen_api_ids = set(stop_line_ids_in_range.values())
        for api_id in seen_api_ids:
            self.client.subscribe(api_id)

        # Unsubscribe from api ids that are not within range anymore
        not_seen_api_ids = set(self.stop_line_ids.values()) - seen_api_ids
        for api_id in not_seen_api_ids:
            self.client.unsubscribe(api_id)

        self.stop_line_ids = stop_line_ids_in_range
        self.last_fetch_location = msg.pose.position
        
    def on_connect(self, client, userdata, flags, rc):
        if rc != 0:
            rospy.logerr('%s - failed to connect to MQTT server %s:%d, return code: %d', rospy.get_name(), self.mqtt_host, self.mqtt_port, rc)
            return

        rospy.loginfo('%s - connected to MQTT server %s:%d', rospy.get_name(), self.mqtt_host, self.mqtt_port)
        if not self.enable_automatic_subscribe:
            client.subscribe(self.mqtt_topic)

    def on_disconnect(self, client, userdata, rc):
        rospy.logerr('%s - disconnected from MQTT server %s:%d, return code: %d', rospy.get_name(), self.mqtt_host, self.mqtt_port, rc)

    def on_message(self, client, userdata, msg):
        rospy.logdebug('%s - MQTT message recieved: %s, %s', rospy.get_name(), msg.topic, str(msg.payload))
        # collect all messages
        api_id = msg.topic

        # if message starts with '{' then it's in json format
        if chr(msg.payload[0]) == "{":
            mqtt_data = json.loads(msg.payload)
            
        else:
            if len(msg.payload) == BINARY_MSG_1_SIZE:
                version, timestamp, status = struct.unpack(BINARY_MQTT_MSG_FORMAT_1, msg.payload)
                since_change = -1
                till_change = -1

            elif len(msg.payload) == BINARY_MSG_2_SIZE:
                version, timestamp, status, since_change = struct.unpack(BINARY_MQTT_MSG_FORMAT_2, msg.payload)
                till_change = -1

            elif len(msg.payload) == BINARY_MSG_3_SIZE:
                version, timestamp, status, since_change, till_change = struct.unpack(BINARY_MQTT_MSG_FORMAT_3, msg.payload)
                
            else:
                rospy.logerr('%s - binary mqtt message size %d does not match any expected sizes %d, %d or %d', rospy.get_name(), len(msg.payload), 
                             BINARY_MSG_1_SIZE, BINARY_MSG_2_SIZE, BINARY_MSG_3_SIZE)
                return

            mqtt_data = {"version": version,
                         "timestamp": timestamp,
                         "status": BINARY_MQTT_TO_STR[status],
                         "since_change": since_change,
                         "till_change": till_change}

        self.mqtt_status[api_id] = mqtt_data

    def publish_timer_callback(self, event):
        """
        Combine extracted stop lines with api_id from the map with the traffic light status from mqtt
        """
        try:

            # iterate over stop lines and create StopLineStatusArray
            tfl_status = StopLineStatusArray()
            tfl_status.type = StopLineStatusArray.TRAFFIC_LIGHT
            tfl_status.header.stamp = rospy.Time.now()

            for stop_line_id, api_id in self.stop_line_ids.items():

                result_str = "UNKNOWN"
                result = MQTT_TO_AUTOWARE_TFL_MAP[result_str]

                # extract status from mqtt_status if key exits
                if api_id in self.mqtt_status:
                    timestamp = self.mqtt_status[api_id]["timestamp"]
                    result_str = self.mqtt_status[api_id]["status"]

                    time_diff = (time.time() * 1000 - timestamp) / 1000

                    # get traffic light status
                    if time_diff > self.timeout:
                        rospy.logwarn('%s - timeout of stop line: %s, by %f seconds', rospy.get_name(), api_id, time_diff)
                    elif time_diff < -0.5:
                        rospy.logwarn('%s - stop line timestamp in the future: %s, by %f seconds', rospy.get_name(), api_id, abs(time_diff))
                    else:
                        if result_str in MQTT_TO_AUTOWARE_TFL_MAP:
                            result = MQTT_TO_AUTOWARE_TFL_MAP[result_str]
                        else:
                            rospy.logwarn('%s - unknown stop line state: %s', rospy.get_name(), result_str)

                tfl_result = StopLineStatus()
                tfl_result.traffic_light_id = 0
                tfl_result.stop_line_id = stop_line_id
                tfl_result.status = result
                tfl_result.status_text = result_str + self.id_string
                tfl_status.statuses.append(tfl_result)

            self.tfl_status_pub.publish(tfl_status)

        except Exception:
            rospy.logerr_throttle(10, "%s - Exception in callback: %s", rospy.get_name(), traceback.format_exc())

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('mqtt_traffic_light_detector', log_level=rospy.INFO)
    node = MqttTrafficLightDetector()
    node.run()
