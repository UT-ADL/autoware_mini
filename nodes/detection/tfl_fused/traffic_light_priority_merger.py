#!/usr/bin/env python3

import rospy
import threading

from autoware_msgs.msg import TrafficLightResultArray


class TrafficLightPriorityMerger:
    def __init__(self):

        # Variables
        self.lock = threading.Lock()
        self.topic_2_tfl_results = {}

        # Publishers
        self.tfl_status_pub = rospy.Publisher('/traffic_light_status', TrafficLightResultArray, queue_size=1)

        # Subscribers
        rospy.Subscriber('/tfl_status_topic_1', TrafficLightResultArray, self.tfl_status_topic_1_callback, queue_size=1)
        rospy.Subscriber('/tfl_status_topic_2', TrafficLightResultArray, self.tfl_status_topic_2_callback, queue_size=1)


    def tfl_status_topic_1_callback(self, msg):

        with self.lock:
            topic_2_tfl_results = self.topic_2_tfl_results

        merged_tfl_status_msg = TrafficLightResultArray()
        merged_tfl_status_msg.header.stamp = msg.header.stamp

        for result in msg.results:
            # priority topic result is not unkown -> add to merged result
            if result.recognition_result_str != "UNKNOWN":
                merged_tfl_status_msg.results.append(result)

                # delete result from topic 2 if it exists there
                if result.lane_id in topic_2_tfl_results.keys():
                    del topic_2_tfl_results[result.lane_id]
            
            if result.recognition_result_str == "UNKNOWN" and result.lane_id in topic_2_tfl_results.keys():
                # if result is unknown and there is a result in topic 2, use topic 2 result
                merged_tfl_status_msg.results.append(topic_2_tfl_results[result.lane_id])
                del topic_2_tfl_results[result.lane_id]
            else:
                # if result is unknown and there is no result in topic 2, use topic 1 result
                merged_tfl_status_msg.results.append(result)

        # add remaining topic 2 results to merged result if there are any
        for result in topic_2_tfl_results.values():
            merged_tfl_status_msg.results.append(result)

        self.tfl_status_pub.publish(merged_tfl_status_msg)


    def tfl_status_topic_2_callback(self, msg):

        # iterate over msg and add results to dictionary
        topic_2_tfl_results = {}

        for result in msg.results:
            if result.lane_id not in topic_2_tfl_results:
                topic_2_tfl_results[result.lane_id] = result
            else:
                rospy.logwarn("traffic_light_priority_merger - Duplicate lane_id (%s) in tfl_status_topic_2, keeping first", result.lane_id)

        with self.lock:
            self.topic_2_tfl_results = topic_2_tfl_results


    def tfl_status_callback(self, mqtt_tfl_status_msg, camera_tfl_status_msg):

        merged_tfl_status_msg = TrafficLightResultArray()
        merged_tfl_status_msg.header.stamp = mqtt_tfl_status_msg.header.stamp


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('traffic_light_priority_merger', log_level=rospy.INFO)
    node = TrafficLightPriorityMerger()
    node.run()