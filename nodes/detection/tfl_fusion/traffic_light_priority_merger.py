#!/usr/bin/env python3

import rospy
import message_filters
import traceback

from autoware_msgs.msg import TrafficLightResultArray


class TrafficLightPriorityMerger:
    def __init__(self):

        # Variables
        self.topic_2_tfl_results = {}

        # Publishers
        self.tfl_status_pub = rospy.Publisher('traffic_light_status', TrafficLightResultArray, queue_size=1, tcp_nodelay=True)

        # Subscribers
        topic_1_tfl_status = message_filters.Subscriber('tfl_status_topic_1', TrafficLightResultArray, queue_size=1, tcp_nodelay=True)
        topic_2_tfl_status = message_filters.Subscriber('tfl_status_topic_2', TrafficLightResultArray, queue_size=1, tcp_nodelay=True)
        ts = message_filters.ApproximateTimeSynchronizer([topic_1_tfl_status, topic_2_tfl_status], queue_size=5, slop=0.1)
        ts.registerCallback(self.tfl_status_callback)

    def tfl_status_callback(self, topic_1_msg, topic_2_msg):

        try:
            merged_tfl_status_msg = TrafficLightResultArray()
            merged_tfl_status_msg.header.stamp = topic_1_msg.header.stamp

            # iterate over msg and add results to dictionary
            topic_2_tfl_results = {}

            for result in topic_2_msg.results:
                if result.lane_id not in topic_2_tfl_results:
                    topic_2_tfl_results[result.lane_id] = result
                else:
                    rospy.logwarn("%s - Duplicate lane_id (%s) in tfl_status_topic_2, keeping first", rospy.get_name(), result.lane_id)

            # iterate over priority topic (MQTT)
            for result in topic_1_msg.results:
                # priority topic result is not unkown -> add to merged result
                if result.recognition_result != 2:
                    merged_tfl_status_msg.results.append(result)
                    # delete result from topic 2 if it exists there
                    if result.lane_id in topic_2_tfl_results.keys():
                        del topic_2_tfl_results[result.lane_id]
                else:
                    if result.lane_id in topic_2_tfl_results.keys():
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

        except Exception as e:
            rospy.logerr_throttle(10, "%s - Exception in callback: %s", rospy.get_name(), traceback.format_exc())

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('traffic_light_priority_merger', log_level=rospy.INFO)
    node = TrafficLightPriorityMerger()
    node.run()