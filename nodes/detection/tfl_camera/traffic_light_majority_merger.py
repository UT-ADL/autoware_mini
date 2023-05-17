#!/usr/bin/env python3

import rospy
import message_filters
import numpy as np

from autoware_msgs.msg import TrafficLightResult, TrafficLightResultArray

TRAFFIC_LIGHT_RESULT_TO_STRING = {
    0: "RED",    # and yellow
    1: "GREEN",
    2: "UNKNOWN"
}

class TrafficLightMajorityMerger:
    def __init__(self):

        # Publishers
        self.tfl_status_pub = rospy.Publisher('traffic_light_status', TrafficLightResultArray, queue_size=1)

        # Subscribers
        camera_fl_tfl_status = message_filters.Subscriber('camera_fl/traffic_light_status', TrafficLightResultArray)
        camera_fr_tfl_status = message_filters.Subscriber('camera_fr/traffic_light_status', TrafficLightResultArray)
        ts = message_filters.ApproximateTimeSynchronizer([camera_fl_tfl_status, camera_fr_tfl_status], queue_size=2, slop=0.1)
        ts.registerCallback(self.camera_tfl_status_callback)

    def camera_tfl_status_callback(self, camera_fl_tfl_status_msg, camera_fr_tfl_status_msg):

        merged_tfl_status_msg = TrafficLightResultArray()
        merged_tfl_status_msg.header.stamp = camera_fl_tfl_status_msg.header.stamp

        # create dictionary out of traffic light status messages - use index as result and increase its count
        tfl_status_counts = {}
        for msg in [camera_fl_tfl_status_msg, camera_fr_tfl_status_msg]:
            for result in msg.results:
                if result.lane_id not in tfl_status_counts:
                    # create list with 3 zeros (3 possible states in TrafficLightResult)
                    tfl_status_counts[result.lane_id] = [0] * 3
                tfl_status_counts[result.lane_id][result.recognition_result] += 1

        # find max_count and decide for result
        for lane_id, status_list in tfl_status_counts.items():
            # always prefer min value of the results: 0 - red / yellow < 1 - green < 2 - unknown
            merged_result = np.argmax(status_list)

            new_msg = TrafficLightResult()
            new_msg.lane_id = lane_id
            new_msg.recognition_result = merged_result
            # TODO QUESTION yellow string and other status string get lost here - should retain?
            new_msg.recognition_result_str = TRAFFIC_LIGHT_RESULT_TO_STRING[merged_result]
            merged_tfl_status_msg.results.append(new_msg)

        self.tfl_status_pub.publish(merged_tfl_status_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('traffic_light_majority_merger', log_level=rospy.INFO)
    node = TrafficLightMajorityMerger()
    node.run()