#!/usr/bin/env python3

import rospy
import message_filters
import numpy as np
import traceback

from autoware_mini.msg import StopLineStatus, StopLineStatusArray

TRAFFIC_LIGHT_RESULT_TO_STRING = {
    0: "RED",    # and yellow
    1: "GREEN",
    2: "UNKNOWN",
    3: "MISSING"
}

class TrafficLightMajorityMerger:
    def __init__(self):

        # Node parameters
        self.id_string = rospy.get_param('~id_string')

        # Publishers
        self.tfl_status_pub = rospy.Publisher('traffic_light_status', StopLineStatusArray, queue_size=1, tcp_nodelay=True)

        # Subscribers
        camera1_tfl_status = message_filters.Subscriber('camera1/traffic_light_status', StopLineStatusArray, queue_size=1, tcp_nodelay=True)
        camera2_tfl_status = message_filters.Subscriber('camera2/traffic_light_status', StopLineStatusArray, queue_size=1, tcp_nodelay=True)
        ts = message_filters.ApproximateTimeSynchronizer([camera1_tfl_status, camera2_tfl_status], queue_size=2, slop=0.2)
        ts.registerCallback(self.camera_tfl_status_callback)

    def camera_tfl_status_callback(self, camera1_tfl_status, camera2_tfl_status):

        try:
            
            merged_tfl_status_msg = StopLineStatusArray()
            merged_tfl_status_msg.type = StopLineStatusArray.TRAFFIC_LIGHT
            merged_tfl_status_msg.header.stamp = min(camera1_tfl_status.header.stamp, camera2_tfl_status.header.stamp)

            # create dictionary out of traffic light status messages - use index as result and increase its count
            tfl_status_counts = {}
            for msg in [camera1_tfl_status, camera2_tfl_status]:
                for result in msg.statuses:
                    if result.stop_line_id not in tfl_status_counts:
                        # create list with 4 zeros (4 possible states in TrafficLightResult)
                        tfl_status_counts[result.stop_line_id] = [0] * 4
                    tfl_status_counts[result.stop_line_id][result.status] += 1

            # find max_count and decide for result
            for stop_line_id, status_list in tfl_status_counts.items():
                if sum(status_list[:3]) == 0:
                    merged_result = StopLineStatus.STATUS_UNKNOWN # publish unknown if all statuses are missing
                else:
                    # always prefer min value of the results: 0 - red / yellow < 1 - green < 2 - unknown
                    merged_result = np.argmax(status_list[:3]) # ignore missing

                new_msg = StopLineStatus()
                new_msg.stop_line_id = stop_line_id
                new_msg.status = int(merged_result)
                new_msg.status_text = TRAFFIC_LIGHT_RESULT_TO_STRING[merged_result] + self.id_string
                merged_tfl_status_msg.statuses.append(new_msg)

            self.tfl_status_pub.publish(merged_tfl_status_msg)

        except Exception as e:
            rospy.logerr_throttle(10, "%s - Exception in callback: %s", rospy.get_name(), traceback.format_exc())

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('traffic_light_majority_merger', log_level=rospy.INFO)
    node = TrafficLightMajorityMerger()
    node.run()