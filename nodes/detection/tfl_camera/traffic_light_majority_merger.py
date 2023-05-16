#!/usr/bin/env python3

import rospy
import message_filters

from autoware_msgs.msg import TrafficLightResult, TrafficLightResultArray

class TrafficLightMajorityMerger:
    def __init__(self):

        # Node parameters

        # Publishers
        self.tfl_status_pub = rospy.Publisher('/traffic_light_status', TrafficLightResultArray, queue_size=1)

        # Subscribers
        # use approcimate time sync to sync 2 camera traffic light satatus messages
        camera_fl_tfl_status = message_filters.Subscriber('/detection/camera_fl_traffic_light_status', TrafficLightResultArray)
        camera_fr_tfl_status = message_filters.Subscriber('/detection/camera_fr_traffic_light_status', TrafficLightResultArray)
        ts = message_filters.ApproximateTimeSynchronizer([camera_fl_tfl_status, camera_fr_tfl_status], queue_size=2, slop=0.5)
        ts.registerCallback(self.camera_tfl_status_callback)

    def camera_tfl_status_callback(self, camera_fl_tfl_status_msg, camera_fr_tfl_status_msg):

        merged_tfl_status_msg = TrafficLightResultArray()
        merged_tfl_status_msg.header.stamp = camera_fl_tfl_status_msg.header.stamp

        # create dictionary of traffic light status messages
        tfl_status_msgs = {}
        for msg in [camera_fl_tfl_status_msg, camera_fr_tfl_status_msg]:
            for result in msg.results:
                if result.lane_id not in tfl_status_msgs:
                    # create list with 4 zeros (3 possible states)  # TODO TrafficLightResult red and yellow both 0 values
                    tfl_status_msgs[result.lane_id] = [0] * 3
                    tfl_status_msgs[result.lane_id][result.recognition_result] += 1
                else:
                    tfl_status_msgs[result.lane_id][result.recognition_result] += 1


        # iterate over dict and find argmax in list
        for lane_id, status_list in tfl_status_msgs.items():
            max_value = max(status_list)
            max_indeces = [i for i, j in enumerate(status_list) if j == max_value]

            # always prefer min value of the results: 0 - red / yellow < 1 - green < 2 - unknown
            merged_result = min(max_indeces)

            # create new TrafficLightResult message
            new_msg = TrafficLightResult()
            new_msg.lane_id = lane_id
            new_msg.recognition_result = merged_result
            # publish new message

            merged_tfl_status_msg.results.append(new_msg)

        self.tfl_status_pub.publish(merged_tfl_status_msg)


    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('traffic_light_majority_merger', log_level=rospy.INFO)
    node = TrafficLightMajorityMerger()
    node.run()