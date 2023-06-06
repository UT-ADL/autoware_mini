#!/usr/bin/env python3

import numpy as np
from scipy.optimize import linear_sum_assignment

import rospy

from autoware_msgs.msg import DetectedObjectArray
from helpers.detection import calculate_iou

class EMATracker:
    def __init__(self):

        # Parameters
        self.detection_counter_threshold = rospy.get_param('~detection_counter_threshold')
        self.missed_counter_threshold = rospy.get_param('~missed_counter_threshold')
        self.velocity_gain = rospy.get_param('~velocity_gain')

        self.tracked_objects = []
        self.tracked_objects_array = np.empty((0, 12), dtype=np.float32)
        self.track_id_counter = 0
        self.stamp = None

        # Publishers
        self.tracked_objects_pub = rospy.Publisher('tracked_objects', DetectedObjectArray, queue_size=1)

        # Subscribers
        rospy.Subscriber('detected_objects', DetectedObjectArray, self.detected_objects_callback, queue_size=1)

    def detected_objects_callback(self, msg):
        assert len(self.tracked_objects) == len(self.tracked_objects_array), str(len(self.tracked_objects)) + ' ' + str(len(self.tracked_objects_array))

        detected_objects = msg.objects
        # convert detected objects into Numpy array
        detected_objects_array = np.empty((len(msg.objects), 12), dtype=np.float32)
        for i, obj in enumerate(msg.objects):
            detected_objects_array[i] = [
                obj.pose.position.x, obj.pose.position.y,
                obj.pose.position.x + obj.dimensions.x, obj.pose.position.y + obj.dimensions.y,
                obj.dimensions.x, obj.dimensions.y,
                obj.velocity.linear.x, obj.velocity.linear.y, 
                obj.acceleration.linear.x, obj.acceleration.linear.y,
                0, 1]   # zero missed detections, one detection
        assert len(detected_objects) == len(detected_objects_array)

        # Calculate the IOU between the tracked objects and the detected objects
        iou = calculate_iou(self.tracked_objects_array[:, :4], detected_objects_array[:, :4])
        assert iou.shape == (len(self.tracked_objects), len(detected_objects))

        # Calculate the association between the tracked objects and the detected objects
        matched_track_indices, matched_detection_indicies = linear_sum_assignment(-iou)
        assert len(matched_track_indices) == len(matched_detection_indicies)

        # Only keep those matches where the IOU is greater than 0.0
        matches = iou[matched_track_indices, matched_detection_indicies] > 0.0
        matched_track_indices = matched_track_indices[matches]
        matched_detection_indicies = matched_detection_indicies[matches]
        assert len(matched_track_indices) == len(matched_detection_indicies)

        # calculate time difference between current and previous message
        if self.stamp is not None:
            time_delta = (msg.header.stamp - self.stamp).to_sec()
        else:
            time_delta = 0.1
        self.stamp = msg.header.stamp

        # update tracked object speeds
        new_velocities = (detected_objects_array[matched_detection_indicies, :2] - self.tracked_objects_array[matched_track_indices, :2]) / time_delta
        current_velocities = self.tracked_objects_array[matched_track_indices, 6:8] 
        detected_objects_array[matched_detection_indicies, 6:8] = (1 - self.velocity_gain) * current_velocities + self.velocity_gain * new_velocities

        # Replace tracked objects with detected objects, keeping the same ID
        for track_idx, detection_idx in zip(matched_track_indices, matched_detection_indicies):
            tracked_obj = self.tracked_objects[track_idx]
            detected_obj = detected_objects[detection_idx]
            detected_obj.id = tracked_obj.id
            if not detected_obj.velocity_reliable:
                detected_obj.velocity.linear.x = detected_objects_array[detection_idx, 6]
                detected_obj.velocity.linear.y = detected_objects_array[detection_idx, 7]
                detected_obj.velocity_reliable = True
            self.tracked_objects[track_idx] = detected_obj
        self.tracked_objects_array[matched_track_indices, :10] = detected_objects_array[matched_detection_indicies, :10]

        # create missed track indices
        all_track_indices = np.arange(0, len(self.tracked_objects), 1, dtype=int)
        missed_track_indices = np.delete(all_track_indices, matched_track_indices)

        # create new detection indices
        all_detection_indices = np.arange(0, len(detected_objects), 1, dtype=int)
        new_detection_indices = np.delete(all_detection_indices, matched_detection_indicies)

        # zero missed counter and increase detected counter for matched objects
        self.tracked_objects_array[matched_track_indices, 10] = 0
        self.tracked_objects_array[matched_track_indices, 11] += 1

        # increase missed counter and zero detection counter for non-matched tracks
        self.tracked_objects_array[missed_track_indices, 10] += 1
        self.tracked_objects_array[missed_track_indices, 11] = 0

        # delete stale tracks
        stale_track_indices = np.where(self.tracked_objects_array[:, 10] >= self.missed_counter_threshold)[0]
        self.tracked_objects_array = np.delete(self.tracked_objects_array, stale_track_indices, axis=0)
        for idx in sorted(stale_track_indices, reverse=True):
            del self.tracked_objects[idx]
        assert len(self.tracked_objects) == len(self.tracked_objects_array), str(len(self.tracked_objects)) + ' ' + str(len(self.tracked_objects_array))

        # add new detections
        for obj_idx in new_detection_indices:
            detected_obj = detected_objects[obj_idx]
            detected_obj.id = self.track_id_counter
            self.tracked_objects.append(detected_obj)
            self.track_id_counter += 1
        self.tracked_objects_array = np.vstack((self.tracked_objects_array, detected_objects_array[new_detection_indices]))
        assert len(self.tracked_objects) == len(self.tracked_objects_array)

        # filter out objects with enough detections
        tracked_objects_indices = np.where(self.tracked_objects_array[:, 11] >= self.detection_counter_threshold)[0]
        tracked_objects = [self.tracked_objects[idx] for idx in tracked_objects_indices]
        assert len(tracked_objects) == len(tracked_objects_indices)

        # publish tracked objects
        tracked_objects_msg = DetectedObjectArray()
        tracked_objects_msg.header.stamp = msg.header.stamp
        tracked_objects_msg.header.frame_id = msg.header.frame_id
        tracked_objects_msg.objects = tracked_objects
        self.tracked_objects_pub.publish(tracked_objects_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('ema_tracker', log_level=rospy.INFO)
    node = EMATracker()
    node.run()
