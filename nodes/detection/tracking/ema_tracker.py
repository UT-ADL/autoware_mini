#!/usr/bin/env python3

import rospy
import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.spatial.distance import cdist
from autoware_msgs.msg import DetectedObjectArray
from helpers.detection import calculate_iou, get_axis_oriented_bounding_box

class EMATracker:
    def __init__(self):

        # Parameters
        self.detection_counter_threshold = rospy.get_param('~detection_counter_threshold')
        self.missed_counter_threshold = rospy.get_param('~missed_counter_threshold')
        self.velocity_gain = rospy.get_param('~velocity_gain')
        self.acceleration_gain = rospy.get_param('~acceleration_gain')
        self.association_method = rospy.get_param('~association_method')
        self.max_euclidean_distance = rospy.get_param('~max_euclidean_distance')

        self.tracked_objects = []
        self.tracked_objects_array = np.empty((0,), dtype=[
            ('centroid', np.float32, (2,)),
            ('bbox', np.float32, (4,)),
            ('velocity', np.float32, (2,)),
            ('acceleration', np.float32, (2,)),
            ('missed_counter', np.int32),
            ('detection_counter', np.int32),
        ])
        self.track_id_counter = 0
        self.stamp = None

        # Publishers
        self.tracked_objects_pub = rospy.Publisher('tracked_objects', DetectedObjectArray, queue_size=1)

        # Subscribers
        rospy.Subscriber('detected_objects', DetectedObjectArray, self.detected_objects_callback, queue_size=1)

    def detected_objects_callback(self, msg):
        ### 1. PREPARE DETECTIONS ###

        # convert detected objects into Numpy array
        detected_objects = msg.objects
        detected_objects_array = np.empty((len(detected_objects)), dtype=self.tracked_objects_array.dtype)
        for i, obj in enumerate(detected_objects):
            detected_objects_array[i]['centroid'] = (obj.pose.position.x, obj.pose.position.y)
            detected_objects_array[i]['bbox'] = get_axis_oriented_bounding_box(obj)
            detected_objects_array[i]['velocity'] = (obj.velocity.linear.x, obj.velocity.linear.y) 
            detected_objects_array[i]['acceleration'] = (obj.acceleration.linear.x, obj.acceleration.linear.y)
            detected_objects_array[i]['missed_counter'] = 0
            detected_objects_array[i]['detection_counter'] = 1
        assert len(detected_objects) == len(detected_objects_array)

        ### 2. PROPAGATE EXISTING TRACKS FORWARD ###

        # calculate time difference between current and previous message
        if self.stamp is not None:
            time_delta = (msg.header.stamp - self.stamp).to_sec()
        else:
            time_delta = 0.1
        self.stamp = msg.header.stamp

        # move tracked objects forward in time
        assert len(self.tracked_objects) == len(self.tracked_objects_array), str(len(self.tracked_objects)) + ' ' + str(len(self.tracked_objects_array))
        position_change = time_delta * self.tracked_objects_array['velocity']
        tracked_object_centroids = self.tracked_objects_array['centroid'].copy()
        tracked_object_centroids += position_change
        tracked_object_bboxes = self.tracked_objects_array['bbox'].copy()
        tracked_object_bboxes[:,:2] += position_change
        tracked_object_bboxes[:,2:4] += position_change

        ### 3. MATCH TRACKS WITH DETECTIONS ###

        if self.association_method == 'iou':
            # Calculate the IOU between the tracked objects and the detected objects
            iou = calculate_iou(tracked_object_bboxes, detected_objects_array['bbox'])
            assert iou.shape == (len(self.tracked_objects_array), len(detected_objects_array))

            # Calculate the association between the tracked objects and the detected objects
            matched_track_indices, matched_detection_indicies = linear_sum_assignment(-iou)
            assert len(matched_track_indices) == len(matched_detection_indicies)

            # Only keep those matches where the IOU is greater than 0.0
            matches = iou[matched_track_indices, matched_detection_indicies] > 0.0
            matched_track_indices = matched_track_indices[matches]
            matched_detection_indicies = matched_detection_indicies[matches]
            assert len(matched_track_indices) == len(matched_detection_indicies)
        elif self.association_method == 'euclidean':
            # Calculate euclidean distance between the tracked object and the detected object centroids
            dists = cdist(tracked_object_centroids, detected_objects_array['centroid'])
            assert dists.shape == (len(self.tracked_objects_array), len(detected_objects_array))

            # Calculate the association between the tracked objects and the detected objects
            matched_track_indices, matched_detection_indicies = linear_sum_assignment(dists)

            # Only keep those matches where the distance is less than threshold
            matches = dists[matched_track_indices, matched_detection_indicies] <= self.max_euclidean_distance
            matched_track_indices = matched_track_indices[matches]
            matched_detection_indicies = matched_detection_indicies[matches]
            assert len(matched_track_indices) == len(matched_detection_indicies)
        else:
            assert False, 'Unknown association method: ' + self.association_method

        ### 4. ESTIMATE TRACKED OBJECT SPEEDS AND ACCELERATIONS ###

        # update tracked object speeds with exponential moving average
        new_velocities = (detected_objects_array['centroid'][matched_detection_indicies] - self.tracked_objects_array['centroid'][matched_track_indices]) / time_delta
        old_velocities = self.tracked_objects_array['velocity'][matched_track_indices]
        detected_objects_array['velocity'][matched_detection_indicies] = (1 - self.velocity_gain) * old_velocities + self.velocity_gain * new_velocities

        # update tracked object accelerations with exponential moving average
        new_accelerations = (detected_objects_array['velocity'][matched_detection_indicies] - self.tracked_objects_array['velocity'][matched_track_indices]) / time_delta
        old_accelerations = self.tracked_objects_array['acceleration'][matched_track_indices]
        detected_objects_array['acceleration'][matched_detection_indicies] = (1 - self.acceleration_gain) * old_accelerations + self.acceleration_gain * new_accelerations

        ### 5. UPDATE TRACKED OBJECTS ###

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
                detected_obj.velocity.linear.x, detected_obj.velocity.linear.y = detected_objects_array['velocity'][detection_idx]
                detected_obj.velocity_reliable = True
            if not detected_obj.acceleration_reliable:
                detected_obj.acceleration.linear.x, detected_obj.acceleration.linear.y = detected_objects_array['acceleration'][detection_idx]
                detected_obj.acceleration_reliable = True
            self.tracked_objects[track_idx] = detected_obj
        self.tracked_objects_array[['centroid', 'bbox', 'velocity', 'acceleration']][matched_track_indices] = \
            detected_objects_array[['centroid', 'bbox', 'velocity', 'acceleration']][matched_detection_indicies]

        ### 6. MANAGE TRACK STATUS ###

        # create missed track indices
        all_track_indices = np.arange(0, len(self.tracked_objects), 1, dtype=int)
        missed_track_indices = np.delete(all_track_indices, matched_track_indices)

        # create new detection indices
        all_detection_indices = np.arange(0, len(detected_objects), 1, dtype=int)
        new_detection_indices = np.delete(all_detection_indices, matched_detection_indicies)

        # zero missed counter and increase detected counter for matched objects
        self.tracked_objects_array['missed_counter'][matched_track_indices] = 0
        self.tracked_objects_array['detection_counter'][matched_track_indices] += 1

        # increase missed counter and zero detection counter for non-matched tracks
        self.tracked_objects_array['missed_counter'][missed_track_indices] += 1
        #self.tracked_objects_array['detection_counter'][missed_track_indices] = 0

        # delete stale tracks
        stale_track_indices = np.where(self.tracked_objects_array['missed_counter'] >= self.missed_counter_threshold)[0]
        for idx in sorted(stale_track_indices, reverse=True):
            del self.tracked_objects[idx]
        self.tracked_objects_array = np.delete(self.tracked_objects_array, stale_track_indices, axis=0)
        assert len(self.tracked_objects) == len(self.tracked_objects_array), str(len(self.tracked_objects)) + ' ' + str(len(self.tracked_objects_array))

        # add new detections
        for obj_idx in new_detection_indices:
            detected_obj = detected_objects[obj_idx]
            detected_obj.id = self.track_id_counter
            self.track_id_counter += 1
            self.tracked_objects.append(detected_obj)
        self.tracked_objects_array = np.concatenate((self.tracked_objects_array, detected_objects_array[new_detection_indices]))
        assert len(self.tracked_objects) == len(self.tracked_objects_array)

        ### 7. PUBLISH CURRENT TRACKS ###

        # filter out objects with enough detections
        tracked_objects_indices = np.where(self.tracked_objects_array['detection_counter'] >= self.detection_counter_threshold)[0]
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
