#!/usr/bin/env python3

import rospy
import numpy as np
import scipy.optimize
import scipy.spatial.distance
import dynamic_reconfigure.server
from autoware_mini.cfg import EMATrackerConfig
from autoware_mini.msg import DetectedObjectArray
from autoware_mini.geometry import get_heading_from_vector, get_speed_from_velocity
from autoware_mini.detection import calculate_iou, get_axis_oriented_bounding_box, update_object_position_dimensions

class EMATracker:
    def __init__(self):

        # Parameters
        self.enable_initial_velocity_estimate = rospy.get_param('~enable_initial_velocity_estimate')
        self.enable_initial_acceleration_estimate = rospy.get_param('~enable_initial_acceleration_estimate')
        self.enable_missed_detection_propagation = rospy.get_param('~enable_missed_detection_propagation')
        self.detection_counter_threshold = rospy.get_param('~detection_counter_threshold')
        self.missed_counter_threshold = rospy.get_param('~missed_counter_threshold')
        self.iou_threshold = rospy.get_param('~iou_threshold')
        self.velocity_gain = rospy.get_param('~velocity_gain')
        self.acceleration_gain = rospy.get_param('~acceleration_gain')
        self.association_method = rospy.get_param('~association_method')
        self.max_euclidean_distance = rospy.get_param('~max_euclidean_distance')
        self.update_heading_bboxes = rospy.get_param('~update_heading_bboxes')
        self.update_heading_bboxes_min_speed = rospy.get_param('~update_heading_bboxes_min_speed')

        self.reconfigure_server = dynamic_reconfigure.server.Server(EMATrackerConfig, self.reconfigure_callback)

        self.tracked_objects = []
        self.tracked_objects_array = np.empty((0,), dtype=[
            ('centroid', np.float32, (2,)),
            ('bbox', np.float32, (4,)),
            ('velocity', np.float32, (2,)),
            ('acceleration', np.float32, (2,)),
            ('missed_counter', np.int32),
            ('detection_counter', np.int32),
            ('position_reliable', np.bool_),
        ])
        self.track_id_counter = 0
        self.stamp = None

        self.velocities = []
        self.object_counts = []
        rospy.on_shutdown(self.shutdown)

        # Publishers
        self.tracked_objects_pub = rospy.Publisher('tracked_objects', DetectedObjectArray, queue_size=1, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber('detected_objects', DetectedObjectArray, self.detected_objects_callback, queue_size=1, buff_size=2**20, tcp_nodelay=True)

    def reconfigure_callback(self, config, level):
        rospy.loginfo("%s - reconfiguring EMA tracker parameters", rospy.get_name())

        # Update parameters
        self.enable_initial_velocity_estimate = config.enable_initial_velocity_estimate
        self.enable_initial_acceleration_estimate = config.enable_initial_acceleration_estimate
        self.enable_missed_detection_propagation = config.enable_missed_detection_propagation
        self.detection_counter_threshold = config.detection_counter_threshold
        self.missed_counter_threshold = config.missed_counter_threshold
        self.iou_threshold = config.iou_threshold
        self.velocity_gain = config.velocity_gain
        self.acceleration_gain = config.acceleration_gain
        self.association_method = config.association_method
        self.max_euclidean_distance = config.max_euclidean_distance
        self.update_heading_bboxes = config.update_heading_bboxes
        
        return config

    def detected_objects_callback(self, msg):
        ### 1. PREPARE DETECTIONS ###

        # convert detected objects into Numpy array
        detected_objects = msg.objects
        detected_objects_array = np.empty((len(detected_objects)), dtype=self.tracked_objects_array.dtype)
        for i, obj in enumerate(detected_objects):
            detected_objects_array['centroid'][i] = (obj.centroid.x, obj.centroid.y)
            detected_objects_array['bbox'][i] = get_axis_oriented_bounding_box(obj)
            detected_objects_array['velocity'][i] = (obj.velocity.x, obj.velocity.y)
            detected_objects_array['acceleration'][i] = (obj.acceleration.x, obj.acceleration.y)
            detected_objects_array['missed_counter'][i] = 0
            detected_objects_array['detection_counter'][i] = 1
            detected_objects_array['position_reliable'][i] = obj.position_reliable
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

        if self.tracked_objects and detected_objects:
            if self.association_method == 'iou':
                # Calculate the IOU between the tracked objects and the detected objects
                iou = calculate_iou(tracked_object_bboxes, detected_objects_array['bbox'])
                assert iou.shape == (len(self.tracked_objects_array), len(detected_objects_array)), str(iou.shape) + ' ' + str((len(self.tracked_objects_array), len(detected_objects_array)))

                # Forbid matching pairs with IOU below threshold by setting cost to a large value
                iou[iou < self.iou_threshold] = -1e9

                # Calculate the association between the tracked objects and the detected objects
                matched_track_indices, matched_detection_indices = scipy.optimize.linear_sum_assignment(-iou)
                assert len(matched_track_indices) == len(matched_detection_indices)

                # Remove pairs that were assigned a large cost (below threshold)
                matches = iou[matched_track_indices, matched_detection_indices] >= self.iou_threshold
                matched_track_indices = matched_track_indices[matches]
                matched_detection_indices = matched_detection_indices[matches]
                assert len(matched_track_indices) == len(matched_detection_indices)

                # find unmatched detections that still overlap with existing tracks
                unmatched_detection_indices = np.delete(np.arange(len(detected_objects), dtype=int), matched_detection_indices)
                best_track_indices = iou[:, unmatched_detection_indices].argmax(axis=0)
                has_overlap = iou[best_track_indices, unmatched_detection_indices] >= self.iou_threshold
                best_track_indices = best_track_indices[has_overlap]
                unmatched_detection_indices = unmatched_detection_indices[has_overlap]

            elif self.association_method == 'euclidean':
                # Calculate euclidean distance between the tracked object and the detected object centroids
                dists = scipy.spatial.distance.cdist(tracked_object_centroids, detected_objects_array['centroid'])
                assert dists.shape == (len(self.tracked_objects_array), len(detected_objects_array))

                # Forbid matching pairs with distance above threshold by setting cost to a large value
                dists[dists > self.max_euclidean_distance] = 1e9

                # Calculate the association between the tracked objects and the detected objects
                matched_track_indices, matched_detection_indices = scipy.optimize.linear_sum_assignment(dists)

                # Remove pairs that were assigned a large cost (above threshold)
                matches = dists[matched_track_indices, matched_detection_indices] <= self.max_euclidean_distance
                matched_track_indices = matched_track_indices[matches]
                matched_detection_indices = matched_detection_indices[matches]
                assert len(matched_track_indices) == len(matched_detection_indices)

                # find unmatched detections that still overlap with existing tracks
                unmatched_detection_indices = np.delete(np.arange(len(detected_objects), dtype=int), matched_detection_indices)
                best_track_indices = dists[:, unmatched_detection_indices].argmin(axis=0)
                has_overlap = dists[best_track_indices, unmatched_detection_indices] <= self.max_euclidean_distance
                best_track_indices = best_track_indices[has_overlap]
                unmatched_detection_indices = unmatched_detection_indices[has_overlap]

            else:
                assert False, 'Unknown association method: ' + self.association_method

            # add overlapping unmatched detections to matched arrays for velocity inheritance
            n_hungarian = len(matched_track_indices)
            matched_track_indices = np.concatenate((matched_track_indices, best_track_indices))
            matched_detection_indices = np.concatenate((matched_detection_indices, unmatched_detection_indices))

            ### 4. ESTIMATE TRACKED OBJECT SPEEDS AND ACCELERATIONS ###

            # update tracked object speeds with exponential moving average
            new_velocities = (detected_objects_array['centroid'][matched_detection_indices] - self.tracked_objects_array['centroid'][matched_track_indices]) / time_delta
            old_velocities = self.tracked_objects_array['velocity'][matched_track_indices]
            if self.enable_initial_velocity_estimate:
                # make initial velocity of an object equal to its first velocity estimate instead of zero
                second_time_detections = self.tracked_objects_array['detection_counter'][matched_track_indices] == 1
                old_velocities[second_time_detections] = new_velocities[second_time_detections]
            detected_objects_array['velocity'][matched_detection_indices] = (1 - self.velocity_gain) * old_velocities + self.velocity_gain * new_velocities

            # update tracked object accelerations with exponential moving average
            new_accelerations = (new_velocities - old_velocities) / time_delta
            old_accelerations = self.tracked_objects_array['acceleration'][matched_track_indices]
            if self.enable_initial_acceleration_estimate:
                # make initial acceleration of an object equal to its first acceleration estimate instead of zero
                third_time_detections = self.tracked_objects_array['detection_counter'][matched_track_indices] == 2
                old_accelerations[third_time_detections] = new_accelerations[third_time_detections]
            detected_objects_array['acceleration'][matched_detection_indices] = (1 - self.acceleration_gain) * old_accelerations + self.acceleration_gain * new_accelerations

            # restore to Hungarian-only matches for steps 5 and 6
            matched_track_indices = matched_track_indices[:n_hungarian]
            matched_detection_indices = matched_detection_indices[:n_hungarian]

            ### 5. UPDATE TRACKED OBJECTS ###

            # Replace tracked objects with detected objects, keeping the same ID
            for track_idx, detection_idx in zip(matched_track_indices, matched_detection_indices):
                tracked_obj = self.tracked_objects[track_idx]
                detected_obj = detected_objects[detection_idx]
                detected_obj.id = tracked_obj.id

                detected_obj.velocity.x, detected_obj.velocity.y = detected_objects_array['velocity'][detection_idx].tolist()
                detected_obj.acceleration.x, detected_obj.acceleration.y = detected_objects_array['acceleration'][detection_idx].tolist()

                self.tracked_objects[track_idx] = detected_obj

            for field in ('centroid', 'bbox', 'velocity', 'acceleration'):
                self.tracked_objects_array[field][matched_track_indices] = detected_objects_array[field][matched_detection_indices]
        else:
            # no tracked objects or no detected objects, so no matches
            matched_track_indices = np.array([], dtype=int)
            matched_detection_indices = np.array([], dtype=int)

        ### 6. MANAGE TRACK STATUS ###

        # create missed track indices
        all_track_indices = np.arange(len(self.tracked_objects), dtype=int)
        missed_track_indices = np.delete(all_track_indices, matched_track_indices)

        # create new detection indices
        all_detection_indices = np.arange(len(detected_objects), dtype=int)
        new_detection_indices = np.delete(all_detection_indices, matched_detection_indices)

        # zero missed counter and increase detected counter for matched objects
        self.tracked_objects_array['missed_counter'][matched_track_indices] = 0
        self.tracked_objects_array['detection_counter'][matched_track_indices] += 1

        # increase missed counter and zero detection counter for non-matched tracks
        self.tracked_objects_array['missed_counter'][missed_track_indices] += 1
        #self.tracked_objects_array['detection_counter'][missed_track_indices] = 0

        # move missed tracks forward
        if self.enable_missed_detection_propagation:
            assert len(self.tracked_objects) == len(self.tracked_objects_array) == len(tracked_object_centroids) == len(tracked_object_bboxes)
            self.tracked_objects_array['centroid'][missed_track_indices] = tracked_object_centroids[missed_track_indices]
            self.tracked_objects_array['bbox'][missed_track_indices] = tracked_object_bboxes[missed_track_indices]
            for idx in missed_track_indices:
                obj = self.tracked_objects[idx]

                obj.centroid.x, obj.centroid.y = self.tracked_objects_array['centroid'][idx].tolist()
                convex_hull = np.array(obj.convex_hull).reshape(-1, 3)
                convex_hull[:, :2] += position_change[idx]
                obj.convex_hull = convex_hull.ravel().tolist()


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
        tracked_objects_indices = np.where(self.tracked_objects_array['position_reliable'] | (self.tracked_objects_array['detection_counter'] >= self.detection_counter_threshold))[0]
        tracked_objects = [self.tracked_objects[idx] for idx in tracked_objects_indices]
        assert len(tracked_objects) == len(tracked_objects_indices)

        ### 8. update tracked objects dimensions, position, heading based on velocity vector ###
        if self.update_heading_bboxes:
            for obj in tracked_objects:
                if get_speed_from_velocity(obj.velocity) >= self.update_heading_bboxes_min_speed:
                    obj.heading = get_heading_from_vector(obj.velocity)
                    update_object_position_dimensions(obj)

        # publish tracked objects
        tracked_objects_msg = DetectedObjectArray()
        tracked_objects_msg.header.stamp = msg.header.stamp
        tracked_objects_msg.header.frame_id = msg.header.frame_id
        tracked_objects_msg.objects = tracked_objects
        self.tracked_objects_pub.publish(tracked_objects_msg)

        # record statistics
        self.velocities.append(np.mean(np.linalg.norm(self.tracked_objects_array['velocity'], axis=1)))
        self.object_counts.append(len(tracked_objects))

    def run(self):
        rospy.spin()
    
    def shutdown(self):
        rospy.loginfo('%s - total tracked objects created: %d', rospy.get_name(), self.track_id_counter)
        rospy.loginfo('%s - average object velocity: %f', rospy.get_name(), np.mean(self.velocities))
        rospy.loginfo('%s - average object count: %f', rospy.get_name(), np.mean(self.object_counts))

if __name__ == '__main__':
    rospy.init_node('ema_tracker', log_level=rospy.INFO)
    node = EMATracker()
    node.run()
