#!/usr/bin/env python3

from collections import defaultdict
import traceback

import rospy
import message_filters
from tf2_ros import Buffer, TransformListener, TransformException

from geometry_msgs.msg import TwistStamped
from autoware_msgs.msg import DetectedObject, DetectedObjectArray
from radar_msgs.msg import RadarTracks
from std_msgs.msg import ColorRGBA

from helpers.detection import create_hull
from helpers.transform import transform_point, transform_vector3

RED = ColorRGBA(1.0, 0.0, 0.0, 0.8)
RADAR_CLASSIFICATION = {0:'unknown', 1:'static', 2:'dynamic'}


class RadarDetector:
    def __init__(self):

        # Parameters
        self.output_frame = rospy.get_param("/detection/output_frame")
        self.consistency_check = rospy.get_param("~consistency_check") # number of frames a radar detection is received before it is considered  true radar detection. Based on ID count
        self.transform_timeout = rospy.get_param("~transform_timeout")

        # Internal variables
        self.uuid_count = defaultdict(int) # dictionary that keeps track of radar objects and their id count. Used for checking consistency of object ids in consistency filter
        self.id_counter = 0 # counter for generating id from uuids
        self.uuid_map = {} # dictionary containing uuid-integer id pairs

        # Publishers
        self.detected_objs_pub = rospy.Publisher("detected_objects", DetectedObjectArray, queue_size=1)

        # Dynamic transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
        # allow time for tf buffer to fill
        rospy.sleep(0.5)

        # Static transform, fetch once
        self.base_link_to_radar_tf = self.tf_buffer.lookup_transform('radar_fc', 'base_link', rospy.Time(0))

        # Subscribers
        tracks_sub = message_filters.Subscriber('/radar_fc/radar_tracks', RadarTracks, queue_size=1)
        ego_speed_sub = message_filters.Subscriber('/localization/current_velocity', TwistStamped, queue_size=1)

        # Strict Time Sync
        ts = message_filters.ApproximateTimeSynchronizer([tracks_sub, ego_speed_sub], queue_size=5, slop=0.02)
        ts.registerCallback(self.syncronised_callback)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def syncronised_callback(self, tracks, ego_speed):
        """
        tracks: radar_msgs/RadarTracks
        ego_speed: geometry_msgs/TwistStamped
        publish: DetectedObjectArray
        """
        try:
            detected_objects_array = DetectedObjectArray()
            detected_objects_array.header.frame_id = self.output_frame
            detected_objects_array.header.stamp = tracks.header.stamp

            try:
                # read source frame to output frame transform
                source_frame_to_output_tf = self.tf_buffer.lookup_transform(self.output_frame, tracks.header.frame_id, tracks.header.stamp, rospy.Duration(self.transform_timeout))
            except (TransformException, rospy.ROSTimeMovedBackwardsException) as e:
                rospy.logwarn("%s - %s", rospy.get_name(), e)
                return

            for i, track in enumerate(tracks.tracks):  # type: radar_msgs/RadarTrack
                # generate integer id from uuid
                uuid = track.uuid.uuid
                if uuid not in self.uuid_map:
                    integer_id = self.id_counter
                    self.uuid_map[uuid] = integer_id
                    # wrap around ids at 1 million
                    self.id_counter = (self.id_counter + 1) % 1000000
                else:
                    integer_id = self.uuid_map[uuid]

                # Check if the radar id is consistent over a few frames. Number of frames is dictated by the param named consistency_check
                self.uuid_count[uuid] += 1
                if self.uuid_count[uuid] < self.consistency_check:
                    continue

                # Detected object
                detected_object = DetectedObject()
                detected_object.header.frame_id = self.output_frame
                detected_object.header.stamp = tracks.header.stamp
                detected_object.id = integer_id
                detected_object.label = RADAR_CLASSIFICATION[track.classification]
                detected_object.color = RED
                detected_object.valid = True
                detected_object.pose.position = transform_point(track.position, source_frame_to_output_tf)
                detected_object.pose.orientation.w = 1.0
                detected_object.pose_reliable = True
                detected_object.velocity.linear = self.transform_velocity(track.velocity, ego_speed.twist.linear, source_frame_to_output_tf)
                detected_object.velocity_reliable = True
                detected_object.acceleration.linear = transform_vector3(track.acceleration, source_frame_to_output_tf)
                detected_object.acceleration_reliable = True
                detected_object.dimensions = track.size
                detected_object.convex_hull = create_hull(detected_object.pose, detected_object.dimensions, self.output_frame, tracks.header.stamp)

                detected_objects_array.objects.append(detected_object)

            self.detected_objs_pub.publish(detected_objects_array)

        except Exception as e:
            rospy.logerr_throttle(10, "%s - Exception in callback: %s", rospy.get_name(), traceback.format_exc())

    def transform_velocity(self, track_velocity, ego_velocity, source_frame_to_output_tf):
        # compute ego_velocity in radar_fc frame
        velocity = transform_vector3(ego_velocity, self.base_link_to_radar_tf)
        # Computing speed relative to map.
        velocity.x += track_velocity.x
        velocity.y += track_velocity.y # this value is zero for track velocity
        velocity.z += track_velocity.z # this value is zero for track velocity
        # transforming the velocity vector to the output frame
        return transform_vector3(velocity, source_frame_to_output_tf)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('radar_detector', log_level=rospy.INFO)
    node = RadarDetector()
    node.run()
