#!/usr/bin/env python3

import math
from collections import defaultdict
import traceback

import rospy
import tf2_ros

from geometry_msgs.msg import TwistStamped
from autoware_mini.msg import DetectedObject, DetectedObjectArray
from radar_msgs.msg import RadarTracks
from std_msgs.msg import ColorRGBA

from autoware_mini.detection import create_hull
from autoware_mini.transform import transform_to_matrix, transform_point_by_matrix, transform_vector3_by_matrix

RED = ColorRGBA(1.0, 0.0, 0.0, 0.7)
RADAR_CLASSIFICATION = {0:'unknown', 1:'static', 2:'dynamic'}


class RadarDetector:
    def __init__(self):

        # Parameters
        self.output_frame = rospy.get_param("/perception/output_frame")
        self.consistency_check = rospy.get_param("~consistency_check") # number of frames a radar detection is received before it is considered  true radar detection. Based on ID count
        self.radar_time_offset = rospy.get_param("~radar_time_offset")
        self.transform_timeout = rospy.get_param("~transform_timeout")

        # Internal variables
        self.current_linear_velocity = None
        self.uuid_count = defaultdict(int) # dictionary that keeps track of radar objects and their id count. Used for checking consistency of object ids in consistency filter
        self.id_counter = 0 # counter for generating id from uuids
        self.uuid_map = {} # dictionary containing uuid-integer id pairs
        self.object_pool = defaultdict(DetectedObject) # pool of DetectedObject messages to reduce memory allocations

        # Publishers
        self.detected_objs_pub = rospy.Publisher("detected_objects", DetectedObjectArray, queue_size=1, tcp_nodelay=True)

        # Dynamic transform listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Static transform, fetch once
        base_link_to_radar_tf = self.tf_buffer.lookup_transform('radar_fc', 'base_link', rospy.Time(0), rospy.Duration(1.0))
        self.base_link_to_radar_tf_matrix = transform_to_matrix(base_link_to_radar_tf)

        # Subscribers
        rospy.Subscriber('/radar_fc/radar_tracks', RadarTracks, self.radar_tracks_callback, queue_size=1, buff_size=2**20, tcp_nodelay=True)
        rospy.Subscriber('/localization/current_velocity', TwistStamped, self.current_velocity_callback, queue_size=1, tcp_nodelay=True)
        
        rospy.loginfo("%s - initialized", rospy.get_name())

    def current_velocity_callback(self, msg):
        self.current_linear_velocity = msg.twist.linear

    def radar_tracks_callback(self, tracks):
        """
        tracks: radar_msgs/RadarTracks
        publish: DetectedObjectArray
        """
        try:
            detected_objects_array = DetectedObjectArray()
            detected_objects_array.header.frame_id = self.output_frame
            detected_objects_array.header.stamp = tracks.header.stamp + rospy.Duration.from_sec(self.radar_time_offset)

            current_linear_velocity = self.current_linear_velocity
            # check if the current linear velocity is available
            if current_linear_velocity is None:
                self.detected_objs_pub.publish(detected_objects_array)
                return

            try:
                # read source frame to output frame transform
                source_frame_to_output_tf = self.tf_buffer.lookup_transform(self.output_frame, tracks.header.frame_id, detected_objects_array.header.stamp, rospy.Duration(self.transform_timeout))
            except (tf2_ros.TransformException, rospy.ROSTimeMovedBackwardsException) as e:
                rospy.logwarn("%s - %s", rospy.get_name(), e)
                return

            source_to_output_tf_matrix = transform_to_matrix(source_frame_to_output_tf)

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
                detected_object = self.object_pool[i]
                detected_object.id = integer_id
                detected_object.label = RADAR_CLASSIFICATION[track.classification]
                detected_object.color = RED
                detected_object.valid = True
                detected_object.centroid = detected_object.center = transform_point_by_matrix(track.position, source_to_output_tf_matrix)
                detected_object.heading = 0.0
                detected_object.position_reliable = False
                heading = math.atan2(track.position.y, track.position.x)
                detected_object.velocity = self.transform_velocity(track, heading, current_linear_velocity, source_to_output_tf_matrix)
                detected_object.velocity_reliable = True
                #detected_object.acceleration = self.transform_acceleration(track, heading, source_to_output_tf_matrix)
                detected_object.acceleration_reliable = False
                detected_object.dimensions = track.size
                detected_object.convex_hull = create_hull(detected_object)

                detected_objects_array.objects.append(detected_object)

            self.detected_objs_pub.publish(detected_objects_array)

        except Exception as e:
            rospy.logerr_throttle(10, "%s - Exception in callback: %s", rospy.get_name(), traceback.format_exc())

    def transform_acceleration(self, track, heading, source_frame_to_output_tf_matrix):
        acceleration = track.acceleration
        acceleration.x /= math.cos(heading)
        acceleration.y = 0.0
        acceleration.z = 0.0
        return transform_vector3_by_matrix(acceleration, source_frame_to_output_tf_matrix)

    def transform_velocity(self, track, heading, ego_velocity, source_frame_to_output_tf_matrix):
        # compute ego_velocity in radar_fc frame
        velocity = transform_vector3_by_matrix(ego_velocity, self.base_link_to_radar_tf_matrix)
        # correct track radial velocity with ego velocity and heading
        velocity.x += track.velocity.x / math.cos(heading)
        # transforming the velocity vector to the output frame
        return transform_vector3_by_matrix(velocity, source_frame_to_output_tf_matrix)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('radar_detector', log_level=rospy.INFO)
    node = RadarDetector()
    node.run()
