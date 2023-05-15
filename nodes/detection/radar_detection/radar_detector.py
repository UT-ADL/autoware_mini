#!/usr/bin/env python3

import cv2
from collections import defaultdict

import rospy
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
import message_filters

from geometry_msgs.msg import Vector3, Vector3Stamped, Twist, TwistStamped, PoseStamped, Point, PolygonStamped
from autoware_msgs.msg import DetectedObject, DetectedObjectArray
from radar_msgs.msg import RadarTracks
from std_msgs.msg import ColorRGBA

RED = ColorRGBA(1.0, 0.0, 0.0, 0.8)
RADAR_CLASSIFICATION = {0: 'unknown', 1:'static', 2:'dynamic'}


class RadarDetector:
    def __init__(self):

        # Parameters
        self.output_frame = rospy.get_param("/detection/output_frame")
        self.consistency_check = rospy.get_param("~consistency_check") # number of frames a radar detection is received before it is considered  true radar detection. Based on ID count
        self.uuid_count = defaultdict(int) # dictionary that keeps track of radar objects and their id count. Used for checking consistency of object ids in consistency filter
        self.id_counter = 0 # counter for generating id from uuids
        self.uuid_map = {} # dictionary containing uuid-integer id pairs

        # Publishers
        self.detected_objs_pub = rospy.Publisher("detected_objects", DetectedObjectArray, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # Allowing to buffer to fill up
        rospy.sleep(0.5)

        # static transform. Fetch once
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
        detected_objects_array = DetectedObjectArray()
        detected_objects_array.header.frame_id = self.output_frame
        detected_objects_array.header.stamp = tracks.header.stamp

        # frame_id of recieved tracks
        source_frame = tracks.header.frame_id

        for i, track in enumerate(tracks.tracks):  # type: radar_msgs/RadarTrack
            # generate integer id from uuid
            uuid = track.uuid.uuid
            if uuid not in self.uuid_map:
                integer_id = self.id_counter
                self.uuid_map[uuid] = integer_id
                self.id_counter +=1
            else:
                integer_id = self.uuid_map[uuid]

            ## how to remove a lost track here?
            ## how do we check if a track which was previously there isn't there anymore until we've seen all the new tracks?
            # removing tracks that have been lost (ids not detected anymore)
            # self.remove_old_tracks(id_list)

            ## Check if the radar id is consistent over a few frames. Number of frames is dictated by the param named consistency_check
            self.uuid_count[uuid] += 1
            if not self.uuid_count[uuid] >= self.consistency_check:
                continue

            # Detected object
            detected_object = DetectedObject()
            detected_object.header.frame_id = self.output_frame
            detected_object.header.stamp = tracks.header.stamp
            detected_object.id = integer_id
            detected_object.label = RADAR_CLASSIFICATION[track.classification]
            detected_object.color = RED
            detected_object.valid = True
            detected_object.pose_reliable = True
            detected_object.pose = self.get_tfed_pose(track.position, source_frame)
            detected_object.velocity_reliable = True
            detected_object.velocity.linear = self.get_tfed_velocity(track.velocity, ego_speed.twist.linear, source_frame)
            detected_object.acceleration_reliable = True
            detected_object.acceleration.linear = self.get_tfed_vector3(track.acceleration, source_frame)
            detected_object.dimensions = track.size
            detected_object.convex_hull = self.produce_hull(detected_object.pose, detected_object.dimensions, tracks.header.stamp)

            detected_objects_array.objects.append(detected_object)

        self.detected_objs_pub.publish(detected_objects_array)

    def get_tfed_velocity(self, track_velocity, ego_speed, source_frame):

        """
        track: radar track (radar_msgs/RadarTrack)
        :param ego_speed: speed of our vehicle (geometry_msgs/TwistStamped)
        :param source_frame: frame in which to transform the velocity vector. Map in  most cases unless specifically required and changed
        :return: velocity vector transformed to map frame (geometry_msgs/Twist)
        """
        # compute ego_velocity in radar_fc frame
        ego_speed_in_base = Vector3Stamped(vector=ego_speed)
        ego_speed_in_radar_fc = tf2_geometry_msgs.do_transform_vector3(ego_speed_in_base, self.base_link_to_radar_tf)
        # Computing speed relative to map.
        velocity_x = ego_speed_in_radar_fc.vector.x + track_velocity.x
        velocity_y = ego_speed_in_radar_fc.vector.y + track_velocity.y # this value is zero for both ego and track velocity
        velocity_z = ego_speed_in_radar_fc.vector.z + track_velocity.z # this value is zero for both ego and track velocity
        velocity_vector = Vector3(velocity_x, velocity_y, velocity_z)

        # transforming the velocity vector to the output frame
        return self.get_tfed_vector3(velocity_vector, source_frame)

    def get_tfed_pose(self, position, source_frame):
        """
        :type pose_with_cov: PoseWithCovariance
        :type source_frame: str
        :returns: tfed Pose to the output_frame
        """
        # To apply a TF we need a pose stamped
        pose_stamped = PoseStamped()
        pose_stamped.pose.position = position
        source_frame_to_output_tf = self.tf_buffer.lookup_transform(self.output_frame, source_frame, rospy.Time(0))
        tfed_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, source_frame_to_output_tf).pose
        return tfed_pose

    def get_tfed_vector3(self, vector3, source_frame):
        """
        :type vector3: Vector3
        :type source_frame: str
        :returns: tfed Vector3 to the output_frame class variable
        """
        # To apply a TF we need a Vector3 stamped
        vector3_stamped = Vector3Stamped(vector=vector3)
        source_frame_to_output_tf = self.tf_buffer.lookup_transform(self.output_frame, source_frame, rospy.Time(0))
        tfed_vector3 = tf2_geometry_msgs.do_transform_vector3(vector3_stamped, source_frame_to_output_tf).vector
        return tfed_vector3

    def produce_hull(self, obj_pose, obj_dims, stamp):

        """
        Produce convex hull for an object given its pose and dimensions
        :param obj_pose: geometry_msgs/Pose. Position and orientation of object
        :param obj_dims: Vector3 - length, width and height of object
        :param vella_stamp: Time stamp at which the lidar pointcloud was created
        :return: geometry_msgs/PolygonStamped
        """
        convex_hull = PolygonStamped()
        convex_hull.header.frame_id = self.output_frame
        convex_hull.header.stamp = stamp

        # use cv2.boxPoints to get a rotated rectangle given the angle
        _, _, heading = euler_from_quaternion(
            (obj_pose.orientation.x, obj_pose.orientation.y, obj_pose.orientation.z, obj_pose.orientation.w))
        points = cv2.boxPoints((
            (obj_pose.position.x, obj_pose.position.y),
            (obj_dims.x, obj_dims.y), heading)) # input angle as 0 because the radar driver outputs pose.orientation as (1,0,0,0)
        convex_hull.polygon.points = [Point(x, y, obj_pose.position.z) for x, y in points]

        return convex_hull
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('radar_detector', log_level=rospy.INFO)
    node = RadarDetector()
    node.run()
