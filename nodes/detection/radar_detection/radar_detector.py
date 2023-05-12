#!/usr/bin/env python3

import rospy
from collections import defaultdict
import cv2
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Vector3, Vector3Stamped, Twist, TwistStamped, PoseWithCovariance, PoseStamped, Point,PolygonStamped
from autoware_msgs.msg import DetectedObject, DetectedObjectArray
from radar_msgs.msg import RadarTrack, RadarTracks
from std_msgs.msg import ColorRGBA
import message_filters

RED = ColorRGBA(1.0, 0.0, 0.0, 0.8)


class RadarDetector:
    def __init__(self):

        # Parameters
        self.output_frame = rospy.get_param("~output_frame", "map")
        self.consistency_check = rospy.get_param("~consistency_check", 5) # number of frames a radar detection is received before it is considered  true radar detection. Based on ID count
        self.id_count = defaultdict(int) # dictionary that keeps track of radar objects and their id count. Used for checking consistency of object ids in consistency filter
        self.id_counter = 0 # counter for generating id from uuids
        self.uuid_map = {}

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

        rospy.loginfo(self.__class__.__name__ + " - Initialized")

    def syncronised_callback(self, tracks, ego_speed):
        """
        tracks: radar_msgs/RadarTracks
        ego_speed: geometry_msgs/TwistStamped
        """
        detected_objs = self.generate_detected_objects_array(tracks, ego_speed)
        # Checks is there is something to publish before publishing
        if detected_objs is not None:
            self.detected_objs_pub.publish(detected_objs)

    def is_consistent(self, track_id):
        self.id_count[track_id] += 1
        return self.id_count[track_id] >= self.consistency_check

    def remove_old_tracks(self, id_list):
        lost_tracks = self.id_count.keys() - id_list
        for key in list(lost_tracks):
            if key in self.id_count:
                del self.id_count[key]

    def generate_detected_objects_array(self, tracks, ego_speed):
        """
        tracks: radar_msgs/RadarTracks
        ego_speed: speed of the ego vehicle (geometry_msgs/TwistStamped)
        returns: array of autoware Detected Objects (DetectedObjectArray) tfed to the output_frame class variable
        """
        # placeholders for detected objects
        detected_objects_array = DetectedObjectArray()
        detected_objects_array.header.frame_id = self.output_frame
        detected_objects_array.header.stamp = tracks.header.stamp

        # frame_id of recieved objects
        source_frame = tracks.header.frame_id
        # converting uuids to integer values
        id_list = []
        for track in tracks.tracks:
            uuid = track.uuid.uuid
            if uuid not in self.uuid_map:
                id = self.id_counter
                self.uuid_map[uuid] = id
                self.id_counter +=1
            else:
                id = self.uuid_map[uuid]
            id_list.append(id)
        # removing tracks that have been lost (ids not detected anymore)
        self.remove_old_tracks(id_list)

        for i, track in enumerate(tracks.tracks):  # type: RadarTrack
            ## Check if the radar id is consistent over a few frames. Number of frames is dictated by the param named consistency_check
            if not self.is_consistent(id_list[i]):
                continue
            # Detected object
            detected_object = DetectedObject()
            detected_object.header.frame_id = self.output_frame
            detected_object.header.stamp = tracks.header.stamp
            detected_object.id = id_list[i]
            detected_object.label = 'unknown'
            detected_object.color = RED
            detected_object.valid = True
            detected_object.pose_reliable = True
            detected_object.pose = self.get_tfed_pose(track.position, source_frame)
            detected_object.velocity_reliable = True
            detected_object.velocity = self.get_tfed_velocity(track, ego_speed, source_frame)
            detected_object.acceleration_reliable = True
            detected_object.acceleration = Twist(linear=self.get_tfed_vector3(track.acceleration, source_frame))
            detected_object.dimensions = track.size
            detected_object.convex_hull = self.produce_hull(detected_object.pose, detected_object.dimensions, tracks.header.stamp)

            detected_objects_array.objects.append(detected_object)
        return detected_objects_array

    def get_tfed_velocity(self, track, ego_speed, source_frame):

        """
        track: radar track (radar_msgs/RadarTrack)
        :param ego_speed: speed of our vehicle (geometry_msgs/TwistStamped)
        :param source_header: frame in which to transform the velocity vector. Map in  most cases unless specifically required and changed
        :return: velocity vector transformed to map frame (geometry_msgs/Twist)
        """

        # compute ego_velocity in radar_fc frame
        ego_speed_in_base = Vector3Stamped(vector=ego_speed.twist.linear)
        ego_speed_in_radar_fc = tf2_geometry_msgs.do_transform_vector3(ego_speed_in_base, self.base_link_to_radar_tf)
        # Computing speed relative to map.
        velocity_x = ego_speed_in_radar_fc.vector.x + track.velocity.x
        velocity_y = ego_speed_in_radar_fc.vector.y + track.velocity.y # this value is zero for both ego and track velocity
        velocity_z = ego_speed_in_radar_fc.vector.z + track.velocity.z # this value is zero for both ego and track velocity
        velocity_vector = Vector3(velocity_x, velocity_y, velocity_z)

        # transforming the velocity vector to map frame
        velocity_in_map = Twist(linear=self.get_tfed_vector3(velocity_vector, source_frame))

        return velocity_in_map

    def get_tfed_pose(self, position, source_frame):
        """
        :type pose_with_cov: PoseWithCovariance
        :type source_header: str
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
        :type source_header: str
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
    rospy.init_node('radar_detector', anonymous=True, log_level=rospy.INFO)
    node = RadarDetector()
    node.run()
