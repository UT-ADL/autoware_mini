#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import tf
import copy
from geometry_msgs.msg import Vector3, Vector3Stamped, Twist, TwistStamped, PoseWithCovariance, PoseStamped, Point32, Quaternion, Point, PointStamped, PolygonStamped
from derived_object_msgs.msg import ObjectWithCovarianceArray
from autoware_msgs.msg import DetectedObject, DetectedObjectArray
from radar_msgs.msg import RadarTrack, RadarTracks
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from genpy import Duration
import message_filters


class radar_relay:

    def __init__(self):

        # Parameters
        self.output_frame = rospy.get_param("~target_frame", "map")
        self.consistency_check = rospy.get_param("~consistency_check", 5) # number of frames a radar detection is received before it is considered  true radar detection. Based on ID count
        self.id_count = {} # dictionary that keeps track of radar objects and their id count. Used for checking consistency of object ids in consistency filter

        # Subscribers and tf listeners
        tracks_sub = message_filters.Subscriber('/radar_fc/radar_tracks', RadarTracks)
        ego_speed_sub = message_filters.Subscriber('/localization/current_velocity', TwistStamped)

        self.tf_listener = tf.TransformListener()

        # Publishers
        self.detected_objs_pub = rospy.Publisher("detected_objects", DetectedObjectArray, queue_size=1)
        self.markers_pub = rospy.Publisher("detected_objects_markers", MarkerArray, queue_size=1)

        # Strict Time Sync
        ts = message_filters.ApproximateTimeSynchronizer([tracks_sub, ego_speed_sub], queue_size=5, slop=0.01)
        ts.registerCallback(self.syncronised_callback)

        rospy.loginfo(self.__class__.__name__ + " - node started")

    def syncronised_callback(self, tracks, ego_speed):
        """
        :type objects: ObjectWithCovarianceArray
        :type tracks: RadarTrackArray
        """
        detected_objs, markers = self.generate_detected_objects_array_and_markers(tracks, ego_speed)

        # Checks is there is something to publish before publishing
        if detected_objs is not None and markers is not None:
            self.detected_objs_pub.publish(detected_objs)
            self.markers_pub.publish(markers)

    def is_consistent(self, track_id):
        if track_id not in self.id_count.keys():
            self.id_count[track_id] = 1
            return False

        self.id_count[track_id] += 1
        return self.id_count[track_id] > self.consistency_check

    def remove_old_tracks(self, id_list):
        lost_tracks = self.id_count.keys() - id_list
        self.remove_entries_from_dict(list(lost_tracks))

    def remove_entries_from_dict(self, entries):
        for key in entries:
            if key in self.id_count:
                del self.id_count[key]

    def generate_detected_objects_array_and_markers(self, tracks, ego_speed):
        """
        :param objects: Array of ObjectWithCovariance
        :param tracks: Array of RadarTrack
        :param source_frame: Source frame of the messages, here it's the radar
        :param ego_speed: speed of the ego vehicle
        :type objects: ObjectWithCovarianceArray
        :type tracks: RadarTrackArray
        :type source_frame: str
        :returns: DetectedObjectArray and MarkerArray tfed to the output_frame class variable
        """

        # palceholders for markers and detected objects
        markers = MarkerArray()
        detected_objects_array = DetectedObjectArray()
        detected_objects_array.header.frame_id = self.output_frame
        detected_objects_array.header.stamp = tracks.header.stamp

        # frame_id of recieved objects
        source_frame = tracks.header.frame_id

        # removing tracks that have been lost (ids not detected anymore)
        # not sure about byteorder
        id_list = [int.from_bytes(track.uuid.uuid[:3], byteorder='big', signed=False) for track in tracks.tracks]
        self.remove_old_tracks(id_list)

        for track in tracks.tracks:  # type: RadarTrack

            ## Check if the radar id is consstent over a of frames. Dictated by the param named consistency_check
            if not self.is_consistent(int.from_bytes(track.uuid.uuid[:3], byteorder='big', signed=False)):
                continue

            # Detected object
            detected_object = DetectedObject()
            detected_object.header.frame_id = self.output_frame
            detected_object.header.stamp = tracks.header.stamp
            detected_object.id = int.from_bytes(track.uuid.uuid[:3], byteorder='big', signed=False)
            detected_object.label = 'unknown'
            detected_object.valid = True
            detected_object.pose_reliable = True
            detected_object.pose = self.get_tfed_pose(track.position, source_frame)
            detected_object.velocity_reliable = True
            detected_object.velocity = self.get_vel_vector_in_map(track, ego_speed, source_frame)
            detected_object.acceleration_reliable = True
            detected_object.acceleration = Twist(linear=self.get_tfed_vector3(track.acceleration, source_frame))
            detected_object.dimensions = track.size
            detected_object.convex_hull = self.produce_hull(detected_object.pose, detected_object.dimensions, tracks.header.stamp)

            # Marker for rviz visualisation
            markers.markers += self.generate_markers(detected_object)
            detected_objects_array.objects.append(detected_object)

        return detected_objects_array, markers

    def generate_markers(self, detected_object):

        marker_list = []

        centroid = Marker()
        centroid.header.frame_id = self.output_frame
        centroid.ns = "radar_centroid"
        centroid.type = centroid.SPHERE
        centroid.action = centroid.ADD
        centroid.scale = Vector3(x=1., y=1., z=1.)
        centroid.color = ColorRGBA(r=1., g=0.08, b=0.58, a=1.)
        centroid.pose.position = detected_object.pose.position
        centroid.pose.orientation = Quaternion(w=1., x=0., y=0., z=0.)
        centroid.id = detected_object.id
        centroid.lifetime = Duration(secs=0.1)

        text = Marker()
        text.header.frame_id = self.output_frame
        text.ns = "radar_text"
        text.type = text.TEXT_VIEW_FACING
        text.action = text.ADD
        text.scale = Vector3(x=0., y=0., z=1.)
        text.color = ColorRGBA(r=1., g=1., b=1., a=1.)
        text.pose.position = copy.copy(detected_object.pose.position)
        text.pose.position.x += 2.5
        text.pose.orientation = Quaternion(w=1., x=0., y=0., z=0.)
        # printing the norm of a radar object's velocity in the label to ensure it matches with the values published by fusion node. Will come in handy when debugging
        text.text = "#%02d_(%05.2f)" % (detected_object.id,
                                        np.linalg.norm((detected_object.velocity.linear.x, detected_object.velocity.linear.y, detected_object.velocity.linear.z)))  # Id & speed
        text.id = detected_object.id + 1000
        text.lifetime = Duration(secs=0.1)

        marker_list += [centroid, text]
        return marker_list

    def get_vel_vector_in_map(self, track, ego_speed, source_frame):

        """
        :param track: radar track message
        :type track: RadarTrack
        :param ego_speed: speed of our vehicle
        :type ego_speed: std_msgs/TwistStamped
        :param source_frame: frame in which to transform the velocity vector. Map in  this case
        :type source_frame: string
        :return: velocity vector transformed to map frame
        :rtype: std_msgs/Twist
        """

        # Computing speed relative to map.
        velocity_x = ego_speed.twist.linear.x + track.velocity.x
        velocity_y = ego_speed.twist.linear.y + track.velocity.y # this value is zero for both ego and track velocity
        velocity_z = ego_speed.twist.linear.z + track.velocity.z # this value is zero for both ego and track velocity

        velocity_vector = Vector3(velocity_x, velocity_y, velocity_z)

        # transforming the velocity vector to map frame
        velocity_in_map = Twist(linear = self.get_tfed_vector3(velocity_vector, source_frame))

        return velocity_in_map

    def get_tfed_pose(self, position, source_frame):
        """
        :type pose_with_cov: PoseWithCovariance
        :type source_frame: str
        :returns: tfed Pose to the output_frame
        """

        # To apply a TF we need a pose stamped
        pose_stamped = PoseStamped()
        pose_stamped.pose.position = Point(x=position.x, y=position.y, z=position.z )
        pose_stamped.header.frame_id = source_frame
        tfed_pose = self.tf_listener.transformPose(self.output_frame, pose_stamped).pose
        return tfed_pose

    def get_tfed_vector3(self, vector3, source_frame):
        """
        :type vector3: Vector3
        :type source_frame: str
        :returns: tfed Vector3 to the output_frame class variable
        """

        # To apply a TF we need a Vector3 stamped
        vector3_stamped = Vector3Stamped(vector=vector3)
        vector3_stamped.header.frame_id = source_frame
        tfed_vector3 = self.tf_listener.transformVector3(self.output_frame, vector3_stamped).vector
        return tfed_vector3

    def get_tfed_point32(self, p32, source_frame):
        """

        :param p32: point to be transformed
        :type p32:
        :param source_frame: radar_frame
        :type source_frame: string
        :return: transformed Point32
        :rtype: GeometryMsgs Point32
        """
        point_stamped = PointStamped()
        point_stamped.header.frame_id = source_frame
        point_stamped.point = Point(x=p32.x, y=p32.y, z=p32.z)
        tfed_point = self.tf_listener.transformPoint(self.output_frame, point_stamped).point
        return Point32(x=tfed_point.x, y=tfed_point.y, z=tfed_point.z)

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
        points = cv2.boxPoints((
            (obj_pose.position.x, obj_pose.position.y),
            (obj_dims.x, obj_dims.y), 0)) # input angle as 0 because the rada driver outputs pose.orientation as (1,0,0,0)
        convex_hull.polygon.points = [Point(x, y, obj_pose.position.z) for x, y in points]

        return convex_hull

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('radar_relay', anonymous=True, log_level=rospy.INFO)
    node = radar_relay()
    node.run()
