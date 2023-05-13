#!/usr/bin/env python3

from __future__ import print_function
from __future__ import division

import math
import numpy as np
import cv2

import rospy
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
from tf.transformations import quaternion_multiply, euler_from_quaternion

from std_msgs.msg import ColorRGBA
from vella_msgs.msg import Track3DArray
from autoware_msgs.msg import DetectedObjectArray, DetectedObject
from geometry_msgs.msg import PolygonStamped, Point, PoseStamped

LIGHT_BLUE = ColorRGBA(0.5, 0.5, 1.0, 0.8)
MPH_TO_MS_MULTIPLIER = 0.447 # if we find out that vella speeds are indeed given in mph then multiply our speed with this constant  to get speeds in m/s

class VellaDetector:
    def __init__(self):
        # Params
        self.confidence_filter = rospy.get_param("~confidence_filter") # filter out objects with score less than this threshold
        self.track_length_filter = rospy.get_param("~track_length_filter") # filter out objects with track length less than this threshold
        self.lidar_frame = rospy.get_param("~lidar_frame") # frame_id for tracks published by vella - vella does not populate frame_id of vdk/tracks messages
        self.output_frame = rospy.get_param("~output_frame")  # transform vella tracks from lidar frame to this frame
        self.transform_timeout = rospy.get_param("~transform_timeout")  # transform timeout when transforming poses

        # transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
        
        # Autoware detected objects publisher
        self.detected_object_array_pub = rospy.Publisher('detected_objects', DetectedObjectArray, queue_size=1)
        
        # vella tracks subscriber
        rospy.Subscriber('/vdk/tracks', Track3DArray, self.vella_tracks_callback, queue_size=1)

        rospy.loginfo("vella_detector - initialized")

    def vella_tracks_callback(self, vella_tracks):

        """
        Publish Autoware DetectedObjectArray from Vella tracks
        :param vella_tracks: vella_msgs Track3DArray
        :return: None
        """
        # create an autoware detected object array and populate the header using vella stamp and vella frame id
        detected_objects_array = DetectedObjectArray()
        detected_objects_array.header.frame_id = self.output_frame
        detected_objects_array.header.stamp = vella_tracks.header.stamp

        # get the transform from lidar_frame to output frame at the time when vella track was published
        transform = self.tf_buffer.lookup_transform(self.output_frame, self.lidar_frame, vella_tracks.header.stamp, rospy.Duration(self.transform_timeout))

        # loop over each track and discard the ones not satisfying our filtering criteria
        for vella_track in vella_tracks.detections:
            if vella_track.confidence < self.confidence_filter or vella_track.track_length < self.track_length_filter:
                continue
            # if filtering test passed, create the autoware detected object
            detected_object = self.generate_autoware_object_from_vella_track(vella_track, vella_tracks.header.stamp, transform)
            detected_objects_array.objects.append(detected_object)

        # publish the detected objects array
        self.detected_object_array_pub.publish(detected_objects_array)

    def generate_autoware_object_from_vella_track(self, vella_track, vella_stamp, transform):

        """
        Generate Autoware DetectedObject from Vella track
        :param vella_track: Vella track
        :param vella_stamp: time stamp at which the vella track was generated. Vella stamp isn't available with individual vella tracks so passed as function argument
        :param tf_matrix: 4x4 homogenous transformation matrix to go from lidar frame to output frame
        :param tf_rot: quaternion representing rotation to go from lidar frame to output frame
        :return: AutowareDetectedObject
        """

        # Initialize a single Autoware DetectedObject
        detected_object = DetectedObject()
        # assign frame_id as the output frame
        detected_object.header.frame_id = self.output_frame
        # stamp it with vella created stamp
        detected_object.header.stamp = vella_stamp
        detected_object.id = vella_track.id
        detected_object.label = vella_track.label
        detected_object.color = LIGHT_BLUE
        detected_object.valid = True

        # Transform position and orientation to output frame using the transformation saved when callback was received
        detected_object.pose = do_transform_pose(PoseStamped(pose=vella_track.pose.pose), transform).pose
        detected_object.pose_reliable = True

        # compute norm of velocity predicted by vella
        detected_object.velocity.linear.x = np.linalg.norm([
                                            vella_track.velocity.twist.linear.x,
                                            vella_track.velocity.twist.linear.y,
                                            vella_track.velocity.twist.linear.z])
        detected_object.velocity_reliable = True
        detected_object.acceleration_reliable = False

        # object dimensions
        detected_object.dimensions.x = vella_track.length
        detected_object.dimensions.y = vella_track.width
        detected_object.dimensions.z = vella_track.height

        # produce convex hull
        detected_object.convex_hull = self.create_hull(detected_object.pose, detected_object.dimensions, vella_stamp)

        return detected_object

    def create_hull(self, obj_pose, obj_dims, vella_stamp):

        """
        Produce convex hull for an object given its pose and dimensions
        :param obj_pose: geometry_msgs/Pose. Position and orientation of object
        :param obj_dims: Vector3 - length, width and height of object
        :param vella_stamp: Time stamp at which vella track was creatd
        :return: geometry_msgs/PolygonStamped
        """
        convex_hull = PolygonStamped()
        convex_hull.header.frame_id = self.output_frame
        convex_hull.header.stamp = vella_stamp

        # Taken from Autoware mini
        # compute heading angle from object's orientation
        _, _, heading  = euler_from_quaternion((obj_pose.orientation.x, obj_pose.orientation.y, obj_pose.orientation.z, obj_pose.orientation.w))

        # use cv2.boxPoints to get a rotated rectangle given the angle
        points = cv2.boxPoints((
                                (obj_pose.position.x, obj_pose.position.y),
                                (obj_dims.x, obj_dims.y),
                                math.degrees(heading)
                                ))
        convex_hull.polygon.points = [Point(x, y, obj_pose.position.z) for x, y in points]
        # Add the first polygon point to the list of points to make it 5 points in total and complete the loop
        convex_hull.polygon.points.append(convex_hull.polygon.points[0])

        return convex_hull

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('vella_to_autoware_converter', anonymous=True, log_level=rospy.INFO)
    node = VellaDetector()
    node.run()
