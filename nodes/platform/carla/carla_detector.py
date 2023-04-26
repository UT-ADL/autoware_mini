#!/usr/bin/env python
#
# Copyright (c) 2023 Autonomous Driving Lab (ADL), University of Tartu.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
ground truth detections. Publishes the following topics:
    receive :derived_object_msgs::ObjectArray and publishes autoware_msgs::DetectedObjectArray
"""
import rospy
import math
import cv2

from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import PolygonStamped, Point
from autoware_msgs.msg import DetectedObjectArray, DetectedObject
from tf.transformations import euler_from_quaternion
from derived_object_msgs.msg import ObjectArray, Object
from localization.SimulationToUTMTransformer import SimulationToUTMTransformer


CLASS_ID_TO_LABEL = {
    Object.CLASSIFICATION_UNKNOWN: 'unknown',
    Object.CLASSIFICATION_UNKNOWN_SMALL: 'unknown_small',
    Object.CLASSIFICATION_UNKNOWN_MEDIUM: 'unknown_medium',
    Object.CLASSIFICATION_UNKNOWN_BIG: 'unknown_big',
    Object.CLASSIFICATION_PEDESTRIAN: 'pedestrian',
    Object.CLASSIFICATION_BIKE: 'bike',
    Object.CLASSIFICATION_CAR: 'car',
    Object.CLASSIFICATION_TRUCK: 'truck',
    Object.CLASSIFICATION_MOTORCYCLE: 'motorcycle',
    Object.CLASSIFICATION_OTHER_VEHICLE: 'other_vehicle',
    Object.CLASSIFICATION_BARRIER: 'barrier',
    Object.CLASSIFICATION_SIGN: 'sign'
}

YELLOW80P = ColorRGBA(1.0, 1.0, 0.0, 0.8)

class CarlaDetector:

    def __init__(self):

        # Node parameters
        self.use_offset = rospy.get_param("~use_offset", default=True)
        use_custom_origin = rospy.get_param("/localization/use_custom_origin", True)
        utm_origin_lat = rospy.get_param("/localization/utm_origin_lat")
        utm_origin_lon = rospy.get_param("/localization/utm_origin_lon")

        # Internal parameters
        self.sim2utm_transformer = SimulationToUTMTransformer(use_custom_origin=use_custom_origin,
                                                              origin_lat=utm_origin_lat,
                                                              origin_lon=utm_origin_lon)

        # Publishers
        self.detected_objects_pub = rospy.Publisher(
            'detected_objects', DetectedObjectArray, queue_size=1)

        # Subscribers
        rospy.Subscriber('/carla/ground_truth_objects',
                         ObjectArray, self.carla_objects_callback, queue_size=1)

    def carla_objects_callback(self, data):
        """
        callback for carla objects
        """

        objects_msg = DetectedObjectArray()
        objects_msg.header = data.header

        for obj in data.objects:

            object_msg = DetectedObject()
            object_msg.header = obj.header
            object_msg.id = obj.id
            object_msg.label = CLASS_ID_TO_LABEL[obj.classification] 
            object_msg.color = YELLOW80P
            object_msg.score = 1
            object_msg.valid = True
            object_msg.space_frame = 'map'
            object_msg.pose = obj.pose

            if self.use_offset:
                object_msg.pose = self.sim2utm_transformer.transform_pose(object_msg.pose)

            object_msg.dimensions.x = obj.shape.dimensions[0]
            object_msg.dimensions.y = obj.shape.dimensions[1]
            object_msg.dimensions.z = obj.shape.dimensions[2]
            object_msg.velocity = obj.twist
            object_msg.acceleration = obj.accel
            object_msg.convex_hull = self.produce_hull(object_msg.pose, object_msg.dimensions, object_msg.header)
            object_msg.pose_reliable = True
            object_msg.velocity_reliable = True
            object_msg.acceleration_reliable = True
            object_msg.valid = True

            objects_msg.objects.append(object_msg)

        # Publish converted detected objects
        self.detected_objects_pub.publish(objects_msg)

    def produce_hull(self, obj_pose, obj_dims, header):
        """
        create hull for a given pose and dimension
        """

        convex_hull = PolygonStamped()
        convex_hull.header = header

        # compute heading angle from object's orientation
        _, _, heading  = euler_from_quaternion((obj_pose.orientation.x, obj_pose.orientation.y, obj_pose.orientation.z, obj_pose.orientation.w))

        # use cv2.boxPoints to get a rotated rectangle given the angle
        points = cv2.boxPoints((
                                (obj_pose.position.x, obj_pose.position.y),
                                (obj_dims.x, obj_dims.y),
                                math.degrees(heading)
                                ))
        convex_hull.polygon.points = [Point(x, y, obj_pose.position.z) for x, y in points]

        return convex_hull

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('carla_detector', log_level=rospy.INFO)
    node = CarlaDetector()
    node.run()
