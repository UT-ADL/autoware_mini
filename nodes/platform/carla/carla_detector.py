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
import numpy as np

from geometry_msgs.msg import PolygonStamped, Point
from autoware_msgs.msg import DetectedObjectArray, DetectedObject
from tf.transformations import quaternion_matrix
from derived_object_msgs.msg import ObjectArray
from localization.SimulationToUTMTransformer import SimulationToUTMTransformer


CLASS_ID_TO_LABEL = {
    0: 'unknown',
    1: 'unknown_small',
    2: 'unknown_medium',
    3: 'unknown_big',
    4: 'pedestrian',
    5: 'bike',
    6: 'car',
    7: 'truck',
    8: 'motorcycle',
    9: 'other_vehicle',
    10: 'barrier',
    11: 'sign'
}

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
            object_msg.score = 1
            object_msg.valid = True
            object_msg.space_frame = 'map'
            object_msg.pose = obj.pose

            if self.use_offset:
                object_msg.pose = self.sim2utm_transformer.transform_pose(object_msg.pose)

            object_msg.dimensions.x = obj.shape.dimensions[0]
            object_msg.dimensions.y = obj.shape.dimensions[1]
            object_msg.dimensions.z = obj.shape.dimensions[2]
            object_msg.velocity.linear.x = math.sqrt(obj.twist.linear.x**2 + obj.twist.linear.y**2 + obj.twist.linear.z**2)
            object_msg.acceleration = obj.accel
            object_msg.convex_hull = self.produce_hull(object_msg.pose.position, object_msg.dimensions, object_msg.header, object_msg.pose.orientation)
            object_msg.pose_reliable = True
            object_msg.velocity_reliable = True
            object_msg.acceleration_reliable = True
            object_msg.valid = True

            objects_msg.objects.append(object_msg)

        # Publish converted detected objects
        self.detected_objects_pub.publish(objects_msg)

    def produce_hull(self, centroid, dim_vector, header, yaw_quaternion):
        """
        create hull for a given pose and dimension
        """

        convex_hull = PolygonStamped()
        convex_hull.header = header

        rot_mat = quaternion_matrix([yaw_quaternion.x, yaw_quaternion.y, yaw_quaternion.z, yaw_quaternion.w])
        halved_dim = [dim_vector.x/2, dim_vector.y/2, dim_vector.z/2]

        point_1 = [halved_dim[0], halved_dim[1], halved_dim[2]]
        point_2 = [- halved_dim[0], halved_dim[1], halved_dim[2]]
        point_3 = [-halved_dim[0], -halved_dim[1], halved_dim[2]]
        point_4 = [halved_dim[0],  - halved_dim[1], halved_dim[2]]

        corner_points = np.array([point_1, point_4, point_3, point_2, point_1])
        corner_points = np.concatenate((corner_points, np.ones((corner_points.shape[0], 1))), axis=1)
        rotated_corners = np.dot(rot_mat, corner_points.T).T

        convex_hull.polygon.points = [Point(centroid.x + x, centroid.y + y, centroid.z + z) for x, y, z, _ in rotated_corners]

        return convex_hull

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('carla_detector', log_level=rospy.INFO)
    node = CarlaDetector()
    node.run()
