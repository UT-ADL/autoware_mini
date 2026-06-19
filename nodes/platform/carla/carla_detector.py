#!/usr/bin/env python3
#
# Copyright (c) 2023 Autonomous Driving Lab (ADL), University of Tartu.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
ground truth detections. Publishes the following topics:
    receive :derived_object_msgs::ObjectArray and publishes autoware_mini::DetectedObjectArray
"""
import rospy

from std_msgs.msg import ColorRGBA
from autoware_mini.msg import DetectedObjectArray, DetectedObject
from derived_object_msgs.msg import ObjectArray, Object
from autoware_mini.detection import create_hull
from autoware_mini.geometry import get_heading_from_orientation

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

YELLOW = ColorRGBA(1.0, 1.0, 0.0, 0.5)

class CarlaDetector:

    def __init__(self):

        # Node parameters
        self.output_frame = rospy.get_param("/perception/output_frame")

        # Publishers
        self.detected_objects_pub = rospy.Publisher(
            'detected_objects', DetectedObjectArray, queue_size=1, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber('/carla/ego_vehicle/objects',
                         ObjectArray, self.carla_objects_callback, queue_size=1, buff_size=2**20, tcp_nodelay=True)

    def carla_objects_callback(self, data):
        """
        callback for carla objects
        """

        objects_msg = DetectedObjectArray()
        objects_msg.header = data.header

        for obj in data.objects:

            object_msg = DetectedObject()
            object_msg.id = obj.id
            object_msg.label = CLASS_ID_TO_LABEL[obj.classification] 
            object_msg.color = YELLOW
            object_msg.score = 1
            object_msg.valid = True
            
            pose = obj.pose

            object_msg.centroid = object_msg.center = pose.position
            object_msg.heading = get_heading_from_orientation(pose.orientation)
            object_msg.dimensions.x = obj.shape.dimensions[0]
            object_msg.dimensions.y = obj.shape.dimensions[1]
            object_msg.dimensions.z = obj.shape.dimensions[2]
            object_msg.velocity = obj.twist.linear
            object_msg.acceleration = obj.accel.linear
            object_msg.convex_hull = create_hull(object_msg)
            object_msg.position_reliable = True
            object_msg.velocity_reliable = True
            object_msg.acceleration_reliable = True

            objects_msg.objects.append(object_msg)

        # Publish converted detected objects
        self.detected_objects_pub.publish(objects_msg)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('carla_detector', log_level=rospy.INFO)
    node = CarlaDetector()
    node.run()
