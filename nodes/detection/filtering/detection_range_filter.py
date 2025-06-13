#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Point
from autoware_mini.msg import DetectedObjectArray
from autoware_mini.geometry import get_distance_between_two_points_2d, get_heading_from_orientation, get_point_using_heading_and_distance
from autoware_mini.transform import get_distance_to_car_front

class DetectionRangeFilter:
    def __init__(self):

        # get parameters
        self.detection_range = rospy.get_param("~detection_range")

        self.current_pose = None
        self.distance_to_car_front = get_distance_to_car_front()

        # detected objects publisher
        self.objects_pub = rospy.Publisher('detected_objects_filtered', DetectedObjectArray, queue_size=1, tcp_nodelay=True)

        # initial position and vehicle command from outside
        rospy.Subscriber('detected_objects', DetectedObjectArray, self.detected_objects_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1, tcp_nodelay=True)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def current_pose_callback(self, msg):
        self.current_pose = msg.pose

    def detected_objects_callback(self, msg):

        if self.current_pose is None:
            return

        # get location of car front
        base_link_point = Point(self.current_pose.position.x, self.current_pose.position.y, self.current_pose.position.z)
        heading = get_heading_from_orientation(self.current_pose.orientation)
        car_front = get_point_using_heading_and_distance(base_link_point, heading, self.distance_to_car_front)

        # Create array objects
        objects = DetectedObjectArray()
        objects.header = msg.header

        for obj in msg.objects:
            convex_hull = np.array(obj.convex_hull).reshape(-1, 3)
            for x, y, z in convex_hull:
                distance = get_distance_between_two_points_2d(car_front, Point(x, y, z))
                if distance < self.detection_range:
                    objects.objects.append(obj)
                    break

        self.objects_pub.publish(objects)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('detection_range_filter', log_level=rospy.INFO)
    node = DetectionRangeFilter()
    node.run()
