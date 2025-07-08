#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Point
from autoware_msgs.msg import DetectedObjectArray
from helpers.geometry import get_point_using_heading_and_distance, get_heading_from_orientation, get_distance_between_two_points_2d

class DetectionRangeFilter:
    def __init__(self):

        # get parameters
        self.detection_range = rospy.get_param("~detection_range")
        self.current_pose_to_car_front = rospy.get_param("/planning/current_pose_to_car_front")

        self.current_pose = None

        # detected objects publisher
        self.objects_pub = rospy.Publisher('detected_objects', DetectedObjectArray, queue_size=1, tcp_nodelay=True)

        # initial position and vehicle command from outside
        rospy.Subscriber('detected_objects_unfiltered', DetectedObjectArray, self.detected_objects_callback, queue_size=1, tcp_nodelay=True)
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
        car_front = get_point_using_heading_and_distance(base_link_point, heading, self.current_pose_to_car_front)

        # Create array objects
        objects = DetectedObjectArray()
        objects.header = msg.header

        for obj in msg.objects:
            for point in obj.convex_hull.polygon.points:
                distance = get_distance_between_two_points_2d(car_front, point)
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
