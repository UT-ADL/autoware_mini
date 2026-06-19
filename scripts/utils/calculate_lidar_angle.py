#!/usr/bin/env python3

import argparse
import math
import rospy
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped

parser = argparse.ArgumentParser()
parser.add_argument("camera_frame", default="camera_fl")
parser.add_argument("lidar_frame", default="lidar_front/os_sensor")
parser.add_argument("--distance", type=float, default=50.0)
args = parser.parse_args()

rospy.init_node('calculate_lidar_angle', anonymous=True)

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)
transform = tf_buffer.lookup_transform(args.lidar_frame, args.camera_frame, rospy.Time(0), rospy.Duration(30.0))

point = PointStamped()
point.header.frame_id = args.camera_frame
point.header.stamp = rospy.Time(0)
point.point.x = 0.0
point.point.y = 0.0
point.point.z = args.distance

print("Original point in camera frame:")
print(point)

transformed_point = do_transform_point(point, transform)
print("Transformed point in lidar frame:")
print(transformed_point)

angle = math.atan2(transformed_point.point.y, transformed_point.point.x)
angle = angle % (2 * math.pi)  # Normalize angle to [0, 2π)
print(f"Yaw angle in lidar frame: {angle} radians, {math.degrees(angle)} degrees")
print(f"Time from lidar scan start: {100 * angle / (2 * math.pi)} ms")