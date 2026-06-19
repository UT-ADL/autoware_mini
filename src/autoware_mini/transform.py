import math

import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import Point, Vector3, Pose, Quaternion, Transform, TransformStamped
from tf import transformations

def transform_point(transform, point):
    mat = transform_to_matrix(transform)
    return transform_point_by_matrix(point, mat)

def transform_vector3(transform, vector3):
    mat = transform_to_matrix(transform)
    return transform_vector3_by_matrix(vector3, mat)

def transform_pose(transform, pose):
    tf_mat = transform_to_matrix(transform)
    pose_mat = transformations.quaternion_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    pose_mat[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    result = tf_mat @ pose_mat
    tx, ty, tz = transformations.translation_from_matrix(result).tolist()
    qx, qy, qz, qw = transformations.quaternion_from_matrix(result).tolist()
    return Pose(position=Point(x=tx, y=ty, z=tz), orientation=Quaternion(x=qx, y=qy, z=qz, w=qw))

def transform_to_pose(transform):
    """Convert a ROS TransformStamped to a Pose."""
    t = transform.transform.translation
    return Pose(position=Point(x=t.x, y=t.y, z=t.z), orientation=transform.transform.rotation)

def pose_to_transform(pose):
    """Convert a Pose to a ROS TransformStamped (header left unset)."""
    p = pose.position
    return TransformStamped(transform=Transform(translation=Vector3(x=p.x, y=p.y, z=p.z), rotation=pose.orientation))

def transform_to_matrix(transform):
    """Convert a ROS TransformStamped to a 4x4 homogeneous matrix."""
    t = transform.transform.translation
    r = transform.transform.rotation
    mat = transformations.quaternion_matrix([r.x, r.y, r.z, r.w])
    mat[:3, 3] = [t.x, t.y, t.z]
    return mat

def transform_point_by_matrix(point, mat):
    x, y, z, _ = (mat @ np.array([point.x, point.y, point.z, 1])).tolist()
    return Point(x=x, y=y, z=z)

def transform_vector3_by_matrix(vector3, mat):
    x, y, z, _ = (mat @ np.array([vector3.x, vector3.y, vector3.z, 0])).tolist()
    return Vector3(x=x, y=y, z=z)

def get_distance_between_origins(frame1, frame2):
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)
    transform = tf_buffer.lookup_transform(frame1, frame2, rospy.Time(0), rospy.Duration(60.0))
    t = transform.transform.translation
    return math.hypot(t.x, t.y, t.z)

def get_distance_to_car_front():
    return get_distance_between_origins("base_link", "car_front")

def get_origin_point(tf_buffer, frame_id, origin_frame_id, stamp=rospy.Time(0)):
    transform = tf_buffer.lookup_transform(frame_id, origin_frame_id, stamp, rospy.Duration(0.06))
    t = transform.transform.translation
    return Point(x=t.x, y=t.y, z=t.z)