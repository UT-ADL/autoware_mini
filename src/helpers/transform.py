from geometry_msgs.msg import PointStamped, Vector3Stamped, PoseStamped
from tf2_geometry_msgs import do_transform_point, do_transform_vector3, do_transform_pose

def transform_point(point, transform):
    # to apply a transform we need a point stamped
    point_stamped = PointStamped(point=point)
    return do_transform_point(point_stamped, transform).point

def transform_vector3(vector3, transform):
    # to apply a transform we need a vector3 stamped
    vector3_stamped = Vector3Stamped(vector=vector3)
    return do_transform_vector3(vector3_stamped, transform).vector

def transform_pose(pose, transform):
    # to apply a transform we need a pose stamped
    pose_stamped = PoseStamped(pose=pose)
    return do_transform_pose(pose_stamped, transform).pose
