import rospy
import shapely
from geometry_msgs.msg import PointStamped, Vector3Stamped, PoseStamped
from tf2_ros import TransformListener, Buffer
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

def get_distance_to_car_front():
    # get the distance between the current_pose (base_link) and the front of the car
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer)
    transform = tf_buffer.lookup_transform("base_link", "car_front", rospy.Time(0), rospy.Duration(30.0))
    return transform.transform.translation.x

def get_car_front_point(tf_buffer, frame_id, stamp=rospy.Time(0)):
    transform = tf_buffer.lookup_transform(frame_id, "car_front", stamp, rospy.Duration(0.06))
    return shapely.Point(transform.transform.translation.x, transform.transform.translation.y)