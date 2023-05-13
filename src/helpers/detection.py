import math
import cv2
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PolygonStamped, Point

def create_hull(obj_pose, obj_dims, output_frame, stamp):

    """
    Produce convex hull for an object given its pose and dimensions
    :param obj_pose: geometry_msgs/Pose. Position and orientation of object
    :param obj_dims: Vector3 - length, width and height of object
    :param output_frame: string frame_id for the convex hull
    :param stamp: Time stamp at which the lidar pointcloud was created
    :return: geometry_msgs/PolygonStamped
    """
    convex_hull = PolygonStamped()
    convex_hull.header.frame_id = output_frame
    convex_hull.header.stamp = stamp

    # compute heading angle from object's orientation
    _, _, heading = euler_from_quaternion(
        (obj_pose.orientation.x, obj_pose.orientation.y, obj_pose.orientation.z, obj_pose.orientation.w))

    # use cv2.boxPoints to get a rotated rectangle given the angle
    points = cv2.boxPoints((
        (obj_pose.position.x, obj_pose.position.y),
        (obj_dims.x, obj_dims.y),
        math.degrees(heading)
    ))
    convex_hull.polygon.points = [Point(x, y, obj_pose.position.z) for x, y in points]

    return convex_hull
