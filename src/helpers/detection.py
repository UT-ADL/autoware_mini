import math
import cv2

from geometry_msgs.msg import PolygonStamped, Point

from helpers.geometry import get_heading_from_orientation

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
    heading = get_heading_from_orientation(obj_pose.orientation)

    # use cv2.boxPoints to get a rotated rectangle given the angle
    points = cv2.boxPoints((
        (obj_pose.position.x, obj_pose.position.y),
        (obj_dims.x, obj_dims.y),
        math.degrees(heading)
    ))
    convex_hull.polygon.points = [Point(x, y, obj_pose.position.z) for x, y in points]

    return convex_hull
