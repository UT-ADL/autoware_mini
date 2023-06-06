import math
import cv2
import numpy as np

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

def calculate_iou(boxes1, boxes2):
    """
    Calculate the IOU between two sets of bounding boxes.

    Args:
        boxes1: a numpy array of shape (n, 4) containing the coordinates of n bounding boxes in the format (x1, y1, x2, y2).
        boxes2: a numpy array of shape (m, 4) containing the coordinates of m bounding boxes in the format (x1, y1, x2, y2).

    Returns:
        a numpy array of shape (n, m) containing the IOU between all pairs of bounding boxes.
    """
    # Calculate the area of each bounding box
    area1 = (boxes1[:, 2] - boxes1[:, 0]) * (boxes1[:, 3] - boxes1[:, 1])
    area2 = (boxes2[:, 2] - boxes2[:, 0]) * (boxes2[:, 3] - boxes2[:, 1])

    # Calculate the coordinates of the intersection bounding boxes
    intersection_x1 = np.maximum(boxes1[:, 0][:, np.newaxis], boxes2[:, 0])
    intersection_y1 = np.maximum(boxes1[:, 1][:, np.newaxis], boxes2[:, 1])
    intersection_x2 = np.minimum(boxes1[:, 2][:, np.newaxis], boxes2[:, 2])
    intersection_y2 = np.minimum(boxes1[:, 3][:, np.newaxis], boxes2[:, 3])

    # Calculate the area of the intersection bounding boxes
    intersection_area = np.maximum(intersection_x2 - intersection_x1, 0) * np.maximum(intersection_y2 - intersection_y1, 0)

    # Calculate the union of the bounding boxes
    union_area = area1[:, np.newaxis] + area2 - intersection_area

    # Calculate the IOU
    iou = intersection_area / union_area

    return iou


if __name__ == '__main__':
    boxes1 = np.array([[0, 0, 10, 10], [10, 10, 20, 20]])
    boxes2 = np.array([[5, 5, 15, 15], [15, 15, 25, 25]])

    iou = calculate_iou(boxes1, boxes2)
    assert np.allclose(iou, np.array([[25 / 175, 0.0], [25 / 175, 25 / 175]])), str(iou)