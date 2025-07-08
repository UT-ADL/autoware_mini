import math
import cv2
import numpy as np
from autoware_mini.geometry import get_heading_from_vector

def create_hull(obj):

    """
    Produce convex hull for an object given its pose and dimensions
    :param obj: autoware_mini/DetectedObject
    :param output_frame: string frame_id for the convex hull
    :param stamp: Time stamp at which the lidar pointcloud was created
    :return: geometry_msgs/PolygonStamped
    """
    # use cv2.boxPoints to get a rotated rectangle given the angle
    points = cv2.boxPoints((
        (obj.center.x, obj.center.y),
        (obj.dimensions.x, obj.dimensions.y),
        math.degrees(obj.heading)
    ))

    z = obj.center.z - obj.dimensions.z / 2
    convex_hull = np.concatenate((points, np.full((points.shape[0], 1), z)), axis=1).ravel().tolist()

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

def get_axis_oriented_bounding_box(obj):
    """
    Get the axis-oriented bounding box of an object
    :param obj: autoware_mini/DetectedObject
    :return: tuple of minx, miny, maxx, maxy
    """
    # take all points from the convex hull
    points = np.array(obj.convex_hull).reshape(-1, 3)[:, :2]

    # find axis-oriented bounding box
    minx, miny = np.min(points, axis=0)
    maxx, maxy = np.max(points, axis=0)

    return minx, miny, maxx, maxy

def update_object_position_dimensions(obj):
    """
    Update width, length and position of the object, based on object's velocity vector aligned bounding box
    :param obj: DetectedObject
    """

    # Collect points from convex_hull and extract rotation center
    points = np.array(obj.convex_hull).reshape(-1, 3)[:, :2]
    center = np.array([obj.center.x, obj.center.y])

    heading_angle = get_heading_from_vector(obj.velocity)

    # Create rotation matrix
    cos_angle = math.cos(-heading_angle)
    sin_angle = math.sin(-heading_angle)
    rotation_matrix = np.array([
        [cos_angle, -sin_angle],
        [sin_angle, cos_angle]
    ])

    # Translate and rotate points
    points -= center
    points = points @ rotation_matrix.T

    # Calculate bounds in the rotated coordinate system
    minx, miny = points.min(axis=0)
    maxx, maxy = points.max(axis=0)
    width = (maxy - miny)
    length = (maxx - minx)
    center_x = (minx + maxx) / 2
    center_y = (miny + maxy) / 2

    # bounding box center
    target_point = np.array([center_x, center_y])

    # Create inverse rotation matrix
    # sin(-a) = -sin(a), cos(-a) = cos(a)
    inverse_rotation_matrix = np.array([
        [cos_angle, sin_angle],
        [-sin_angle, cos_angle]
    ])

    # Apply inverse rotation to target points, then translation
    target_point = target_point @ inverse_rotation_matrix.T
    target_point += center

    obj.center.x = target_point[0]
    obj.center.y = target_point[1]
    obj.dimensions.x = length
    obj.dimensions.y = width
    obj.heading = heading_angle 


if __name__ == '__main__':
    boxes1 = np.array([[0, 0, 10, 10], [10, 10, 20, 20]])
    boxes2 = np.array([[5, 5, 15, 15], [15, 15, 25, 25]])

    iou = calculate_iou(boxes1, boxes2)
    assert np.allclose(iou, np.array([[25 / 175, 0.0], [25 / 175, 25 / 175]])), str(iou)