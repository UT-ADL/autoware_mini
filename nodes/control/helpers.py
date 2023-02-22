#!/usr/bin/env python

import tf
import math

from autoware_msgs.msg import WaypointState


def get_heading_from_pose_orientation(pose):
    """
    Get heading from pose
    :param pose: PoseStamped
    :return: heading in radians
    """

    quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    _, _, heading = tf.transformations.euler_from_quaternion(quaternion)

    return heading


def get_heading_between_two_poses(pose1, pose2):
    """
    Get heading between two poses
    Heading is calculated from pose1 to pose2
    :param pose1: Pose
    :param pose2: Pose
    :return: heading in radians
    """

    return math.atan2(pose2.position.y - pose1.position.y, pose2.position.x - pose1.position.x)


def get_blinker_state(steering_state):
    """
    Get blinker state  from WaypointState/steering_state
    :param steering_state: steering state
    :return: LampCmd (l, r) included in VehicleCmd
    """

    if steering_state == WaypointState.STR_LEFT:
        return 1, 0
    elif steering_state == WaypointState.STR_RIGHT:
        return 0, 1
    elif steering_state == WaypointState.STR_STRAIGHT:
        return 0, 0
    else:
        return 0, 0