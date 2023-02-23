import tf
import math

from autoware_msgs.msg import WaypointState
from geometry_msgs.msg import Pose


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


def get_cross_track_error(ego_pose, pose1, pose2):
    """
    Get cross track error from ego pose and two poses
    # calc distance from track
    # https://robotics.stackexchange.com/questions/22989/what-is-wrong-with-my-stanley-controller-for-car-steering-control

    :param ego_pose: Pose
    :param pose1: Pose
    :param pose2: Pose
    :return: cross track error
    """
    x_ego = ego_pose.position.x
    y_ego = ego_pose.position.y
    x1 = pose1.position.x
    y1 = pose1.position.y
    x2 = pose2.position.x
    y2 = pose2.position.y
    
    numerator = (x2 - x1) * (y1 - y_ego) - (x1 - x_ego) * (y2 - y1)
    denominator = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    return numerator / denominator


def get_front_wheel_pose(pose, heading, wheel_base):
    """
    Get front wheel pose from pose and heading
    :param pose: Pose
    :param heading: heading in radians
    :param wheel_base: wheel base
    :return: Pose
    """

    front_wheel_pose = Pose()
    front_wheel_pose.position.x = pose.position.x + wheel_base * math.cos(heading)
    front_wheel_pose.position.y = pose.position.y + wheel_base * math.sin(heading)
    front_wheel_pose.position.z = pose.position.z
    front_wheel_pose.orientation = pose.orientation

    return front_wheel_pose

def get_heading_angle_difference(heading_error):
    """
    Get steering difference from heading error
    TODO... heading_error can be in range of 
    :param heading_error: heading error in radians
    :return: steering difference in radians
    """
    error_degrees = math.degrees(abs(heading_error))

    if error_degrees < 180:
        return error_degrees
    else:
        return 360 - error_degrees