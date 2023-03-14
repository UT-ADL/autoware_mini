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


def get_pose_using_heading_and_distance(start_pose, heading, distance):
    """
    Get pose from given pose and extrapolating it using heading and distance
    :param start_pose: Pose
    :param heading: heading in radians
    :param distance: wheel base
    :return: Pose - z and orientation is the same as start_pose
    """

    pose = Pose()
    pose.position.x = start_pose.position.x + distance * math.cos(heading)
    pose.position.y = start_pose.position.y + distance * math.sin(heading)
    pose.position.z = start_pose.position.z
    pose.orientation = start_pose.orientation

    return pose

def get_relative_heading_error(path_heading, current_heading):
    """
    Get heading error relative to path using current heading and track heading
    Input angles are within range [-pi, pi] and it needs to be considered when calculating the error
    and to retain the information of the direction of the error.
    :param heading_error: heading error in radians
    :return: steering difference in radians
    """

    err = (path_heading - current_heading)

    if err > math.pi:
        err -= 2 * math.pi
    elif err < -math.pi:
        err += 2 * math.pi

    return err


def get_intersection_point(ego_pose, pose1, pose2):
    """
    Calculates intersection point of two lines. One line is given by two points,
    the other line is given by a point and is perpendicular to the first line.
    :param pose1: Pose
    :param pose2: Pose
    :param pose3: Pose
    :return: pose
    """
    # ego_pose (front wheel)
    x_ego = ego_pose.position.x
    y_ego = ego_pose.position.y
    z = ego_pose.position.z
    # extract x and y from poses
    x1 = pose1.position.x
    y1 = pose1.position.y
    x2 = pose2.position.x
    y2 = pose2.position.y
 

    # calculate slope for the first line

    # no slope - horizontal line
    if (y2 - y1) == 0:
        x = x_ego
        y = y2
    # infinite slope - vertical line
    elif (x2 - x1) == 0:
        x = x2
        y = y_ego
    else:
        # calculate slopes
        m = (y2 - y1) / (x2 - x1)
        m_perp = -1 / m
        # calculate location on line - intersection point
        x = (m * x1 - m_perp * x_ego + y_ego - y1) / (m - m_perp)
        y = m * (x - x1) + y1

    # return x and y in Pose
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    
    return pose


def get_distance_between_two_poses(pose1, pose2):
    """
    Get distance between two poses
    :param pose1: Pose
    :param pose2: Pose
    :return: distance
    """
    x1 = pose1.position.x
    y1 = pose1.position.y
    x2 = pose2.position.x
    y2 = pose2.position.y

    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def get_point_on_path_within_distance(waypoints, last_idx, idx, pose, distance):
    """
    Get point on path within distance from ego pose
    :param waypoints: waypoints
    :param last_idx: last wp index
    :param idx: wp index from where to start calculate the distance
    :param pose: starting pose for distance calculation
    :param d: distance where to find the point on the path
    :return pose, v:
    """
    
    end_pose = Pose()
    i = idx
    d = get_distance_between_two_poses(pose, waypoints[i].pose.pose)
    while d < distance:
        i += 1
        d = get_distance_between_two_poses(pose, waypoints[i].pose.pose)
        if i == last_idx:
            break
    
    # Find end_pose orientation and distance difference and correct end_pose along path backwards
    end_orientation =  get_heading_between_two_poses(waypoints[i].pose.pose, waypoints[i - 1].pose.pose)
    dx = (distance - d) * math.cos(end_orientation)
    dy = (distance - d) * math.sin(end_orientation)
    end_pose.position.x = waypoints[i].pose.pose.position.x - dx
    end_pose.position.y = waypoints[i].pose.pose.position.y - dy
    end_pose.position.z = waypoints[i].pose.pose.position.z

    return end_pose
