import math
from tf.transformations import euler_from_quaternion
from autoware_msgs.msg import WaypointState
from geometry_msgs.msg import Pose, Point


def get_heading_from_pose_orientation(pose):
    """
    Get heading from pose
    :param pose: PoseStamped
    :return: heading in radians
    """

    quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    _, _, heading = euler_from_quaternion(quaternion)

    return heading


def get_heading_between_two_points(back_p, forward_p):
    """
    Get heading between two points
    :param back_p: Point
    :param forward_p: Point
    :return: heading in radians
    """

    return math.atan2(forward_p.y - back_p.y, forward_p.x - back_p.x)


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
    :param distance: distance in meters
    :return: Pose - z and orientation is the same as start_pose
    """

    pose = Pose()
    pose.position.x = start_pose.position.x + distance * math.cos(heading)
    pose.position.y = start_pose.position.y + distance * math.sin(heading)
    pose.position.z = start_pose.position.z
    pose.orientation = start_pose.orientation

    return pose

def normalize_heading_error(err):
    """
    Get heading error relative to path
    Previously subtracted track and current heading need to be normilized, since the original
    heading angles are within range [-pi, pi]
    :param err: heading error
    :return err: steering difference in radians
    """

    if err > math.pi:
        err -= 2 * math.pi
    elif err < -math.pi:
        err += 2 * math.pi

    return err


def get_closest_point(ego_point, point1, point2):
    """
    Calculates closest point on path. Constructs one line that is given by two points and
    the other line is given by a point and is known to be perpendicular to the first line.
    Closest point is the intersection of these lines.
    :param ego_pose: Pose
    :param pose2: Pose backward
    :param pose3: Pose forward
    :return: Point
    """
    # ego_pose (front wheel)
    x_ego = ego_point.x
    y_ego = ego_point.y
    z = ego_point.z
    # extract x and y from poses
    x1 = point1.x
    y1 = point1.y
    x2 = point2.x
    y2 = point2.y
 

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
    point = Point(x=x, y=y, z=z)
    
    return point


def get_distance_between_two_points(point1, point2):
    """
    Get distance between two points
    :param point1: Pose
    :param point2: Pose
    :return: distance
    """
    x1 = point1.x
    y1 = point1.y
    x2 = point2.x
    y2 = point2.y

    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def get_point_on_path_within_distance(waypoints, front_wp_idx, start_point, distance):
    """
    Get point on path within distance from ego pose
    :param waypoints: waypoints
    :param last_idx: last wp index
    :param front_wp_idx: wp index from where to start calculate the distance
    :param start_point: starting point for distance calculation
    :param distance: distance where to find the point on the path
    :return: Point
    """

    point = Point()
    last_idx = len(waypoints) - 1

    i = front_wp_idx
    d = get_distance_between_two_points(start_point, waypoints[i].pose.pose.position)
    while d < distance:
        i += 1
        d += get_distance_between_two_points(waypoints[i-1].pose.pose.position, waypoints[i].pose.pose.position)
        if i == last_idx:
            break

    # Find point orientation and distance difference and correct along path backwards
    end_orientation =  get_heading_between_two_points(waypoints[i].pose.pose.position, waypoints[i - 1].pose.pose.position)
    dx = (distance - d) * math.cos(end_orientation)
    dy = (distance - d) * math.sin(end_orientation)
    point.x = waypoints[i].pose.pose.position.x - dx
    point.y = waypoints[i].pose.pose.position.y - dy
    point.z = waypoints[i].pose.pose.position.z
    return point
