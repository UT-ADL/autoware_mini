import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Point, Quaternion


def get_heading_from_orientation(orientation):
    """
    Get heading angle from pose.orientation.
    :param pose: PoseStamped
    :return: heading in radians
    """

    quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
    _, _, heading = euler_from_quaternion(quaternion)

    return heading

def get_orientation_from_heading(heading):
    """
    Get orientation from heading (-pi...pi)
    :param heading: heading in radians
    :return: orientation
    """

    x, y, z, w = quaternion_from_euler(0, 0, heading)
    return Quaternion(x, y, z, w)

def get_heading_between_two_points(back_p, forward_p):
    """
    Get heading between two points
    :param back_p: Point
    :param forward_p: Point
    :return: heading in radians
    """

    return math.atan2(forward_p.y - back_p.y, forward_p.x - back_p.x)

def get_cross_track_error(ego_pos, pos1, pos2):
    """
    Get cross track error from ego pose and two poses
    # calc distance from track
    # https://robotics.stackexchange.com/questions/22989/what-is-wrong-with-my-stanley-controller-for-car-steering-control

    :param ego_pose: Pose
    :param pose1: Pose
    :param pose2: Pose
    :return: cross track error
    """

    numerator = (pos2.x - pos1.x) * (pos1.y - ego_pos.y) - (pos1.x - ego_pos.x) * (pos2.y - pos1.y)
    denominator = math.sqrt((pos2.x - pos1.x) ** 2 + (pos2.y - pos1.y) ** 2)

    return numerator / denominator

def get_point_using_heading_and_distance(start_point, heading, distance):
    """
    Get pose from given pose and extrapolating it using heading and distance
    :param start_pose: Pose
    :param heading: heading in radians
    :param distance: distance in meters
    :return: Pose - z and orientation is the same as start_pose
    """

    point = Point()
    point.x = start_point.x + distance * math.cos(heading)
    point.y = start_point.y + distance * math.sin(heading)
    point.z = start_point.z

    return point

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

def clamp(value, minimum, maximum):
    """
    Clamp value between minimum and maximum
    :param value: value to be clamped
    :param minimum: minimum value
    :param maximum: maximum value
    :return: clamped value
    """

    return max(minimum, min(value, maximum))

def get_closest_point_on_line(ego_point, point1, point2, clamp_output=True):
    """
    Calculates closest point on path. Constructs one line that is given by two points and
    the other line is given by a point and is known to be perpendicular to the first line.
    Closest point is the intersection of these lines.
    :param ego_point: Point
    :param point1: Point
    :param point2: Point
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

    # very small slope - almost horizontal line
    if abs(y2 - y1) < 0.0001:
        x = x_ego
        y = y2
    # infinite slope - almost vertical line
    elif abs(x2 - x1) < 0.0001:
        x = x2
        y = y_ego
    else:
        # calculate slopes
        m = (y2 - y1) / (x2 - x1)
        m_perp = -1 / m
        # calculate location on line - intersection point
        x = (m * x1 - m_perp * x_ego + y_ego - y1) / (m - m_perp)
        y = m * (x - x1) + y1

    # clip output to be within the line segment
    if clamp_output:
        x = clamp(x, min(x1, x2), max(x1, x2))
        y = clamp(y, min(y1, y2), max(y1, y2))

    return Point(x=x, y=y, z=z)

def get_distance_between_two_points_2d(p1, p2):
    """
    Get distance between two points
    :param point1: Pose
    :param point2: Pose
    :return: distance
    """

    return math.sqrt((p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2)

def get_angle_three_points_2d(point1, point2, point3):
    """
    Get angle between three points in 2D, point 2 is the center point.
    :param point1: Point
    :param point2: Point
    :param point3: Point
    :return: angle
    """

    v1x = point1.x - point2.x
    v1y = point1.y - point2.y
    v2x = point3.x - point2.x
    v2y = point3.y - point2.y
    dot = v1x * v2x + v1y * v2y
    cross = v1x * v2y - v1y * v2x
    angle = math.atan2(cross, dot)

    return angle

def get_vector_norm_3d(vec):
    """
    Get norm of 3d vector
    :param vec: vector
    :return: norm
    """

    return math.sqrt(vec.x ** 2 + vec.y ** 2 + vec.z ** 2)
