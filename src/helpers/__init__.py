import math
import numpy as np
import matplotlib.pyplot as plt
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from autoware_msgs.msg import WaypointState
from geometry_msgs.msg import Pose, Point, Quaternion


def get_heading_from_pose_orientation(pose):
    """
    Get heading angle from pose.orientation.
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

def get_orientation_from_yaw(heading):
    """
    Get orientation from heading (-pi...pi)
    :param heading: heading in radians
    :return: orientation
    """

    x, y, z, w = quaternion_from_euler(0, 0, heading)
    orientation = Quaternion(x, y, z, w)
    return orientation


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


def get_two_nearest_waypoint_idx(waypoint_tree, x, y):
    """
    Find 2 cloest waypoint index values from the waypoint_tree
    :param waypoint_tree:
    :param x
    :param y
    """

    idx = waypoint_tree.kneighbors([(x, y)], 2, return_distance=False)

    # sort to get them in ascending order - follow along path
    idx[0].sort()
    return idx[0][0], idx[0][1]


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


def get_closest_point_on_line(ego_point, point1, point2):
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
    x = np.clip(x, min(x1, x2), max(x1, x2))
    y = np.clip(y, min(y1, y2), max(y1, y2))

    return Point(x = x, y = y, z=z)


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

def interpolate_velocity_between_waypoints(point, backward_wp, forward_wp):
    """
    Interpolate velocity between two waypoints.
    :param point: Point - location where the velocity will be interpolated using backward and forward waypoints.
    :param bacward_wp: Waypoint
    :param forward_wp: Waypoint
    :return: velocity
    """

    # distance to backward waypoint
    distance_to_backward_wp = get_distance_between_two_points(point, backward_wp.pose.pose.position)
    # distance to forward waypoint
    distance_to_forward_wp = get_distance_between_two_points(point, forward_wp.pose.pose.position)

    backward_wp_vel = backward_wp.twist.twist.linear.x * distance_to_forward_wp / (distance_to_backward_wp + distance_to_forward_wp)
    forward_wp_vel = forward_wp.twist.twist.linear.x * distance_to_backward_wp / (distance_to_backward_wp + distance_to_forward_wp)

    return backward_wp_vel + forward_wp_vel


def create_closest_point_on_path(waypoints, closest_idx, origin_point):
    """
    Project origin_point onto the path. First decide weather to use backward or forward waypointand then call
    get_closest_point_on_line function to get the closest point on path.
    :param waypoints: list of waypoints
    :param closest_idx: index of the closest waypoint
    :param origin_point: Point - origin point
    :return: Point - closest point on path
    """

    #initialize angles
    backward_angle = 0
    forward_angle = 0

    if closest_idx > 0:
        backward_angle = abs(get_angle_three_points_2d(waypoints[closest_idx - 1].pose.pose.position, origin_point, waypoints[closest_idx].pose.pose.position))
    if closest_idx < len(waypoints) - 1:
        forward_angle = abs(get_angle_three_points_2d(waypoints[closest_idx].pose.pose.position, origin_point, waypoints[closest_idx + 1].pose.pose.position))

    # if backward angle is bigger then project point to the backward section
    if backward_angle > forward_angle:
        closest_idx -= 1

    origin_position = Point(x = origin_point.x, y = origin_point.y, z = waypoints[closest_idx].pose.pose.position.z)
    point = get_closest_point_on_line(origin_position, waypoints[closest_idx].pose.pose.position, waypoints[closest_idx + 1].pose.pose.position)
    return point


def get_angle_three_points_2d(point1, point2, point3):
    """
    Get angle between three points in 2D, point 2 is the center point.
    :param point1: Point
    :param point2: Point
    :param point3: Point
    :return: angle
    """

    v1 = np.array([point1.x - point2.x, point1.y - point2.y])
    v2 = np.array([point3.x - point2.x, point3.y - point2.y])
    dot = np.dot(v1, v2)
    cross = np.cross(v1, v2)
    angle = math.atan2(cross, dot)

    return angle

def get_point_on_path_within_distance(waypoints, front_wp_idx, start_point, distance):
    """
    Get point on path within distance from ego pose
    :param waypoints: waypoints
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


def debug_plots_path_smoothing(x_path, y_path, z_path, blinker, x_new, y_new, z_new, blinker_new, distances, new_distances, speed, speed_new):

    fig = plt.figure(figsize=(10, 15))
    ax = fig.subplots()
    ax.scatter(x_path,y_path, color = 'blue')
    ax.scatter(x_new, y_new, color = 'red', marker = 'x', alpha = 0.5, label = 'interpolated')
    plt.legend()
    plt.show()

    # new plot for heights 
    fig = plt.figure(figsize=(10, 15))
    ax = fig.subplots()
    ax.scatter(new_distances, z_new, color = 'red', marker = 'x', alpha = 0.5, label = 'height interpolated')
    ax.plot(new_distances, z_new, color = 'red', alpha = 0.5, label = 'height interpolated')
    ax.scatter(distances, z_path, color = 'blue', alpha = 0.5, label = 'height old')
    plt.legend()
    plt.show()

    # new plot for blinkers
    fig = plt.figure(figsize=(10, 15))
    ax = fig.subplots()
    ax.scatter(new_distances, blinker_new, color = 'red', marker = 'x', alpha = 0.5, label = 'blinker interpolated')
    ax.plot(new_distances, blinker_new, color = 'red', alpha = 0.5, label = 'blinker interpolated')
    ax.scatter(distances, blinker, color = 'blue', alpha = 0.5, label = 'blinker old')
    plt.legend()
    plt.show()

    # new plot for speed
    fig = plt.figure(figsize=(10, 15))
    ax = fig.subplots()
    ax.scatter(new_distances, speed_new * 3.6, color = 'red', marker = 'x', alpha = 0.5, label = 'speed interpolated')
    ax.plot(new_distances, speed_new * 3.6, color = 'red', alpha = 0.5, label = 'speed interpolated')
    ax.scatter(distances, speed * 3.6, color = 'blue', alpha = 0.5, label = 'speed old')
    plt.legend()
    plt.show()