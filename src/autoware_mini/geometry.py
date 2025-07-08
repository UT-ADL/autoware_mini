import math
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Point, Quaternion

def get_heading_from_vector(vector):
    """
    Get heading from vector
    :param vector: vector
    :return: heading in radians
    """

    return math.atan2(vector.y, vector.x)

def get_heading_from_orientation(orientation):
    """
    Get heading angle from orientation.
    :param orientation: Quaternion
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
    :rtype: Quaternion
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

def get_point_using_heading_and_distance(start_point, heading, distance):
    """
    Get point from given point and extrapolating it using heading and distance
    :param start_point: Point
    :param heading: heading in radians
    :param distance: distance in meters
    :return: Point
    """

    x = start_point.x + distance * math.cos(heading)
    y = start_point.y + distance * math.sin(heading)

    return Point(x=x, y=y, z=start_point.z)

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

def get_distance_between_two_points_2d(p1, p2):
    """
    Get distance between two points
    :param point1: Point
    :param point2: Point
    :return: distance
    """

    return math.hypot(p2.x - p1.x, p2.y - p1.y)

def get_speed_from_velocity(vec):
    """
    Get norm of 2d vector
    :param vec: vector
    :return: norm
    """

    return math.hypot(vec.x, vec.y)

def project_vector_to_heading(heading_angle, vector):
    """
    Project vector to heading
    :param heading_angle: heading angle in radians
    :param vector: vector
    :return: projected vector
    """

    return vector.x * math.cos(heading_angle) + vector.y * math.sin(heading_angle)

def create_vector_from_heading_and_scalar(heading, scalar):
    """
    Create vector from heading and scalar
    :param heading: heading in radians
    :param scalar: scalar
    :return: vector
    """

    return (scalar * math.cos(heading), scalar * math.sin(heading))

def get_angle_between_three_points(first_point, middle_point, third_point):
    """
    Calculates angle between three points
    :param first_point: Point
    :param middle_point: Middle point
    :param third_point: Point
    :return: angle in radians
    """

    BA = np.array([first_point.x - middle_point.x, first_point.y - middle_point.y])
    BC = np.array([third_point.x - middle_point.x, third_point.y - middle_point.y])
    
    dot_product = np.dot(BA, BC)
    
    magnitude_ba = np.linalg.norm(BA)
    magnitude_bc = np.linalg.norm(BC)
    
    cos_theta = dot_product / (magnitude_ba * magnitude_bc)
    
    # Clip the cosine value within range [-1, 1] to avoid numerical issues
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    
    angle = np.arccos(cos_theta)
    
    return angle

def calculate_points_on_bezier_curve(start_point, control_point1, control_point2, end_point, n):
    """
    Generates n number of equaly spaced points on a Bezier curve
    :param start_point: Bezier curve starting point
    :param control_point1: Bezier curve control point 1
    :param control_point2: Bezier curve control point 2
    :param end_point: Bezier curve end point
    :param n: number of points to calculate
    :return: points on Bezier curve
    """

    p0 = np.array([start_point.x, start_point.y])
    p1 = np.array([control_point1.x, control_point1.y])
    p2 = np.array([control_point2.x, control_point2.y])
    p3 = np.array([end_point.x, end_point.y])

    # using implicit broadcasting to ensure that the operation is performed on each point
    t = np.linspace(0, 1, n)[:, np.newaxis]
    bezier_points = (1-t)**3 * p0 + 3*(1-t)**2 * t * p1 + 3*(1-t) * t**2 * p2 + t**3 * p3
    return bezier_points

def get_angle_between_two_headings(angle1, angle2):
    """
    Get angle between two heading angles
    :param angle1: angle 1 from -pi to pi
    :param angle2: angle 2 from -pi to pi
    :return: relative angle difference
    """

    difference = abs(angle1 - angle2)
    if difference > math.pi:
        difference = 2*math.pi - difference
    return difference

def convert_geometry_to_line_list(geometry, delta_z=0):
    """
    Convert geometry to line list
    :param geometry: geometry
    :param delta_z: z offset added to actual z values
    :return: line list with Points
    """

    # create list of Point
    if isinstance(geometry[0], tuple):  # Case: geometry is list of coordinate tuples
        points = [Point(x=x, y=y, z=z + delta_z) for x, y, z in geometry]
    else:  # Case: Lanelet2 linestring
        points = [Point(x=point.x, y=point.y, z=point.z + delta_z) for point in geometry]

    # Create line list
    line_list_points = []
    for i in range(len(points) - 1):
        line_list_points.append(points[i])
        line_list_points.append(points[i + 1])
    
    return line_list_points
