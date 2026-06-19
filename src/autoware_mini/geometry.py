import math
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Point, Point32, Quaternion

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

    x, y, z, w = quaternion_from_euler(0, 0, heading).tolist()
    return Quaternion(x=x, y=y, z=z, w=w)

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

def get_point32_using_heading_and_distance(start_point, heading, distance):
    """
    Get Point32 from given point and extrapolating it using heading and distance
    :param start_point: Point32
    :param heading: heading in radians
    :param distance: distance in meters
    :return: Point32
    """

    x = start_point.x + distance * math.cos(heading)
    y = start_point.y + distance * math.sin(heading)

    return Point32(x=x, y=y, z=start_point.z)

def normalize_angle(angle):
    """
    Normalize angle to be within range [-pi, pi]
    :param angle: heading angle in radians
    :return: normalized angle
    """

    return (angle + math.pi) % (2*math.pi) - math.pi

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

    bax = first_point.x - middle_point.x
    bay = first_point.y - middle_point.y
    bcx = third_point.x - middle_point.x
    bcy = third_point.y - middle_point.y

    dot_product = bax * bcx + bay * bcy
    magnitude_ba = math.sqrt(bax * bax + bay * bay)
    magnitude_bc = math.sqrt(bcx * bcx + bcy * bcy)

    cos_theta = max(-1.0, min(1.0, dot_product / (magnitude_ba * magnitude_bc)))

    return math.acos(cos_theta)

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
    Get the smallest absolute angle difference between two heading angles (in radians).
    Works with scalars or NumPy arrays.

    :param angle1: angle(s) 1, from -pi to pi
    :param angle2: angle(s) 2, from -pi to pi
    :return: absolute relative angle difference(s), from 0 to pi
    """

    diff = np.abs(angle1 - angle2)
    diff = np.where(diff > np.pi, 2 * np.pi - diff, diff)
    return diff

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

def calculate_headings(path_coords, distances=None):
    """
    Calculate headings for a linestring, either at each segment
    or at specific distances along the path.

    :param path_coords: path coordinates as a sequence of tuples or numpy array of shape (N, 2)
    :param distances: None (returns all headings) or array-like of distances
    :return:
        - If distances is None: returns array of headings for each vertex
        - If distances is provided: returns headings at those distances
    """

    path_arr = np.asarray(path_coords)

    # Calculate headings for each segment
    diffs = np.diff(path_arr[:, :2], axis=0)
    headings = np.arctan2(diffs[:, 1], diffs[:, 0])

    if distances is None:
        # Return headings for each vertex
        return np.append(headings, headings[-1]) # last heading same as second last

    # Calculate headings at the given distances.
    seg_lengths = np.linalg.norm(diffs, axis=1)
    cum_lengths = np.insert(np.cumsum(seg_lengths), 0, 0.0)

    seg_idx = np.searchsorted(cum_lengths, distances, side='right') - 1
    seg_idx = np.clip(seg_idx, 0, len(headings) - 1)

    return headings[seg_idx]

def calculate_radius(x, y, n, eps=1e-10):
    """
    Compute the circumradius of triangles formed by points separated by `n`
    steps along a 2D polyline.

    For each index `i`, a triangle is formed by the points
    `(x[i-n], y[i-n])`, `(x[i], y[i])`, and `(x[i+n], y[i+n])`.
    The radius of the circumscribed circle of this triangle is computed.
    The resulting radius array has the same length as the input, with
    edge values replicated.

    The circumradius is computed using the relation:

        R = (a * b * c) / (4 * A)

    where `a`, `b`, and `c` are the triangle side lengths and `A` is the
    triangle area. The area is evaluated via the 2D cross product for
    numerical stability.

    Parameters
    ----------
    x, y : array_like
        Coordinates of the polyline points. Must have the same length.
    n : int
        Step size used to select points before and after each center point.
        If the path is too short, `n` is reduced to `(len(x) - 1) // 2`.
    eps : float, optional
        Small positive value used to clamp the triangle area in order to
        avoid division by zero for collinear points.

    Returns
    -------
    radius : ndarray
        Array of circumcircle radii with the same length as `x` and `y`.
        The first and last `n` values are replicated from the nearest
        computed radius.

    Notes
    -----
    - Collinear or nearly collinear points yield very large radii.
    - If `n == 0` or fewer than three points are available, the function
      returns an array of `np.inf`.
    - This function is commonly used as a curvature-related measure for
      polylines.
    """
    x = np.asarray(x)
    y = np.asarray(y)
    m = x.size

    # adjust n for very short paths
    n = min(n, (m - 1) // 2)

    # not enough points (or n becomes 0) -> radius undefined
    if n == 0 or m < 3:
        return np.full(m, np.inf, dtype=float)

    ax, ay = x[:-2*n], y[:-2*n]
    bx, by = x[n:-n],  y[n:-n]
    cx, cy = x[2*n:],  y[2*n:]

    # side lengths
    a = np.hypot(bx - ax, by - ay)
    b = np.hypot(cx - ax, cy - ay)
    c = np.hypot(cx - bx, cy - by)

    # twice triangle area via cross product: |(B-A) x (C-A)|
    area2 = np.abs((bx - ax) * (cy - ay) - (by - ay) * (cx - ax))
    area2 = np.maximum(area2, eps)  # avoid divide-by-zero / huge spikes

    # circumradius: R = abc / (4A) = abc / (2 * area2)
    radius_mid = (a * b * c) / (2 * area2)

    # extend back to original length
    return np.pad(radius_mid, (n, n), mode="edge")