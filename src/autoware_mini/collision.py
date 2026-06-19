import math
import shapely
import numpy as np

from autoware_mini.geometry import get_angle_between_two_headings
from autoware_mini.shapely import calculate_linestring_heading_at_distance


DTYPE = np.dtype([
    ('x', np.float32),
    ('y', np.float32),
    ('z', np.float32),
    ('vx', np.float32),
    ('vy', np.float32),
    ('vz', np.float32),
    ('distance_to_stop', np.float32),
    ('deceleration_limit', np.float32),
    ('deceleration', np.float32),
    ('category', np.int32)
])

class CollisionPoints:

    EMPTY = np.array([], dtype=DTYPE)

    NO_PATH = -1
    NO_OBSTACLES = 0
    GOAL_POINT = 1
    TRAFFIC_LIGHT_STOP_LINE = 2
    OBJECT_ON_PATH = 3
    MOVING_OBJECT_ON_PATH = 4
    COLLIDING_TRAJECTORY = 5
    GIVE_WAY = 6
    OBJECT_ON_CROSSWALK = 7
    TRAJECTORY_ON_CROSSWALK = 8
    YIELDING_TRAJECTORY = 9
    STOP_LINE_FORCED_STOP = 10
    STOP_SIGN_STOP = 11
    GIVE_WAY_BUS = 12
    LANE_BOUNDARY = 13
    RIGHT_OF_WAY = 14
    APPROACHING_OBJECT_ON_PATH = 15

    COLLISION_POINT_CATEGORY_CAPTION = {
        NO_PATH:                            "Waiting for path",
        NO_OBSTACLES:                       "Following path",
        GOAL_POINT:                         "Arriving destination",
        TRAFFIC_LIGHT_STOP_LINE:            "Stopping for traffic light",
        OBJECT_ON_PATH:                     "Stopping for object",
        MOVING_OBJECT_ON_PATH:              "Following object",
        COLLIDING_TRAJECTORY:               "Stopping for traj. collision",
        GIVE_WAY:                           "Giving way to object",
        OBJECT_ON_CROSSWALK:                "Stopping for obj. on crosswalk",
        TRAJECTORY_ON_CROSSWALK:            "Stopping for crosswalk traj.",
        YIELDING_TRAJECTORY:                "Yielding to object",
        STOP_LINE_FORCED_STOP:              "Stopping for stop line",
        STOP_SIGN_STOP:                     "Stopping for stop sign",
        GIVE_WAY_BUS:                       "Giving way to a bus",
        LANE_BOUNDARY:                      "Stopping for lane boundary",
        RIGHT_OF_WAY:                       "Yielding to right-of-way",
        APPROACHING_OBJECT_ON_PATH:         "Stopping for approaching object"
    }

    COLLISION_POINT_CATEGORY_COLOR = {
        NO_PATH:                            "Gray",
        NO_OBSTACLES:                       "PaleGreen",
        GOAL_POINT:                         "PaleGreen",
        TRAFFIC_LIGHT_STOP_LINE:            "LightCoral",
        OBJECT_ON_PATH:                     "LightCoral",
        MOVING_OBJECT_ON_PATH:              "Khaki",
        COLLIDING_TRAJECTORY:               "LightCoral",
        GIVE_WAY:                           "Khaki",
        OBJECT_ON_CROSSWALK:                "LightCoral",
        TRAJECTORY_ON_CROSSWALK:            "Khaki",
        YIELDING_TRAJECTORY:                "Khaki",
        STOP_LINE_FORCED_STOP:              "LightCoral",
        STOP_SIGN_STOP:                     "LightCoral",
        GIVE_WAY_BUS:                       "Khaki",
        LANE_BOUNDARY:                      "LightCoral",
        RIGHT_OF_WAY:                       "Khaki",
        APPROACHING_OBJECT_ON_PATH:         "LightCoral"
    }

    COLLISION_POINT_CATEGORY_IGNORE_CAPTION = {
        TRAFFIC_LIGHT_STOP_LINE:            "Ignoring traffic light",
        GIVE_WAY:                           "Ignoring give way",
        TRAJECTORY_ON_CROSSWALK:            "Ignoring crosswalk trajectory",
        YIELDING_TRAJECTORY:                "Ignoring yielding",
        GIVE_WAY_BUS:                       "Ignoring give way to a bus",
        RIGHT_OF_WAY:                       "Ignoring right of way"
    }

    def __init__(self):
        self.points = []

    def add_point(self, x, y, z, distance_to_stop, category, deceleration, vx=0.0, vy=0.0, vz=0.0, deceleration_limit=np.inf):
        self.points.append(np.array([(x, y, z, vx, vy, vz, distance_to_stop, deceleration_limit, deceleration, category)], dtype=DTYPE))

    def add_points(self, points, z, distance_to_stop, category, deceleration, vx=0.0, vy=0.0, vz=0.0, deceleration_limit=np.inf):
        points = np.asarray(points)
        new_points = np.empty(len(points), dtype=DTYPE)
        new_points['x'] = points[:, 0]
        new_points['y'] = points[:, 1]
        new_points['z'] = z
        new_points['vx'] = vx
        new_points['vy'] = vy
        new_points['vz'] = vz
        new_points['distance_to_stop'] = distance_to_stop
        new_points['deceleration_limit'] = deceleration_limit
        new_points['deceleration'] = deceleration
        new_points['category'] = category
        self.points.append(new_points)

    def add_stop_line(self, wp, distance_to_stop, category, **kwargs):
        self.add_points(
            [[wp.left_boundary_point.x, wp.left_boundary_point.y],
             [wp.position.x, wp.position.y],
             [wp.right_boundary_point.x, wp.right_boundary_point.y]],
            z=wp.position.z,
            distance_to_stop=distance_to_stop,
            category=category,
            **kwargs)


def calculate_time_to_destination(speed, acceleration, distances):
    """
    Calculation of the time to reach certain distances given a constant speed and acceleration.
    :param speed: float, the speed of the object
    :param acceleration: float, the acceleration of the object
    :param distances: numpy array of floats, the distances to calculate the time to reach
    :return: numpy array of floats, the time it takes to reach the distances
    """

    # Initialize result with inf (invalid cases)
    time_to_destination = np.full_like(distances, np.inf, dtype=float)

    # Handle zero acceleration and zero speed cases outside of the main calculation
    if speed == 0 and acceleration == 0:
        # Object is not moving and has no acceleration
        pass
    elif acceleration == 0:
        # Object is moving with constant speed
        valid_distances = distances >= 0
        time_to_destination[valid_distances] = distances[valid_distances] / speed
    else:
        # Vectorized calculation for cases with acceleration
        discriminant = speed**2 + 2 * acceleration * distances  # Calculate the discriminant
        valid_discriminants = discriminant >= 0  # Mask invalid discriminants (negative values)

        # Apply formula only where discriminant is valid
        time_to_destination[valid_discriminants] = (
            -speed / acceleration +
            np.sqrt(discriminant[valid_discriminants]) / np.abs(acceleration)
        )

    return time_to_destination


def create_trajectory_buffers(detected_objects):
    """Create trajectory linestrings and buffers for all detected objects.

    Returns:
        obj_indices: np.array of int indices into detected_objects
        trajectory_linestrings: np.array of shapely LineStrings
        trajectory_buffers: np.array of shapely Polygons
    """
    obj_indices = []
    linestrings = []
    widths = []
    for obj_idx, obj in enumerate(detected_objects):
        for path in obj.candidate_trajectories.paths:
            coords = np.array([(wp.position.x, wp.position.y, wp.position.z) for wp in path.waypoints])
            obj_indices.append(obj_idx)
            linestrings.append(shapely.linestrings(coords))
            widths.append(obj.dimensions.y / 2)

    if not linestrings:
        return np.array([], dtype=int), np.array([]), np.array([])

    trajectory_linestrings = np.array(linestrings)
    trajectory_buffers = shapely.buffer(trajectory_linestrings, np.array(widths), cap_style="flat")
    return np.array(obj_indices), trajectory_linestrings, trajectory_buffers


def is_coming_from_behind(behind_linestring, behind_path_width, ahead_linestring, ahead_path_width, heading_limit):
    """
    Check if behind_linestring is coming from behind the ahead_linestring.
    :param behind_linestring: shapely LineString of the trajectory behind
    :param behind_path_width: width of the behind path
    :param ahead_linestring: shapely LineString of the trajectory ahead
    :param ahead_path_width: width of the ahead path
    :param heading_limit: maximum heading difference (degrees) for paths to be considered same direction
    :return: True if behind_linestring is coming from behind ahead_linestring, False otherwise
    """

    # check if start point of ahead_linestring is within buffer distance of behind_linestring
    buffer_width = (behind_path_width + ahead_path_width) / 2.0
    start_point = shapely.Point(ahead_linestring.coords[0])
    if start_point.dwithin(behind_linestring, buffer_width):

        # find heading of behind_linestring at closest point to start of ahead_linestring
        projected_distance = behind_linestring.project(start_point)
        behind_heading = calculate_linestring_heading_at_distance(behind_linestring, projected_distance)
        ahead_heading = calculate_linestring_heading_at_distance(ahead_linestring, 0.0)

        # check if headings are within limit
        if math.degrees(get_angle_between_two_headings(ahead_heading, behind_heading)) <= heading_limit:
            return True

    return False

