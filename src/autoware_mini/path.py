import shapely
import numpy as np
from copy import deepcopy
from functools import cached_property
import scipy.interpolate
from autoware_mini.msg import Waypoint
from geometry_msgs.msg import Point, Pose
from autoware_mini.geometry import get_heading_between_two_points, get_orientation_from_heading, calculate_headings
from autoware_mini.shapely import calculate_cross_track_error

class PathWrapper:
    def __init__(self, waypoints):

        if len(waypoints) == 1:
            raise ValueError("Path must have 0 or at least 2 waypoints")

        self.waypoints = waypoints

        if waypoints:
            self.centerline_array = np.array([(waypoint.position.x, waypoint.position.y, waypoint.position.z) for waypoint in waypoints])
        else:
            self.centerline_array = np.empty((0, 3))

        self.linestring = shapely.linestrings(self.centerline_array)
        shapely.prepare(self.linestring)

    @cached_property
    def distances(self):
        d = np.cumsum(np.sqrt(np.sum(np.diff(self.centerline_array[:, :2], axis=0)**2, axis=1)))
        return np.insert(d, 0, 0)

    @cached_property
    def left_boundary_array(self):
        return np.array([(waypoint.left_boundary_point.x, waypoint.left_boundary_point.y, waypoint.left_boundary_point.z) for waypoint in self.waypoints])

    @cached_property
    def right_boundary_array(self):
        return np.array([(waypoint.right_boundary_point.x, waypoint.right_boundary_point.y, waypoint.right_boundary_point.z) for waypoint in self.waypoints])

    @cached_property
    def left_boundary_distances(self):
        d = np.cumsum(np.sqrt(np.sum(np.diff(self.left_boundary_array[:, :2], axis=0)**2, axis=1)))
        return np.insert(d, 0, 0)

    @cached_property
    def right_boundary_distances(self):
        d = np.cumsum(np.sqrt(np.sum(np.diff(self.right_boundary_array[:, :2], axis=0)**2, axis=1)))
        return np.insert(d, 0, 0)

    @cached_property
    def speeds(self):
        return np.array([waypoint.speed for waypoint in self.waypoints])

    @cached_property
    def speed_limits(self):
        return np.array([waypoint.speed_limit for waypoint in self.waypoints])

    @cached_property
    def turn_signals(self):
        return np.array([waypoint.turn_signal for waypoint in self.waypoints])

    @cached_property
    def priorities(self):
        return np.array([waypoint.priority for waypoint in self.waypoints])

    @cached_property
    def boundary_types(self):
        return np.array([(waypoint.left_boundary_type, waypoint.right_boundary_type) for waypoint in self.waypoints])

    @cached_property
    def stop_line_ids(self):
        return np.array([waypoint.stop_line_id for waypoint in self.waypoints])

    @cached_property
    def stop_line_types(self):
        return np.array([waypoint.stop_line_type for waypoint in self.waypoints])

    @cached_property
    def left_boundary(self):
        left_boundary = shapely.linestrings(self.left_boundary_array)
        shapely.prepare(left_boundary)
        return left_boundary

    @cached_property
    def right_boundary(self):
        right_boundary = shapely.linestrings(self.right_boundary_array)
        shapely.prepare(right_boundary)
        return right_boundary

    @cached_property
    def _distance_to_heading_interpolator(self):
        h = calculate_headings(self.centerline_array)
        return scipy.interpolate.interp1d(self.distances, h , kind='previous', bounds_error=False, fill_value="extrapolate")

    @cached_property
    def _distance_to_speed_interpolator(self):
        return scipy.interpolate.interp1d(self.distances, self.speeds, kind='linear', bounds_error=False, fill_value=0.0)

    @cached_property
    def _distance_to_speed_limit_interpolator(self):
        return scipy.interpolate.interp1d(self.distances, self.speed_limits, kind='previous', bounds_error=False, fill_value=0.0)

    @cached_property
    def _distance_to_turn_signal_interpolator(self):
        return scipy.interpolate.interp1d(self.distances, self.turn_signals, kind='previous', bounds_error=False, fill_value=Waypoint.TURN_STRAIGHT)

    @cached_property
    def _distance_to_priority_interpolator(self):
        return scipy.interpolate.interp1d(self.distances, self.priorities.astype(float) , kind='previous', bounds_error=False, fill_value=0.0)

    @cached_property
    def _distance_to_lane_change_interpolator(self):
        lane_change_states = np.array([waypoint.lanechange_state for waypoint in self.waypoints])
        return scipy.interpolate.interp1d(self.distances, lane_change_states, kind='nearest', bounds_error=False, fill_value=0.0)

    @cached_property
    def _left_type_interpolator(self):
        return scipy.interpolate.interp1d(self.distances, self.boundary_types[:, 0] , kind='previous', bounds_error=False, fill_value=Waypoint.VIRTUAL)

    @cached_property
    def _right_type_interpolator(self):
        return scipy.interpolate.interp1d(self.distances, self.boundary_types[:, 1] , kind='previous', bounds_error=False, fill_value=Waypoint.VIRTUAL)

    @cached_property
    def _centerline_to_left_boundary_distance(self):
        return scipy.interpolate.interp1d(self.distances, self.left_boundary_distances, kind='linear', bounds_error=False, fill_value='extrapolate')

    @cached_property
    def _centerline_to_right_boundary_distance(self):
        return scipy.interpolate.interp1d(self.distances, self.right_boundary_distances, kind='linear', bounds_error=False, fill_value='extrapolate')

    def get_stop_lines(self, target_types, exclude_ids=None):
        # get stop lines of certain types, optionally excluding specific ids
        mask = np.isin(self.stop_line_types, target_types)
        if exclude_ids:
            mask &= ~np.isin(self.stop_line_ids, exclude_ids)
        # return distances, ids, types and indices of stop lines matching the criteria
        return self.distances[mask], self.stop_line_ids[mask], self.stop_line_types[mask], np.where(mask)[0]

    def get_waypoint_index_at_distance(self, distance, side="left"):
        """
        Get waypoint at a certain distance along the path
        :param distance: distance along the path (m)
        :param side: side to search for the distance
        :return: waypoint
        """
        return np.searchsorted(self.distances, distance, side)

    def _extract_waypoints(self, index_start, index_end, copy=False):
        """
        Extract waypoints from index_start to index_end
        :param index_start: start index
        :param index_end: end index
        :return: waypoints
        """
        waypoints = []

        if copy:
            # for each new waypoint copy only the necessary parts
            for waypoint in self.waypoints[index_start:index_end]:
                new_waypoint = Waypoint(position=waypoint.position,
                                        speed=waypoint.speed,
                                        left_boundary_point=waypoint.left_boundary_point,
                                        right_boundary_point=waypoint.right_boundary_point,
                                        left_boundary_type=waypoint.left_boundary_type,
                                        right_boundary_type=waypoint.right_boundary_type,
                                        lanechange_state=waypoint.lanechange_state,
                                        turn_signal=waypoint.turn_signal,
                                        priority=waypoint.priority,
                                        stop_line_id=waypoint.stop_line_id,
                                        stop_line_type=waypoint.stop_line_type)
                waypoints.append(new_waypoint)
        else:
            waypoints = self.waypoints[index_start:index_end]

        return waypoints

    def extract_waypoints(self, distance_start, distance_end, trim=False, copy=False):
        """
        Get waypoints between two distances along the path
        :param distance_start: start distance along the path (m)
        :param distance_end: end distance along the path (m)
        :param trim: trim the waypoints to exact distances
        :return: waypoints
        """

        index_start = self.get_waypoint_index_at_distance(distance_start, side="right")
        index_end = self.get_waypoint_index_at_distance(distance_end, side="left")

        # extend the path by one waypoint backwards
        index_start = max(0, index_start - 1)

        # there must be at least two waypoints for a valid path
        if index_end <= index_start + 1:
            return []

        waypoints = self._extract_waypoints(index_start, index_end, copy=copy)

        if trim:
            if not copy:
                # if not copy, we need to create new waypoints
                waypoints[0] = deepcopy(waypoints[0])
                waypoints[-1] = deepcopy(waypoints[-1])

            start_wp_pose = self.linestring.interpolate(distance_start)
            waypoints[0].position.x = start_wp_pose.x
            waypoints[0].position.y = start_wp_pose.y

            end_wp_pose = self.linestring.interpolate(distance_end)
            waypoints[-1].position.x = end_wp_pose.x
            waypoints[-1].position.y = end_wp_pose.y

            waypoints[0].speed = float(self.get_speed_at_distance(distance_start))
            waypoints[-1].speed = float(self.get_speed_at_distance(distance_end))

            waypoints[0].speed_limit = float(self.get_speed_limit_at_distance(distance_start))
            waypoints[-1].speed_limit = float(self.get_speed_limit_at_distance(distance_end))

            waypoints[0].turn_signal = int(self.get_turn_signal_at_distance(distance_start))
            waypoints[-1].turn_signal = int(self.get_turn_signal_at_distance(distance_end))

            waypoints[0].left_boundary_point = self.get_left_boundary_point_at_distance(distance_start)
            waypoints[-1].left_boundary_point = self.get_left_boundary_point_at_distance(distance_end)

            waypoints[0].right_boundary_point = self.get_right_boundary_point_at_distance(distance_start)
            waypoints[-1].right_boundary_point = self.get_right_boundary_point_at_distance(distance_end)

        return waypoints

    def extract_points_and_distances(self, distance_start, distance_end):
        """
        Get waypoints and their distances between start and end distance along the path
        :param distance_start: start distance along the path (m)
        :param distance_end: end distance along the path (m)
        :return: points and distances as numpy arrays
        """

        index_start = self.get_waypoint_index_at_distance(distance_start, side="right")
        index_end = self.get_waypoint_index_at_distance(distance_end, side="left")

        distances = self.distances[index_start:index_end]
        points = self.centerline_array[index_start:index_end]
        points = shapely.points(points)

        # interpolate and add start and end points
        distances = np.insert(distances, 0, distance_start)
        points = np.insert(points, 0, self.linestring.interpolate(distance_start))
        distances = np.append(distances, distance_end)
        points = np.append(points, self.linestring.interpolate(distance_end))

        return points, distances

    def get_speed_at_distance(self, distance):
        """
        Get the target speed at a certain distance along the path.
        :param distance: array of distances or a single distance from the path start (m)
        :return: target speed
        """
        return self._distance_to_speed_interpolator(distance)

    def get_speed_limit_at_distance(self, distance):
        """
        Get the speed limit at a certain distance along the path.
        :param distance: array of distances or a single distance from the path start (m)
        :return: speed limit
        """
        return self._distance_to_speed_limit_interpolator(distance)

    def get_turn_signal_with_lookahead(self, ego_distance_from_path_start, turn_signal_lookahead_distance):
        """
        Get turn signal with lookahead.
        :param ego_distance_from_path_start: distance from path start (m)
        :param turn_signal_lookahead_distance: distance to look ahead point for turn signals (m)
        :return: Waypoint turn signal constant (TURN_LEFT, TURN_RIGHT, TURN_STRAIGHT)
        """
        wapoints = self.extract_waypoints(ego_distance_from_path_start, ego_distance_from_path_start + turn_signal_lookahead_distance, trim=False, copy=False)
        # iterate over waypoints and return first non-straight turn signal
        for waypoint in wapoints:
            if waypoint.turn_signal != Waypoint.TURN_STRAIGHT:
                return waypoint.turn_signal
        return Waypoint.TURN_STRAIGHT

    def get_turn_signal_at_distance(self, distance):
        """
        Get turn signal state.
        :param distance: array of distances or a single distance from path start (m)
        :return: turn signal state
        """
        return self._distance_to_turn_signal_interpolator(distance)

    def get_lane_change_state_at_distance(self, distance):
        """
        Get lane change state.
        :param ego_distance_from_path_start: array of distances or a single distance from path start (m)
        :return: lane change state
        """
        return self._distance_to_lane_change_interpolator(distance).astype(int)

    def check_turn_signal_in_range(self, distance_start, distance_end, turn_signal):
        """
        Check if a certain turn signal is present in a certain distance range along the path
        :param distance_start: start distance along the path (m)
        :param distance_end: end distance along the path (m)
        :param turn_signal: turn signal to check for
        :return: True if turn signal is found, False otherwise
        """

        waypoints = self.extract_waypoints(distance_start, distance_end, trim=False, copy=False)
        for waypoint in waypoints:
            if waypoint.turn_signal == turn_signal:
                return True
        return False

    def get_priority_at_distance(self, distance):
        """
        Get priority at a certain distance along the path.
        :param distance: distance from the path start (m)
        :return: priority
        """
        return self._distance_to_priority_interpolator(distance)

    def get_elevation_at_distance(self, distance):
        """
        Get the elevation at a certain distance along the path.
        :param distance: distance from the path start (m)
        :return: elevation
        """
        return self.linestring.interpolate(distance).z

    def get_pose_at_distance(self, distance):
        """
        Get pose at a certain distance along the path
        :param distance: distance along the path (m)
        :return: Pose
        """

        # Find the point on the path
        point = self.linestring.interpolate(distance)
        heading = self.get_heading_at_distance(distance)

        return Pose(position = Point(x=point.x, y=point.y, z=point.z),
                    orientation = get_orientation_from_heading(heading))

    def get_point_at_distance(self, distance):
        """
        Get a path point at a certain distance along the path.
        :param distance: array of distances or a single distance from the path start (m)
        :return: list of Points or Point
        """
        point = self.linestring.interpolate(distance)
        if isinstance(point, shapely.Point):
            return Point(x=point.x, y=point.y, z=point.z)
        else:
            return [Point(x=p.x, y=p.y, z=p.z) for p in point]

    def get_heading_at_distance(self, distance):
        """
        Get heading of the path at a given distance
        :param distance: array of distances or a single distance along the path
        :return: heading angle in radians
        """
        return self._distance_to_heading_interpolator(distance)

    def get_left_boundary_point_at_distance(self, distance):
        """
        Get the left boundary point at a certain distance along the centerline.
        :param distance: array of distances or a single distance from the path start (m)
        :return: left boundary point(s)
        """
        boundary_distance = np.nan_to_num(self._centerline_to_left_boundary_distance(distance), nan=0.0)
        point = self.left_boundary.interpolate(boundary_distance)
        if isinstance(point, shapely.Point):
            return Point(x=point.x, y=point.y, z=point.z)
        else:
            return [Point(x=p.x, y=p.y, z=p.z) for p in point]

    def get_right_boundary_point_at_distance(self, distance):
        """
        Get the right boundary point at a certain distance along the centerline.
        :param distance: array of distances or a single distance from the path start (m)
        :return: right boundary point(s)
        """
        boundary_distance = np.nan_to_num(self._centerline_to_right_boundary_distance(distance), nan=0.0)
        point = self.right_boundary.interpolate(boundary_distance)
        if isinstance(point, shapely.Point):
            return Point(x=point.x, y=point.y, z=point.z)
        else:
            return [Point(x=p.x, y=p.y, z=p.z) for p in point]


    def get_left_boundary_type_at_distance(self, distance):
        """
        Get left boundary type state at a certain distance along the path.
        :param ego_distance_from_path_start: array of distances or a single distance from path start (m)
        :return: left boundary type
        """
        return self._left_type_interpolator(distance)

    def get_right_boundary_type_at_distance(self, distance):
        """
        Get right boundary type state at a certain distance along the path.
        :param ego_distance_from_path_start: distance from path start (m)
        :return: right boundary type
        """
        return self._right_type_interpolator(distance)

    def get_cross_track_error(self, current_position):
        """
        Get cross track error - calc distance from track and get the sign
        :param current_pose: current pose
        :return: cross track error
        """
        current_position = shapely.Point(current_position.x, current_position.y, current_position.z)
        return float(calculate_cross_track_error(self.linestring, current_position))

    def get_heading_towards_path(self, point):
        """
        Get heading from point towards the closest point on path
        :param point: Shapely point
        :return: heading angle in radians
        """
        distance = self.linestring.project(point)
        location = self.linestring.interpolate(distance)
        return get_heading_between_two_points(point, location)
