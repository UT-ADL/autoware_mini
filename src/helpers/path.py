import math
import numpy as np
from scipy.interpolate import interp1d
from autoware_msgs.msg import WaypointState, Waypoint
from geometry_msgs.msg import Point, Pose
from helpers.geometry import get_heading_between_two_points, get_orientation_from_heading
from shapely.geometry import LineString, Point as ShapelyPoint
from shapely import prepare


class Path:
    def __init__(self, waypoints, velocities=False, blinkers=False):

        self.waypoints = waypoints
        self._waypoints_xyz = np.array([(waypoint.pose.pose.position.x, waypoint.pose.pose.position.y, waypoint.pose.pose.position.z) for waypoint in self.waypoints])

        self.linestring = LineString(self._waypoints_xyz)
        prepare(self.linestring)

        d = np.cumsum(np.sqrt(np.sum(np.diff(self._waypoints_xyz[:, :2], axis=0)**2, axis=1)))
        self._distances = np.insert(d, 0, 0)

        if velocities:
            v = np.array([waypoint.twist.twist.linear.x for waypoint in self.waypoints])
            distance_to_velocity_interpolator = interp1d(self._distances, v, kind='linear', bounds_error=False, fill_value=0.0)
            self._distance_to_velocity_interpolator = distance_to_velocity_interpolator

        if blinkers:
            b = np.array([(waypoint.wpstate.steering_state) for waypoint in self.waypoints])
            distance_to_blinker_interpolator = interp1d(self._distances, (b).astype(np.float) , kind='previous', bounds_error=False, fill_value=WaypointState.STR_STRAIGHT)
            self._distance_to_blinker_interpolator = distance_to_blinker_interpolator

    def get_waypoint_index_at_distance(self, distance, side="left"):
        """
        Get waypoint at a certain distance along the path
        :param distance: distance along the path (m)
        :param side: side to search for the distance
        :return: waypoint
        """

        return np.searchsorted(self._distances, distance, side)

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
                new_waypoint = Waypoint(pose = waypoint.pose, wpstate = waypoint.wpstate)
                new_waypoint.twist.twist.linear.x = waypoint.twist.twist.linear.x
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
        waypoints = self._extract_waypoints(index_start, index_end, copy=copy)

        if trim:
            # modify start and end of the path by shifting waypoints to exact locations determined by distances
            waypoints[0].twist.twist.linear.x = float(self._distance_to_velocity_interpolator(distance_start))
            waypoints[0].wpstate.steering_state = int(self._distance_to_blinker_interpolator(distance_start))
            start_wp_pose = self.linestring.interpolate(distance_start)
            # z will remain the same
            waypoints[0].pose.pose.position.x = start_wp_pose.x
            waypoints[0].pose.pose.position.y = start_wp_pose.y

            waypoints[-1].twist.twist.linear.x = float(self._distance_to_velocity_interpolator(distance_end))
            waypoints[-1].wpstate.steering_state = int(self._distance_to_blinker_interpolator(distance_end))
            end_wp_pose = self.linestring.interpolate(distance_end)
            # z will remain the same
            waypoints[-1].pose.pose.position.x = end_wp_pose.x
            waypoints[-1].pose.pose.position.y = end_wp_pose.y

        return waypoints


    def get_velocity_at_distance(self, distance):
        """
        Get the target velocity at a certain distance along the path.
        :param distance: distance from the path start (m)
        :return: target velocity
        """
        assert hasattr(self, '_distance_to_velocity_interpolator'), "Velocity interpolator not available, check that path was initialized with velocities=True"
        return float(self._distance_to_velocity_interpolator(distance))

    def get_blinker_state_with_lookahead(self, ego_distance_from_path_start, blinker_lookahead_distance):
        """
        Get blinker state. 
        :param ego_distance_from_path_start: distance from path start (m)
        :param blinker_lookahead_d: distance to look ahead point for blinkers (m)
        :return: LampCmd (l, r) included in VehicleCmd
        """
        assert hasattr(self, '_distance_to_blinker_interpolator'), "Blinker interpolator not available, check that path was initialized with blinkers=True"
        current_pose_blinker_state = int(self._distance_to_blinker_interpolator(ego_distance_from_path_start))

        if current_pose_blinker_state != WaypointState.STR_STRAIGHT:
            return get_blinker_state(current_pose_blinker_state)
        else:
            lookahead_blinker_state = int(self._distance_to_blinker_interpolator(blinker_lookahead_distance))
            return get_blinker_state(lookahead_blinker_state)


    def get_pose_at_distance(self, distance):
        """
        Get perpendicular pose at a certain distance along the path
        :param distance: distance along the path (m)
        :return: Pose
        """

        # Find the point on the path
        point_location = self.linestring.interpolate(distance)
        # if distance is negative it is measured from the end of the linestring in reverse direction
        point_before = self.linestring.interpolate(max(0, distance - 0.1))

        heading = get_heading_between_two_points(point_before, point_location)

        pose = Pose(position = Point(x = point_location.x, y = point_location.y, z = point_location.z),
                    orientation = get_orientation_from_heading(heading))

        return pose


    def get_heading_at_distance(self, distance):
        """
        Get heading of the path at a given distance
        :param distance: distance along the path
        :return: heading angle in radians
        """

        point_after_object = self.linestring.interpolate(distance + 0.1)
        # if distance is negative it is measured from the end of the linestring in reverse direction
        point_before_object = self.linestring.interpolate(max(0, distance - 0.1))

        # get heading between two points
        path_heading = math.atan2(point_after_object.y - point_before_object.y, point_after_object.x - point_before_object.x)

        return path_heading


    def get_cross_track_error(self, current_position):
        """
        Get cross track error - calc distance from track and get the sign
        https://robotics.stackexchange.com/questions/22989/what-is-wrong-with-my-stanley-controller-for-car-steering-control
        
        :param current_pose: current pose
        :return: cross track error
        """

        current_position = ShapelyPoint(current_position.x, current_position.y, current_position.z)

        ego_distance_from_path_start = self.linestring.project(current_position)

        # if distance is negative it is measured from the end of the linestring in reverse direction
        pos1 = self.linestring.interpolate(max(0, ego_distance_from_path_start - 0.1))
        pos2 = self.linestring.interpolate(ego_distance_from_path_start + 0.1)

        numerator = (pos2.x - pos1.x) * (pos1.y - current_position.y) - (pos1.x - current_position.x) * (pos2.y - pos1.y)
        denominator = math.sqrt((pos2.x - pos1.x) ** 2 + (pos2.y - pos1.y) ** 2)

        return numerator / denominator


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