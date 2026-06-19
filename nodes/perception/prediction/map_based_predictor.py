#!/usr/bin/env python3

import rospy
import numpy as np
import shapely
import lanelet2
from functools import lru_cache

from autoware_mini.msg import DetectedObjectArray, Path, Waypoint, StopLineStatusArray, StopLineStatus
from geometry_msgs.msg import PoseStamped

from autoware_mini.geometry import get_angle_between_two_headings, get_distance_between_two_points_2d
from autoware_mini.detection import update_object_position_dimensions
from autoware_mini.lanelet2 import load_lanelet2_map, follow_lanelets, get_stop_lines, get_lanelets_in_range
from autoware_mini.shapely import offset_curves, ensure_points, calculate_cross_track_error, calculate_linestring_heading_at_distance

# Normalization constants for lanelet matching cost calculation
CROSS_TRACK_NORM_METERS = 2.0
HEADING_NORM_DEGREES = 30.0
SPEED_NORM_MPS = 11.1  # ~40 km/h

DEFAULT_SPEED_LIMIT_KMH = 50.0
TRAJECTORY_SIMPLIFY_TOLERANCE = 0.1  # meters

TURN_PENALTY = {
    'straight': {'straight': 0, 'left': 2, 'right': 2},
    'left':     {'left': 0, 'straight': 1, 'right': 2},
    'right':    {'right': 0, 'straight': 1, 'left': 2},
}

# Structured array dtype for extracted object data (1:1 with msg.objects)
OBJECT_DTYPE = np.dtype([
    ('x', np.float64),
    ('y', np.float64),
    ('z', np.float64),
    ('heading', np.float64),
    ('half_length', np.float64),
    ('half_height', np.float64),
    ('speed', np.float64),
    ('accel', np.float64),
    ('match_point', object),         # 2d object center point used for matching
])

# Structured array dtype for object-lanelet matches
MATCH_DTYPE = np.dtype([
    ('obj_idx', np.int32),          # Index into objects/msg.objects
    ('lanelet', object),            # Lanelet object reference
    ('linestring', object),         # Centerline LineString
    ('lanelet_speed', np.float64),  # Speed limit from lanelet
    ('cross_track', np.float64),    # Cross-track offset
    ('distance', np.float64),       # Distance along lanelet
    ('heading_diff', np.float64),   # Heading difference in degrees
    ('heading', np.float64),        # Lanelet heading in radians
    ('cost', np.float64),           # Matching cost
])

# Structured array dtype for trajectories
TRAJ_DTYPE = np.dtype([
    ('obj_idx', np.int32),          # Index into objects/msg.objects
    ('lanelets', object),           # Tuple of lanelets in trajectory
    ('start_dist', np.float64),     # Starting distance along first lanelet
    ('cross_track', np.float64),    # Cross-track offset
    ('cost', np.float64),           # Cost from matching
    ('heading', np.float64),        # Lanelet heading in radians at object position
    ('end_lanelet_id', np.int64),   # For deduplication
    ('linestring', object),         # Built trajectory linestring
    ('traj_length', np.float64),    # Trajectory length (may be clipped at stop line)
])

class MapBasedPredictor:
    def __init__(self):
        # Parameters
        self.prediction_horizon = rospy.get_param('~prediction_horizon')
        self.prediction_interval = rospy.get_param('~prediction_interval')
        self.trajectories_to_predict = rospy.get_param('~trajectories_to_predict')
        self.prediction_min_speed = rospy.get_param('~prediction_min_speed')
        self.distance_from_lanelet = rospy.get_param('~distance_from_lanelet')
        self.heading_difference_threshold = rospy.get_param('~heading_difference_threshold')
        self.prediction_clipping_deceleration_limit = rospy.get_param('~prediction_clipping_deceleration_limit')
        self.max_height_difference = rospy.get_param('~max_height_difference')
        self.matching_cross_track_weight = rospy.get_param('~matching_cross_track_weight')
        self.matching_heading_weight = rospy.get_param('~matching_heading_weight')
        self.matching_speed_weight = rospy.get_param('~matching_speed_weight')
        self.max_prediction_length = rospy.get_param('~max_prediction_length')
        lanelet2_map_path = rospy.get_param("~lanelet2_map_path")
        self.local_path_length = rospy.get_param("/planning/local_path_length")

        # Variables
        self.lanelet2_map = load_lanelet2_map(lanelet2_map_path)
        traffic_rules_vehicle = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany, lanelet2.traffic_rules.Participants.Vehicle)
        traffic_rules_taxi = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany, lanelet2.traffic_rules.Participants.VehicleTaxi)
        traffic_rules_bicycle = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany, lanelet2.traffic_rules.Participants.Bicycle)
        self.graph_vehicle = lanelet2.routing.RoutingGraph(self.lanelet2_map, traffic_rules_vehicle)
        # TODO Use Taxi routing graph for bus lanes until #207 is solved and bus_lane support is added to lanelet2 ParticipantsMap
        self.graph_bus = lanelet2.routing.RoutingGraph(self.lanelet2_map, traffic_rules_taxi)
        self.graph_bicycle = lanelet2.routing.RoutingGraph(self.lanelet2_map, traffic_rules_bicycle)
        num_timesteps = int(self.prediction_horizon // self.prediction_interval) + 1
        self.timesteps = np.arange(num_timesteps) * self.prediction_interval
        self.last_stop_line_extract_location = None
        self.stop_lines_in_area = None
        self.stop_lines_tfl_filtered = None

        # Publishers
        self.predicted_objects_pub = rospy.Publisher('predicted_objects', DetectedObjectArray, queue_size=1, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber('tracked_objects', DetectedObjectArray, self.tracked_objects_callback, queue_size=1, buff_size=2**20, tcp_nodelay=True)
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/perception/traffic_light_status', StopLineStatusArray, self.traffic_light_status_callback, queue_size=1, tcp_nodelay=True)

        rospy.loginfo("%s - initialized", rospy.get_name())

    @staticmethod
    @lru_cache(maxsize=None)
    def get_centerline_coords(lanelet):
        """Get or create cached centerline coordinates for a lanelet."""
        return np.array([(p.x, p.y, p.z) for p in lanelet.centerline])

    @staticmethod
    @lru_cache(maxsize=None)
    def get_centerline_linestring(lanelet):
        """Get or create cached centerline linestring for a lanelet."""
        return shapely.linestrings(MapBasedPredictor.get_centerline_coords(lanelet))

    @staticmethod
    @lru_cache(maxsize=None)
    def get_turn_direction(lanelet):
        """Get turn direction from lanelet attributes, defaulting to straight."""
        return lanelet.attributes["turn_direction"] if "turn_direction" in lanelet.attributes else "straight"

    def get_lanelet_speed(self, lanelet):
        """Get speed limit from lanelet attributes in m/s."""
        if 'speed_ref' in lanelet.attributes:
            return float(lanelet.attributes['speed_ref']) / 3.6
        if 'speed_limit' in lanelet.attributes:
            return float(lanelet.attributes['speed_limit']) / 3.6
        return DEFAULT_SPEED_LIMIT_KMH / 3.6

    def get_stop_lines(self):
        """Get stop lines array, preferring TFL-filtered if available."""
        if self.stop_lines_tfl_filtered is not None:
            stop_lines_dict = self.stop_lines_tfl_filtered
        elif self.stop_lines_in_area is not None:
            stop_lines_dict = self.stop_lines_in_area
        else:
            return None
        return np.array(list(stop_lines_dict.values()), dtype=object)

    def extract_objects(self, msg_objects):
        """Extract object data from msg.objects into a structured array (1:1 mapping)."""
        # Use list comprehensions for efficient extraction, then build structured array
        positions = np.array([(o.center.x, o.center.y, o.center.z) for o in msg_objects])
        headings = np.array([o.heading for o in msg_objects])
        half_lengths = np.array([o.dimensions.x / 2 for o in msg_objects])
        half_heights = np.array([o.dimensions.z / 2 for o in msg_objects])
        velocities = np.array([(o.velocity.x, o.velocity.y) for o in msg_objects])
        accelerations = np.array([(o.acceleration.x, o.acceleration.y) for o in msg_objects])

        objects = np.empty(len(msg_objects), dtype=OBJECT_DTYPE)
        objects['x'] = positions[:, 0]
        objects['y'] = positions[:, 1]
        objects['z'] = positions[:, 2]
        objects['heading'] = headings
        objects['half_length'] = half_lengths
        objects['half_height'] = half_heights
        objects['speed'] = np.hypot(velocities[:, 0], velocities[:, 1])
        objects['accel'] = np.hypot(accelerations[:, 0], accelerations[:, 1])
        objects['match_point'] = shapely.points(positions[:, :2])
        return objects

    def find_lanelet_matches(self, objects):
        """Find lanelet matches for objects with speed >= min_speed."""
        match_list = []
        for i in range(len(objects)):
            if objects['speed'][i] < self.prediction_min_speed:
                continue
            lanelets = get_lanelets_in_range(
                self.lanelet2_map, objects['x'][i], objects['y'][i], objects['z'][i],
                self.distance_from_lanelet, ["road", "bus_lane", "bicycle_lane"],
                self.max_height_difference + objects['half_height'][i]
            )
            for lanelet in lanelets:
                match_list.append((
                    i, lanelet, self.get_centerline_linestring(lanelet),
                    self.get_lanelet_speed(lanelet), 0.0, 0.0, 0.0, 0.0, 0.0
                ))

        matches = np.array(match_list, dtype=MATCH_DTYPE)
        if matches.size == 0:
            return matches

        # Vectorized computation of cross_track, distance, heading_diff, cost
        obj_indices = matches['obj_idx']
        match_points = objects['match_point'][obj_indices]
        match_linestrings = matches['linestring']

        # Cross-track offset (negated so positive = right of centerline)
        cross_track = -calculate_cross_track_error(match_linestrings, match_points)
        matches['cross_track'] = cross_track

        # Distance along centerline provides approximate longitudinal position
        matches['distance'] = shapely.line_locate_point(match_linestrings, match_points)

        # Heading difference
        lanelet_headings = calculate_linestring_heading_at_distance(match_linestrings, matches['distance'])
        heading_diff = np.degrees(get_angle_between_two_headings(objects['heading'][obj_indices], lanelet_headings))
        matches['heading_diff'] = heading_diff
        matches['heading'] = lanelet_headings

        # Matching cost
        speed_diff = np.abs(objects['speed'][obj_indices] - matches['lanelet_speed'])
        matches['cost'] = (
            self.matching_cross_track_weight * np.abs(cross_track) / CROSS_TRACK_NORM_METERS +
            self.matching_heading_weight * heading_diff / HEADING_NORM_DEGREES +
            self.matching_speed_weight * speed_diff / SPEED_NORM_MPS
        )

        return matches

    def filter_matches(self, matches):
        """Filter matches by heading threshold."""
        return matches[matches['heading_diff'] < self.heading_difference_threshold]

    def build_trajectories(self, matches, objects):
        """Build trajectories by following lanelets from each match."""
        traj_list = []
        last_timestep = self.timesteps[-1]

        for i in range(len(matches)):
            obj_idx = matches['obj_idx'][i]
            lanelet = matches['lanelet'][i]
            distance = matches['distance'][i]
            cross_track = matches['cross_track'][i]
            heading = matches['heading'][i]
            cost = matches['cost'][i]

            # Compute prediction length
            pred_length = (
                objects['accel'][obj_idx] * last_timestep**2 / 2 +
                objects['speed'][obj_idx] * last_timestep +
                distance + objects['half_length'][obj_idx]
            )

            # Select routing graph based on lane subtype
            subtype = lanelet.attributes["subtype"] if "subtype" in lanelet.attributes else None
            if subtype == "bus_lane":
                routing_graph = self.graph_bus
            elif subtype == "bicycle_lane":
                routing_graph = self.graph_bicycle
            else:
                routing_graph = self.graph_vehicle

            # Follow lanelets to build all possible trajectories
            for lanelets in follow_lanelets(routing_graph, lanelet, min(pred_length, self.max_prediction_length)):
                traj_list.append((
                    obj_idx, tuple(lanelets), 0.0, cross_track, cost,
                    heading, lanelets[-1].id, None, 0.0
                ))

        trajectories = np.array(traj_list, dtype=TRAJ_DTYPE)
        if trajectories.size == 0:
            return trajectories

        # Build combined linestrings for each trajectory
        linestrings = np.array([
            shapely.linestrings(np.concatenate([self.get_centerline_coords(ll) for ll in traj['lanelets']]))
            for traj in trajectories
        ], dtype=object)
        linestrings = shapely.simplify(linestrings, TRAJECTORY_SIMPLIFY_TOLERANCE)

        # Apply cross-track offset
        nonzero_mask = trajectories['cross_track'] != 0
        if np.any(nonzero_mask):
            nonzero_indices = np.where(nonzero_mask)[0]
            coords_list = [shapely.get_coordinates(ls, include_z=True) for ls in linestrings[nonzero_indices]]
            offset_coords = offset_curves(coords_list, trajectories['cross_track'][nonzero_indices])
            # Per-element conversion needed because trajectories have varying vertex counts
            linestrings[nonzero_indices] = [shapely.linestrings(c) for c in offset_coords]

        # Update linestring and its lengths
        trajectories['linestring'] = linestrings
        trajectories['traj_length'] = shapely.length(linestrings)

        # Recalculate start_dist on offset path
        obj_indices = trajectories['obj_idx']
        match_points = objects['match_point'][obj_indices]
        half_lengths = objects['half_length'][obj_indices]
        trajectories['start_dist'] = shapely.line_locate_point(linestrings, match_points) + half_lengths

        # Filter out trajectories where start_dist is at or beyond end
        valid_mask = trajectories['start_dist'] < trajectories['traj_length']
        trajectories = trajectories[valid_mask]

        return trajectories

    def filter_trajectories(self, trajectories):
        """Deduplicate trajectories and score by turn alignment, limit to top N per object."""

        # Deduplicate by (obj_idx, end_lanelet_id), keeping lowest cost
        # Sort by (obj_idx, end_lanelet_id, cost)
        sort_order = np.lexsort((trajectories['cost'], trajectories['end_lanelet_id'], trajectories['obj_idx']))
        trajectories = trajectories[sort_order]

        # Keep first occurrence of each (obj_idx, end_lanelet_id) pair
        _, unique_idx = np.unique(
            np.column_stack([trajectories['obj_idx'], trajectories['end_lanelet_id']]),
            axis=0, return_index=True
        )
        trajectories = trajectories[unique_idx]

        if trajectories.size == 0:
            return trajectories

        # Get group boundaries for filtering by turn alignment
        _, first_idx, counts = np.unique(trajectories['obj_idx'], return_index=True, return_counts=True)

        # Build list of indices to keep
        keep_indices = []
        for i in range(len(first_idx)):
            start = first_idx[i]
            count = counts[i]

            if count <= self.trajectories_to_predict:
                keep_indices.extend(range(start, start + count))
            else:
                obj_trajs = trajectories[start:start + count]
                scores = obj_trajs['cost'] + np.array([
                    self.score_trajectory_turn_alignment([self.get_turn_direction(ll) for ll in traj['lanelets']])
                    for traj in obj_trajs
                ])
                order = np.argsort(scores)
                keep_indices.extend(start + order[:self.trajectories_to_predict])

        return trajectories[keep_indices]

    def trim_trajectories(self, trajectories, objects, stop_lines):
        """Trim trajectories at stop lines."""
        if stop_lines.size == 0:
            return trajectories

        # Batch intersection check
        intersects = shapely.intersects(
            trajectories['linestring'][:, np.newaxis],
            stop_lines[np.newaxis, :]
        )

        for i in np.where(np.any(intersects, axis=1))[0]:
            traj = trajectories[i]
            obj = objects[traj['obj_idx']]
            traj_ls = traj['linestring']

            # Get intersection points with stop lines
            stop_lines_hit = stop_lines[intersects[i]]
            intersection_results = traj_ls.intersection(stop_lines_hit)
            stop_line_points = ensure_points(intersection_results)
            distances_to_stop_lines = sorted(traj_ls.project(stop_line_points))
            start_dist = traj['start_dist']

            for d in distances_to_stop_lines:
                if start_dist < d < traj['traj_length']:
                    deceleration_distance = d - start_dist
                    deceleration = (obj['speed']**2) / (2 * deceleration_distance)
                    if deceleration < self.prediction_clipping_deceleration_limit:
                        trajectories[i]['traj_length'] = min(traj['traj_length'], d)
                        break

        return trajectories

    def create_predictions(self, msg_objects, trajectories, objects):
        """Create prediction waypoints and add to msg_objects."""
        # Precompute velocity and distance profiles per object (avoids redundant computation)
        unique_obj_indices = np.unique(trajectories['obj_idx'])
        obj_speeds = {}
        obj_distances = {}
        for obj_idx in unique_obj_indices:
            obj = objects[obj_idx]
            obj_speeds[obj_idx] = obj['speed'] + obj['accel'] * self.timesteps
            obj_distances[obj_idx] = obj['accel'] * self.timesteps**2 / 2 + obj['speed'] * self.timesteps

        for traj in trajectories:
            obj_idx = traj['obj_idx']
            msg_obj = msg_objects[obj_idx]

            traj_ls = traj['linestring']
            traj_length = traj['traj_length']

            speeds = obj_speeds[obj_idx]
            distances = obj_distances[obj_idx]
            interpolate_distances = distances + traj['start_dist']

            if traj_length <= interpolate_distances[0]:
                continue

            # Clip distances that extend beyond trajectory
            if interpolate_distances[-1] > traj_length:
                index = np.searchsorted(interpolate_distances, traj_length)
                interpolate_distances = np.append(interpolate_distances[:index], traj_length)

            # Interpolate points along trajectory
            points = traj_ls.interpolate(interpolate_distances)
            coords = shapely.get_coordinates(points, include_z=True)

            # Build path message
            path = Path()
            for (x, y, z), speed in zip(coords.tolist(), speeds.tolist()):
                wp = Waypoint()
                wp.position.x = x
                wp.position.y = y
                wp.position.z = z
                wp.speed = speed
                path.waypoints.append(wp)
            msg_obj.candidate_trajectories.paths.append(path)

    def current_pose_callback(self, msg):
        if self.last_stop_line_extract_location is not None and get_distance_between_two_points_2d(self.last_stop_line_extract_location, msg.pose.position) < self.local_path_length:
            return
        # Fetch stop lines within a range (2 x local_path length) around the current position
        stop_lines_in_area = get_stop_lines(self.lanelet2_map, msg.pose.position.x, msg.pose.position.y, 2 * self.local_path_length,
                                            subtypes=["yield", "yield_stop", "yield_manual", "yield_right"])

        self.stop_lines_in_area = stop_lines_in_area
        self.last_stop_line_extract_location = msg.pose.position

    def traffic_light_status_callback(self, msg):

        if self.stop_lines_in_area is None:
            return

        # Copy stop lines and remove those with GO status
        self.stop_lines_tfl_filtered = self.stop_lines_in_area.copy()
        for status in msg.statuses:
            if status.status == StopLineStatus.STATUS_GO and status.stop_line_id in self.stop_lines_tfl_filtered:
                del self.stop_lines_tfl_filtered[status.stop_line_id]

    def tracked_objects_callback(self, msg):
        stop_lines = self.get_stop_lines()
        if stop_lines is None:
            return

        if not msg.objects:
            self.predicted_objects_pub.publish(msg)
            return

        # Extract object data into array (1:1 with msg.objects)
        objects = self.extract_objects(msg.objects)

        # Find and score lanelet matches (filters by min speed internally)
        matches = self.find_lanelet_matches(objects)
        if matches.size == 0:
            self.predicted_objects_pub.publish(msg)
            return

        # Filter matches by heading and select top N by cost per object
        matches = self.filter_matches(matches)
        if matches.size == 0:
            self.predicted_objects_pub.publish(msg)
            return

        # Build trajectories by following lanelets
        trajectories = self.build_trajectories(matches, objects)
        if trajectories.size == 0:
            self.predicted_objects_pub.publish(msg)
            return

        # Deduplicate and score trajectories by turn alignment
        trajectories = self.filter_trajectories(trajectories)
        if trajectories.size == 0:
            self.predicted_objects_pub.publish(msg)
            return

        # Update object heading using best trajectory's lanelet heading
        unique_obj_indices, first_idx, counts = np.unique(trajectories['obj_idx'], return_index=True, return_counts=True)
        for obj_idx, start, count in zip(unique_obj_indices, first_idx, counts):
            obj_trajs = trajectories[start:start + count]
            best_idx = np.argmin(obj_trajs['cost'])
            msg.objects[obj_idx].heading = float(obj_trajs['heading'][best_idx])
            update_object_position_dimensions(msg.objects[obj_idx])

        # Trim trajectories at stop lines
        trajectories = self.trim_trajectories(trajectories, objects, stop_lines)

        # Create prediction waypoints
        self.create_predictions(msg.objects, trajectories, objects)

        self.predicted_objects_pub.publish(msg)

    @staticmethod
    def score_trajectory_turn_alignment(turn_directions):
        """Score trajectory by summing turn direction change penalties.

        Returns a single float (lower = better). Only actual direction changes between
        consecutive lanelets contribute to the score (same-direction transitions cost 0).
        """
        previous = turn_directions[0]
        score = 0.0
        for turn in turn_directions:
            score += TURN_PENALTY[previous][turn]
            previous = turn
        return score

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('map_based_predictor', log_level=rospy.INFO)
    node = MapBasedPredictor()
    node.run()
