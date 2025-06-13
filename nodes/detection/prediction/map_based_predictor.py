#!/usr/bin/env python3

import rospy
import math
import numpy as np
import shapely
import lanelet2

from autoware_mini.msg import DetectedObjectArray, Path, Waypoint
from geometry_msgs.msg import PoseStamped

from autoware_mini.path import calculate_cross_track_error
from autoware_mini.geometry import get_speed_from_velocity, get_heading_between_two_points, get_angle_between_two_headings, get_distance_between_two_points_2d
from autoware_mini.lanelet2 import load_lanelet2_map, follow_lanelets, get_stop_lines_in_area, get_lanelets_in_range
from autoware_mini.shapely import offset_curve

CAR_INDICATOR_VS_TURN_DIRECTION_SCORING = {
    'straight': {'straight': 1, 'left': 0.5, 'right': 0.5},
    'left': {'straight': 0.5, 'left': 1, 'right': -1},
    'right': {'straight': 0.5, 'left': -1, 'right': 1}
}

class MapBasedPredictor:
    def __init__(self):
        # Parameters
        self.prediction_horizon = rospy.get_param('~prediction_horizon')
        self.prediction_interval = rospy.get_param('~prediction_interval')
        self.trajectories_to_predict = rospy.get_param('~trajectories_to_predict')
        self.prediction_min_speed = rospy.get_param('~prediction_min_speed')
        self.distance_from_lanelet = rospy.get_param('~distance_from_lanelet')
        self.heading_difference_threshold = rospy.get_param('~heading_difference_threshold')
        self.use_offset_for_prediction = rospy.get_param('~use_offset_for_prediction')
        self.prediction_clipping_deceleration_limit = rospy.get_param('~prediction_clipping_deceleration_limit')
        lanelet2_map_path = rospy.get_param("~lanelet2_map_path")
        self.local_path_length = rospy.get_param("/planning/local_path_length")

        # Variables
        self.lanelet2_map = load_lanelet2_map(lanelet2_map_path)
        traffic_rules_vehicle = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany, lanelet2.traffic_rules.Participants.Vehicle)
        traffic_rules_vehicle_taxi = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany, lanelet2.traffic_rules.Participants.VehicleTaxi)
        self.graph_vehicle = lanelet2.routing.RoutingGraph(self.lanelet2_map, traffic_rules_vehicle)
        self.graph_vehicle_taxi = lanelet2.routing.RoutingGraph(self.lanelet2_map, traffic_rules_vehicle_taxi)
        num_timesteps = int(self.prediction_horizon // self.prediction_interval) + 1
        self.timesteps = np.arange(num_timesteps) * self.prediction_interval
        self.last_stop_line_extract_location = None
        self.stop_lines = []

        # Publishers
        self.predicted_objects_pub = rospy.Publisher('predicted_objects', DetectedObjectArray, queue_size=1, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber('tracked_objects', DetectedObjectArray, self.tracked_objects_callback, queue_size=1, buff_size=2**20, tcp_nodelay=True)
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1, tcp_nodelay=True)

    def current_pose_callback(self, msg):
        if self.last_stop_line_extract_location is not None and get_distance_between_two_points_2d(self.last_stop_line_extract_location, msg.pose.position) < self.local_path_length:
            return
        # Fetch stop lines within a range (2 x local_path length) around the current position
        stop_lines = get_stop_lines_in_area(self.lanelet2_map, msg.pose.position.x, msg.pose.position.y, 2 * self.local_path_length,
                                                ["stop_line", "yield_stop", "yield"])

        shapely.prepare(stop_lines)
        self.stop_lines = np.array(stop_lines)
        self.last_stop_line_extract_location = msg.pose.position

    def tracked_objects_callback(self, msg):

        stop_lines = self.stop_lines

        for obj in msg.objects:
            object_distance_on_start_lanelet = {}
            # when no speed in 2d, don't create any trajectories
            if get_speed_from_velocity(obj.velocity) < self.prediction_min_speed:
                continue

            # 1. SEARCH BEST MATCHING LANELET FOR AN OBJECT
            object_position = shapely.Point(obj.center.x, obj.center.y)

            # find lanelets within distance to object_location - distance measured from lanelet borders. Inside lanelet area this distance would be 0
            lanelets_within_distance = get_lanelets_in_range(self.lanelet2_map, obj.center.x, obj.center.y, self.distance_from_lanelet,
                                                             ["road", "bus_lane", "bicycle_lane"])

            selected_lanelets = []
            for lanelet in lanelets_within_distance:

                # Calculate heading difference
                linestring = shapely.LineString([(p.x, p.y) for p in lanelet.centerline])
                object_distance_from_start = linestring.project(object_position)
                # skip lanelet if object front is over it and there are no following lanelets
                if (object_distance_from_start + obj.dimensions.x / 2) > linestring.length and not self.graph_vehicle_taxi.following(lanelet):
                    continue
                object_location_on_lanelet = linestring.interpolate(object_distance_from_start)
                forward_point = linestring.interpolate(object_distance_from_start + 0.1)
                lanelet_heading = get_heading_between_two_points(object_location_on_lanelet, forward_point)
                heading_difference_degrees = math.degrees(get_angle_between_two_headings(obj.heading, lanelet_heading))

                # Add lanelet if heading difference is within threshold
                if heading_difference_degrees < self.heading_difference_threshold:
                    object_distance_on_start_lanelet[lanelet.id] = object_distance_from_start
                    selected_lanelets.append((lanelet, object_distance_from_start, heading_difference_degrees))

            # Sort by heading difference and limit selection to match `trajectories_to_predict`
            if len(selected_lanelets) > self.trajectories_to_predict:
                selected_lanelets.sort(key=lambda l: l[2])
                selected_lanelets = selected_lanelets[:self.trajectories_to_predict]

            # 2. CREATE ALL TRAJECTORIES
            all_trajectories = []
            if len(selected_lanelets) > 0:
                object_speed = get_speed_from_velocity(obj.velocity)
                object_accel = get_speed_from_velocity(obj.acceleration)
                velocities = object_speed + object_accel * self.timesteps
                distances = (object_accel * self.timesteps**2) / 2 + object_speed * self.timesteps
                all_trajectories = self.create_trajectories(selected_lanelets, distances[-1], obj.dimensions.x)

            # 3. SCORING IF NEEDED
            if len(all_trajectories) > self.trajectories_to_predict:
                trajectory_turn_directions = [[lanelet.attributes["turn_direction"] if "turn_direction" in lanelet.attributes else "straight" for lanelet in trajectory] for trajectory in all_trajectories]
                # Score each trajectory 
                # TODO use first lanelet's turn direction as object indicator, in future should be replaced by object's real indicator information
                scores = [self.score_paths(trajectory_turn_directions[i], trajectory_turn_directions[i][0]) for i in range(len(all_trajectories))]

                # Pair trajectories with their scores, sort and limit the number of trajectories to match `trajectories_to_predict`
                scored_trajectories = list(zip(all_trajectories, scores))
                scored_trajectories.sort(key=lambda t: t[1], reverse=True)
                all_trajectories = [trajectory for trajectory, _ in scored_trajectories[:self.trajectories_to_predict]]

            # 4. CREATE PREDICTIONS AND PUBLISH
            # create shapely linestring from lanelet centerlines and then use it to interpolate points in necessary distances
            for trajectory in all_trajectories:
                centerline_linestring = shapely.simplify(shapely.LineString([(p.x, p.y, p.z) for lanelet in trajectory for p in lanelet.centerline]), 0.1)
                if self.use_offset_for_prediction:
                    cross_track_offset = -calculate_cross_track_error(centerline_linestring, object_position)
                    trajectory_linestring = offset_curve(centerline_linestring, cross_track_offset)
                else:
                    trajectory_linestring = centerline_linestring

                trajectory_limit = trajectory_linestring.length
                interpolate_distances = distances + object_distance_on_start_lanelet[trajectory[0].id] + obj.dimensions.x / 2

                # check intersection with stop_lines
                mask = trajectory_linestring.intersects(stop_lines)
                if np.any(mask):
                    stop_line_intersection_result = trajectory_linestring.intersection(stop_lines[mask])
                    distances_to_stoplines = sorted(trajectory_linestring.project(stop_line_intersection_result))
                    for d in distances_to_stoplines:
                        # check for deceleration if stop line somewhere within the predicted trajectory
                        if interpolate_distances[0] < d < interpolate_distances[-1]:
                            deceleration_distance = d - interpolate_distances[0]
                            deceleration = (object_speed**2) / (2 * deceleration_distance)
                            if deceleration < self.prediction_clipping_deceleration_limit:
                                trajectory_limit = min(trajectory_limit, d)
                                break

                # interpolate_distances extend further than trajectory_linestring (case of dangling lanelets and sometimes also offset curve might reduce
                # its length), so clip the exessive distances otherwise duplicate points cause problems later with triangulation
                if interpolate_distances[-1] > trajectory_limit:
                    index = np.argmax(interpolate_distances > trajectory_limit)
                    interpolate_distances = np.append(interpolate_distances[:index], trajectory_limit)

                points_trajectory = trajectory_linestring.interpolate(interpolate_distances)

                path = Path()
                for i, d in enumerate(interpolate_distances):
                    wp = Waypoint()
                    wp.position.x = points_trajectory[i].x
                    wp.position.y = points_trajectory[i].y
                    wp.position.z = points_trajectory[i].z
                    wp.speed = velocities[i]
                    path.waypoints.append(wp)
                obj.candidate_trajectories.paths.append(path)

        # Publish predicted objects
        self.predicted_objects_pub.publish(msg)

    def create_trajectories(self, start_lanelets, prediction_length, object_length):
        all_trajectories = []
        heading_differences = []
        for start_lanelet, object_distance_from_start, heading_difference in start_lanelets:
            prediction_length_from_start_lanelet = prediction_length + object_distance_from_start + object_length / 2
            if "subtype" in start_lanelet.attributes and start_lanelet.attributes["subtype"] == "bus_lane":
                routing_graph = self.graph_vehicle_taxi
            else:
                routing_graph = self.graph_vehicle
            # explore following lanelets recursively
            trajectories = follow_lanelets(routing_graph, start_lanelet, prediction_length_from_start_lanelet)
            all_trajectories.extend(trajectories)
            for i in range(len(trajectories)):
                heading_differences.append(heading_difference)

        # If there are multiple trajectories that end in the same lanelet, keep the one with the smallest heading difference (better match)
        best_trajectories = {}
        for heading_difference, trajectory in zip(heading_differences, all_trajectories):
            end_lanelet = trajectory[-1]  # Get the last lanelet
            # If the end_id is not in the dictionary or the new heading difference is smaller, update the dictionary
            if end_lanelet.id not in best_trajectories or heading_difference < best_trajectories[end_lanelet.id][0]:
                best_trajectories[end_lanelet.id] = (heading_difference, trajectory)
        # Extract the filtered trajectories
        filtered_trajectories = [item[1] for item in best_trajectories.values()]

        return filtered_trajectories

    def score_paths(self, path, object_indicator):
        path_score = 0
        for i, turn in enumerate(path):
            # score the lanelet according to how well it matches the object indicator
            lanelet_score = CAR_INDICATOR_VS_TURN_DIRECTION_SCORING[object_indicator][turn]
            if i > 0:
                # discount farther lanelets
                lanelet_score /= i
            # path score is sum of lanelet scores
            path_score += lanelet_score
        return path_score

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('map_based_predictor', log_level=rospy.INFO)
    node = MapBasedPredictor()
    node.run()