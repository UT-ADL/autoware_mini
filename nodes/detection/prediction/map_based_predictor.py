#!/usr/bin/env python3

import rospy
import math
import numpy as np
from autoware_msgs.msg import DetectedObjectArray, Lane, Waypoint

from helpers.geometry import get_heading_from_vector, get_vector_norm_3d, get_heading_between_two_points, get_distance_between_two_points_2d, create_vector_from_heading_and_scalar
from helpers.lanelet2 import load_lanelet2_map
import lanelet2
from lanelet2.core import BasicPoint2d
from lanelet2.geometry import findWithin2d, findNearest
from shapely.geometry import LineString, Point as ShapelyPoint

class MapBasedPredictor:
    def __init__(self):
        # Parameters
        self.prediction_horizon = rospy.get_param('~prediction_horizon')
        self.prediction_interval = rospy.get_param('~prediction_interval')
        self.distance_from_lanelet = rospy.get_param('~distance_from_lanelet')
        self.distance_from_centerline = rospy.get_param('~distance_from_centerline')
        self.angle_threshold = rospy.get_param('~angle_threshold')

        lanelet2_map_name = rospy.get_param("/planning/lanelet2_global_planner/lanelet2_map_name")
        coordinate_transformer = rospy.get_param("/localization/coordinate_transformer")
        use_custom_origin = rospy.get_param("/localization/use_custom_origin")
        utm_origin_lat = rospy.get_param("/localization/utm_origin_lat")
        utm_origin_lon = rospy.get_param("/localization/utm_origin_lon")

        self.lanelet2_map = load_lanelet2_map(lanelet2_map_name, coordinate_transformer, use_custom_origin, utm_origin_lat, utm_origin_lon)
        # traffic rules
        traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                  lanelet2.traffic_rules.Participants.VehicleTaxi)
        # routing graph
        self.graph = lanelet2.routing.RoutingGraph(self.lanelet2_map, traffic_rules)

        # Publishers
        self.predicted_objects_pub = rospy.Publisher('predicted_objects', DetectedObjectArray, queue_size=1, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber('tracked_objects', DetectedObjectArray, self.tracked_objects_callback, queue_size=1, buff_size=2**20, tcp_nodelay=True)

    def tracked_objects_callback(self, msg):

        num_timesteps = int(self.prediction_horizon // self.prediction_interval)

        for i, obj in enumerate(msg.objects):

            # 1. SEARCH FOR SUITABLE LANELETS FOR OBJECT
            obj_location = BasicPoint2d(obj.pose.position.x, obj.pose.position.y)
            # find lanelets within distance to obj_location - distance measured from lanelet borders. Inside lanelet area this distance would be 0
            lanelets_witihn_distance = findWithin2d(self.lanelet2_map.laneletLayer, obj_location, self.distance_from_lanelet)

            angle_differences = []
            potential_lanelets = []
            for d, lanelet in lanelets_witihn_distance:

                # Skip crosswalks - don't want to snap predictions to crosswalks
                if lanelet.attributes:
                    if lanelet.attributes["subtype"] == "crosswalk":
                        continue

                # Skip lanelet if distance from centerline over limit
                linestring = LineString([(p.x, p.y) for p in lanelet.centerline])
                obj_distance_from_lanelet_start = linestring.project(ShapelyPoint(obj_location.x, obj_location.y))
                trajectory_start_point = linestring.interpolate(obj_distance_from_lanelet_start)
                obj_distance_from_centerline = get_distance_between_two_points_2d(trajectory_start_point, obj_location)
                if obj_distance_from_centerline > self.distance_from_centerline:
                    continue
                
                # Skip lanelet if angle difference between object heading and lanelet heading is over limit
                obj_heading = get_heading_from_vector(obj.velocity.linear)
                forward_point = linestring.interpolate(obj_distance_from_lanelet_start + 0.1)
                lanelet_heading = get_heading_between_two_points(trajectory_start_point, forward_point)
                angle_diff = math.degrees(abs(obj_heading - lanelet_heading))
                if angle_diff > 180:
                    angle_diff = 360 - angle_diff                

                if angle_diff > self.angle_threshold:
                    continue

                angle_differences.append(angle_diff)
                potential_lanelets.append(lanelet)

            selected_lanelet = None
            if len(angle_differences) > 0:
                min_index = np.argmin(angle_differences)
                selected_lanelet = potential_lanelets[min_index]

            # 2. CREATE MAP BASED TRAJECTORIES FOR OBJECT
            if selected_lanelet is not None:

                obj_speed = get_vector_norm_3d(obj.velocity.linear)
                obj_accel = get_vector_norm_3d(obj.acceleration.linear)

                # Predict future positions and velocities
                timesteps = np.arange(num_timesteps) * self.prediction_interval
                velocities = obj_speed + obj_accel * timesteps
                distances = np.cumsum(np.insert(velocities[:-1] * self.prediction_interval, 0, 0))

                # get all possible paths (lanelet branching) from selected lanelet to max distance
                all_trajectories = self.graph.possiblePaths(selected_lanelet, distances[-1])

                for trajectory in all_trajectories:
                    # create shapely linestring from lanelet centerlines and then use it to interpolate points in necessary distances
                    trajectory_linestring = LineString([(p.x, p.y, p.z) for lanelet in trajectory for p in lanelet.centerline])

                    lane = Lane()
                    for i, d in enumerate(distances):
                        wp = Waypoint()
                        p = trajectory_linestring.interpolate(obj_distance_from_lanelet_start + d)
                        wp.pose.pose.position.x = p.x
                        wp.pose.pose.position.y = p.y
                        wp.pose.pose.position.z = p.z
                        # TODO Recalculating velocity vector based on lanelet heading at the object location.
                        # Wrong when lanelet changes direction (turns), but good enough for now?
                        speed_x, speed_y = create_vector_from_heading_and_scalar(lanelet_heading, velocities[i])
                        wp.twist.twist.linear.x = speed_x
                        wp.twist.twist.linear.y = speed_y
                        lane.waypoints.append(wp)
                    obj.candidate_trajectories.lanes.append(lane)

            # 3. OBJECTS NOT LINKED WITH ANY LANELET - PREDICT BASED ON CURRENT VELOCITY AND ACCELERATION
            else:
                # Predict future positions and velocities
                predicted_centroids = np.empty((num_timesteps, 2), dtype=np.float32)
                predicted_velocities = np.empty((num_timesteps, 2), dtype=np.float32)

                predicted_centroids[0] = [obj.pose.position.x, obj.pose.position.y]
                predicted_velocities[0] = [obj.velocity.linear.x, obj.velocity.linear.y]
                obj_acceleration = np.array([obj.acceleration.linear.x, obj.acceleration.linear.y])

                for j in range(1, num_timesteps):
                    predicted_centroids[j] = predicted_centroids[j-1] + predicted_velocities[j-1] * self.prediction_interval
                    predicted_velocities[j] = predicted_velocities[j-1] + obj_acceleration * self.prediction_interval
                
                lane = Lane()
                for j in range(num_timesteps):
                    wp = Waypoint()
                    wp.pose.pose.position.x, wp.pose.pose.position.y = predicted_centroids[j]
                    wp.pose.pose.position.z = obj.pose.position.z
                    wp.twist.twist.linear.x, wp.twist.twist.linear.y = predicted_velocities[j]
                    lane.waypoints.append(wp)
                
                obj.candidate_trajectories.lanes.append(lane)

        # Publish predicted objects
        self.predicted_objects_pub.publish(msg)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('map_based_predictor', log_level=rospy.INFO)
    node = MapBasedPredictor()
    node.run()