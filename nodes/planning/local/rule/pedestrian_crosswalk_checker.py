#!/usr/bin/env python3

import math
import rospy
import shapely
import numpy as np
from collections import defaultdict
from autoware_mini.msg import Path, DetectedObjectArray
from sensor_msgs.msg import PointCloud2
from autoware_mini.geometry import get_speed_from_velocity, get_heading_from_vector, get_angle_between_two_headings
from autoware_mini.collision import CollisionPoints
from autoware_mini.path import PathWrapper
from autoware_mini.lanelet2 import load_lanelet2_map, get_crosswalks

class PedestrianCrosswalkChecker:

    def __init__(self):

        # parameters
        self.safety_box_width = rospy.get_param("safety_box_width")
        self.stopped_speed_limit = rospy.get_param("stopped_speed_limit")
        self.braking_safety_distance_crosswalk = rospy.get_param("~braking_safety_distance_crosswalk")
        self.crossing_angle_max_limit = rospy.get_param("~crossing_angle_max_limit")
        self.use_object_width = rospy.get_param("use_object_width")
        self.ignore_static_obstacles = rospy.get_param("~ignore_static_obstacles")
        self.crosswalk_maximum_deceleration = rospy.get_param("~crosswalk_maximum_deceleration")
        self.prediction_counter_min_limit = rospy.get_param("~prediction_counter_min_limit")
        lanelet2_map_path = rospy.get_param("~lanelet2_map_path")

        # variables
        self.objects = None
        self.crosswalks_on_global_path = None
        self.object_crosswalk_counter = defaultdict(dict)

        # load lanelet2 map
        lanelet2_map = load_lanelet2_map(lanelet2_map_path)
        self.crosswalks = self.prepare_crosswalks(get_crosswalks(lanelet2_map))

        # publishers
        self.crosswalk_collision_pub = rospy.Publisher('crosswalk_collision_points', PointCloud2, queue_size=1, tcp_nodelay=True)

        # subscribers
        rospy.Subscriber('global_path', Path, self.global_path_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('extracted_local_path', Path, self.local_path_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/detection/predicted_objects', DetectedObjectArray, self.predicted_objects_callback, queue_size=1, buff_size=2**20, tcp_nodelay=True)

    def predicted_objects_callback(self, msg):
        self.objects = msg.objects

    def global_path_callback(self, msg):
        global_path = PathWrapper(msg.waypoints)
        global_path.linestring = global_path.linestring.simplify(0.01)
        shapely.prepare(global_path.linestring)

        mask = global_path.linestring.intersects(self.crosswalks)
        self.crosswalks_on_global_path = self.crosswalks[mask]

    def local_path_callback(self, msg):
        objects = self.objects
        crosswalks_on_global_path = self.crosswalks_on_global_path
        object_crosswalk_counter = defaultdict(dict)

        if crosswalks_on_global_path is None:
            rospy.logwarn_throttle(3, "%s - global path not received!", rospy.get_name())
            return

        if objects is None:
            rospy.logwarn_throttle(3, "%s - detected objects not received!", rospy.get_name())
            return

        collision_points = CollisionPoints()
        if len(msg.waypoints) > 0 and len(crosswalks_on_global_path) > 0 and len(objects) > 0:
            local_path = PathWrapper(msg.waypoints, distances=False)
            local_path_buffer = local_path.linestring.buffer(self.safety_box_width / 2, cap_style="flat")
            shapely.prepare(local_path_buffer)

            # extract crosswalks that intersect with local path and create closest intersection point as collision point
            mask = local_path.linestring.intersects(crosswalks_on_global_path)
            crosswalks_on_local_path = crosswalks_on_global_path[mask]
            crosswalk_collision_points = []
            for crosswalk in crosswalks_on_local_path:
                intersection_coords = shapely.get_coordinates(local_path.linestring.intersection(crosswalk))
                intersection_points = shapely.points(intersection_coords)
                distances = local_path.linestring.project(intersection_points)
                min_index = np.argmin(distances)
                crosswalk_collision_points.append(intersection_points[min_index])

            if len(crosswalks_on_local_path) > 0:
                for obj in objects:
                    object_speed = get_speed_from_velocity(obj.velocity)
                    # ignore objects that are not moving
                    if self.ignore_static_obstacles and object_speed < self.stopped_speed_limit:
                        continue
                    object_position = shapely.Point(obj.centroid.x, obj.centroid.y)
                    object_distance_from_local_path_start = local_path.linestring.project(object_position)
                    # ignore objects behind the ego vehicle
                    if math.isclose(object_distance_from_local_path_start, 0.0, abs_tol=0.001):
                        continue
                    object_polygon = shapely.polygons(np.array(obj.convex_hull).reshape(-1, 3))
                    object_heading = get_heading_from_vector(obj.velocity)
                    object_to_path_heading = local_path.get_heading_towards_path(object_position)
                    object_path_approach_angle = math.degrees(get_angle_between_two_headings(object_heading, object_to_path_heading))

                    for crosswalk, collision_point in zip(crosswalks_on_local_path, crosswalk_collision_points):

                        # INTERSECTING OBJECTS
                        if crosswalk.intersects(object_polygon):
                            # objects on crosswalk approaching local path or have crossed it and departing, but still within the local path buffer
                            if not self.ignore_static_obstacles or object_path_approach_angle < self.crossing_angle_max_limit or \
                                (180 - object_path_approach_angle < self.crossing_angle_max_limit and local_path_buffer.intersects(object_polygon)):
                                collision_points.add_point(x = collision_point.x,
                                                            y = collision_point.y,
                                                            z = obj.center.z - obj.dimensions.z / 2,
                                                            vx = 0,
                                                            vy = 0,
                                                            vz = 0,
                                                            distance_to_stop = self.braking_safety_distance_crosswalk,
                                                            deceleration_limit = np.inf,
                                                            category = CollisionPoints.OBJECT_ON_CROSSWALK)

                        # NON-INTERSECTING OBJECTS - CONSIDER TRAJECTORIES
                        elif len(obj.candidate_trajectories.paths) > 0:
                            for path in obj.candidate_trajectories.paths:
                                trajectory = PathWrapper(path.waypoints)
                                trajectory_to_check = trajectory.linestring

                                if self.use_object_width:
                                    trajectory_to_check = trajectory.linestring.buffer(obj.dimensions.y / 2, cap_style="flat")
                                    shapely.prepare(trajectory_to_check)

                                if crosswalk.intersects(trajectory_to_check):
                                    intersection_coords = shapely.get_coordinates(crosswalk.intersection(trajectory_to_check))
                                    intersection_points = shapely.points(intersection_coords)
                                    distances = trajectory.linestring.project(intersection_points)
                                    min_index = np.argmin(distances)
                                    closest_distance_to_object = distances[min_index]
                                    closest_intersection_point = intersection_points[min_index]

                                    trajectory_heading_at_closest_intersection = trajectory.get_heading_at_distance(closest_distance_to_object)
                                    # find heading from the closest intersection point to its projection on local_path
                                    closest_intersection_to_path_heading = local_path.get_heading_towards_path(closest_intersection_point)
                                    closest_intersection_path_approach_angle = math.degrees(get_angle_between_two_headings(trajectory_heading_at_closest_intersection, closest_intersection_to_path_heading))

                                    if closest_intersection_path_approach_angle < self.crossing_angle_max_limit or \
                                        (180 - closest_intersection_path_approach_angle < self.crossing_angle_max_limit and local_path_buffer.intersects(trajectory_to_check)):

                                        # Add object id to the counter if present else increment the counter
                                        if obj.id not in self.object_crosswalk_counter[id(crosswalk)]:
                                            object_crosswalk_counter[id(crosswalk)][obj.id] = 1
                                        else:
                                            object_crosswalk_counter[id(crosswalk)][obj.id] = self.object_crosswalk_counter[id(crosswalk)][obj.id] + 1
                                        # add if counter is equal or above the limit
                                        if object_crosswalk_counter[id(crosswalk)][obj.id] >= self.prediction_counter_min_limit:
                                            collision_points.add_point(x = collision_point.x,
                                                                    y = collision_point.y,
                                                                    z = obj.center.z - obj.dimensions.z / 2,
                                                                    vx = 0,
                                                                    vy = 0,
                                                                    vz = 0,
                                                                    distance_to_stop = self.braking_safety_distance_crosswalk,
                                                                    deceleration_limit = self.crosswalk_maximum_deceleration,
                                                                    category = CollisionPoints.TRAJECTORY_ON_CROSSWALK)

            # update object_crosswalk_counter with current state
            self.object_crosswalk_counter = object_crosswalk_counter

        collision_points_msg = collision_points.create_message()
        collision_points_msg.header = msg.header
        self.crosswalk_collision_pub.publish(collision_points_msg)

    def prepare_crosswalks(self, crosswalks_in):
        crosswalks_out = []
        for crosswalk in crosswalks_in:
            polygon = shapely.Polygon([(p.x, p.y) for p in crosswalk.polygon2d()])
            shapely.prepare(polygon)
            crosswalks_out.append(polygon)
        return np.array(crosswalks_out)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('pedestrian_crosswalk_checker')
    node = PedestrianCrosswalkChecker()
    node.run()