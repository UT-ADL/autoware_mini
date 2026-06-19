#!/usr/bin/env python3

import math
import rospy
import shapely
import threading
import numpy as np
import concurrent.futures
import dynamic_reconfigure.server
from collections import defaultdict
from ros_numpy import msgify

from std_msgs.msg import Int32
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import PointCloud2
from autoware_mini.cfg import CollisionCheckerConfig
from autoware_mini.msg import Path, Waypoint, Log, DetectedObjectArray, StopLineStatus, StopLineStatusArray
from autoware_mini.collision import CollisionPoints, calculate_time_to_destination, create_trajectory_buffers, is_coming_from_behind
from autoware_mini.path import PathWrapper
from autoware_mini.geometry import (get_distance_between_two_points_2d, get_speed_from_velocity, get_heading_from_vector,
                                    get_angle_between_two_headings, calculate_headings)
from autoware_mini.transform import get_distance_to_car_front
from autoware_mini.lanelet2 import load_lanelet2_map, get_right_of_way_regulatory_elements, get_crosswalks
from autoware_mini.shapely import (side_of_linestring, get_boundary_points,
                                   linesubstring, calculate_linestring_heading_at_distance)

class CollisionChecker:

    def __init__(self):

        # ============================================================
        # Global parameters
        # ============================================================
        self.safety_box_width = rospy.get_param("safety_box_width")
        self.safety_box_length = rospy.get_param("safety_box_length")
        self.stopped_speed_limit = rospy.get_param("stopped_speed_limit")
        self.from_behind_heading_limit = rospy.get_param("from_behind_heading_limit")
        self.collision_point_interval = rospy.get_param("collision_point_interval")
        self.ego_vehicle_stopped_speed_limit = rospy.get_param("ego_vehicle_stopped_speed_limit")
        self.default_deceleration = rospy.get_param("default_deceleration")

        wide_safety_box_width = rospy.get_param("wide_safety_box_width")
        parallel_paths = rospy.get_param("parallel_paths")
        parallel_paths_interval = rospy.get_param("parallel_paths_interval")
        self._object_buffer_width = wide_safety_box_width / 2 + (parallel_paths // 2 + 1) * parallel_paths_interval

        # Enable flags are loaded via dynamic_reconfigure below; the
        # server picks up the matching ~enable_* rosparams as its
        # initial values, overriding the .cfg defaults.
        self.parallel_checkers = rospy.get_param("~parallel_checkers")

        # ============================================================
        # Checker-specific parameters
        # ============================================================

        # goal_stop_checker
        self._goal_braking_safety_distance = rospy.get_param("~goal_stop_checker/braking_safety_distance_goal")

        # object_collision_checker
        self._object_approaching_deceleration = rospy.get_param("~object_collision_checker/approaching_deceleration")
        self._object_shielded_deceleration = rospy.get_param("~object_collision_checker/shielded_deceleration")
        self._object_braking_safety_distance = rospy.get_param("~object_collision_checker/braking_safety_distance_obstacle")

        # traffic_light_stop_line_checker
        self._tfl_braking_safety_distance = rospy.get_param("~traffic_light_stop_line_checker/braking_safety_distance_stop_line")
        self._tfl_force_stop_speed_limit = rospy.get_param("~traffic_light_stop_line_checker/tfl_force_stop_speed_limit")
        self._tfl_deceleration_limit = rospy.get_param("~traffic_light_stop_line_checker/tfl_deceleration_limit")

        # pedestrian_crosswalk_checker
        self._crosswalk_braking_safety_distance = rospy.get_param("~pedestrian_crosswalk_checker/braking_safety_distance_crosswalk")
        self._crosswalk_crossing_angle_max_limit = rospy.get_param("~pedestrian_crosswalk_checker/crossing_angle_max_limit")
        self._crosswalk_ignore_static_obstacles = rospy.get_param("~pedestrian_crosswalk_checker/ignore_static_obstacles")
        self._crosswalk_deceleration_limit = rospy.get_param("~pedestrian_crosswalk_checker/crosswalk_deceleration_limit")
        self._crosswalk_prediction_counter_min_limit = rospy.get_param("~pedestrian_crosswalk_checker/prediction_counter_min_limit")
        self._crosswalk_wide_safety_box_width = wide_safety_box_width

        # yielding_checker
        self._yielding_braking_safety_distance = rospy.get_param("~yielding_checker/braking_safety_distance_yield")
        self._yielding_distance_limit = rospy.get_param("~yielding_checker/yielding_distance_limit")
        self._yielding_deceleration_limit = rospy.get_param("~yielding_checker/yielding_deceleration_limit")

        # give_way_checker
        self._give_way_braking_safety_distance = rospy.get_param("~give_way_checker/braking_safety_distance_give_way")
        self._give_way_deceleration_limit = rospy.get_param("~give_way_checker/give_way_deceleration_limit")
        self._give_way_counter_min_limit = rospy.get_param("~give_way_checker/give_way_counter_min_limit")
        self._give_way_check_right_turn = rospy.get_param("~give_way_checker/check_right_turn")
        self._give_way_check_left_turn = rospy.get_param("~give_way_checker/check_left_turn")

        # right_of_way_checker
        self._right_of_way_braking_safety_distance = rospy.get_param("~right_of_way_checker/braking_safety_distance_right_of_way")
        self._right_of_way_deceleration_limit = rospy.get_param("~right_of_way_checker/right_of_way_deceleration_limit")
        self._right_of_way_heading_alignment_limit = rospy.get_param("~right_of_way_checker/heading_alignment_limit")
        self._right_of_way_right_turn_check_range = rospy.get_param("~right_of_way_checker/right_turn_check_range")

        # trajectory_collision_checker
        self._traj_deceleration = rospy.get_param("~trajectory_collision_checker/trajectory_collision_deceleration")
        self._traj_braking_safety_distance = rospy.get_param("~trajectory_collision_checker/braking_safety_distance_trajectory")
        self._traj_safety_time_ego_front = rospy.get_param("~trajectory_collision_checker/safety_time_ego_front")
        self._traj_safety_time_ego_rear = rospy.get_param("~trajectory_collision_checker/safety_time_ego_rear")
        self._traj_counter_min_limit = rospy.get_param("~trajectory_collision_checker/trajectory_counter_min_limit")
        self._traj_no_time_collision_check_distance = rospy.get_param("~trajectory_collision_checker/no_time_collision_check_distance")

        # manual_yield_checker
        self._manual_yield_keep_stop_line_for = rospy.get_param("~manual_yield_checker/keep_stop_line_for")
        self._manual_yield_braking_safety_distance = rospy.get_param("~manual_yield_checker/braking_safety_distance_stop_line")

        # stop_sign_checker
        self._stop_sign_braking_safety_distance = rospy.get_param("~stop_sign_checker/braking_safety_distance_stop_sign")
        self._stop_sign_stopping_distance = rospy.get_param("~stop_sign_checker/stopping_distance")

        # lane_boundary_checker
        self._lane_boundary_braking_safety_distance = rospy.get_param("~lane_boundary_checker/braking_safety_distance_lane_boundary")
        self._lane_boundary_min_collision_point_distance = rospy.get_param("~lane_boundary_checker/min_collision_point_distance")

        # bus_stop_checker
        self._bus_stop_braking_safety_distance = rospy.get_param("~bus_stop_checker/braking_safety_distance_bus_stop")
        self._bus_stop_deceleration_limit = rospy.get_param("~bus_stop_checker/bus_stop_deceleration_limit")

        # ============================================================
        # Shared state from subscribers
        # ============================================================
        self._predicted_objects_map = None
        self._predicted_objects = None
        self._tracked_objects = None
        self._current_speed = None
        self._current_position = None
        self._regulated_stop_line_ids = []
        self._status_stop_stop_line_ids = []

        # ============================================================
        # Cached properties and locks
        # ============================================================
        self.local_path = None
        self._local_path_buffer = None
        self._local_path_buffer_lock = threading.Lock()
        self._parallel_paths_buffer = None
        self._parallel_paths_buffer_lock = threading.Lock()
        self._predicted_objects_map_data = None
        self._predicted_objects_map_data_lock = threading.Lock()
        self._predicted_objects_data = None
        self._predicted_objects_data_lock = threading.Lock()
        if self.parallel_checkers:
            self._executor = concurrent.futures.ThreadPoolExecutor()

        # ============================================================
        # Per-checker state
        # ============================================================

        # goal_stop_checker
        self._goal_point = None

        # trajectory_collision_checker
        self._traj_object_counter = defaultdict(int)

        # give_way_checker
        self._give_way_object_counter = defaultdict(int)

        # pedestrian_crosswalk_checker
        self._crosswalk_object_counter = defaultdict(int)

        # manual_yield_checker
        self._manual_yield_current_closest_stop_line_id = -1
        self._manual_yield_ignore_stop_line_id = -1
        self._manual_yield_timer = rospy.Time.now()

        # stop_sign_checker
        self._stop_sign_stopped_lines = set()

        # ============================================================
        # Load lanelet2 map data (always loaded so checkers can be
        # enabled at runtime via dynamic_reconfigure)
        # ============================================================
        lanelet2_map_path = rospy.get_param("~lanelet2_map_path")
        lanelet2_map = load_lanelet2_map(lanelet2_map_path)

        self._right_of_way_polygons, self._right_of_way_headings = get_right_of_way_regulatory_elements(lanelet2_map)
        for polygons in self._right_of_way_polygons.values():
            shapely.prepare(polygons)

        self._crosswalks, self._crosswalk_polygons = get_crosswalks(lanelet2_map, self.collision_point_interval)

        # Distance to car front (needs TF, must be after map loading)
        self._distance_to_car_front = get_distance_to_car_front()

        # ============================================================
        # Publishers
        # ============================================================
        self.collision_points_pub = rospy.Publisher('collision_points', PointCloud2, queue_size=1, tcp_nodelay=True)
        self.stop_line_status_pub = rospy.Publisher('stop_line_status', StopLineStatusArray, queue_size=5, tcp_nodelay=True)
        self.confirm_drive_pub = rospy.Publisher('confirm_drive', Int32, queue_size=1, tcp_nodelay=True)
        self.log_message_pub = rospy.Publisher('/dashboard/log_message', Log, queue_size=1, tcp_nodelay=True)

        # ============================================================
        # Dynamic reconfigure (constructed before subscribers so the
        # initial callback populates self._checkers before any message
        # can trigger _process)
        # ============================================================
        self.reconfigure_server = dynamic_reconfigure.server.Server(CollisionCheckerConfig, self.reconfigure_callback)

        # ============================================================
        # Subscribers
        # ============================================================
        rospy.Subscriber('/perception/predicted_objects_map', DetectedObjectArray, self.predicted_objects_map_callback, queue_size=1, buff_size=2**20, tcp_nodelay=True)
        rospy.Subscriber('/perception/predicted_objects', DetectedObjectArray, self.predicted_objects_callback, queue_size=1, buff_size=2**20, tcp_nodelay=True)
        rospy.Subscriber('/perception/tracked_objects', DetectedObjectArray, self.tracked_objects_callback, queue_size=1, buff_size=2**20, tcp_nodelay=True)
        rospy.Subscriber('/perception/traffic_light_status', StopLineStatusArray, self.traffic_light_status_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/localization/current_velocity', TwistStamped, self.current_velocity_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('extracted_local_path', Path, self.local_path_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('global_path', Path, self.global_path_callback, queue_size=None, tcp_nodelay=True)
        rospy.Subscriber('confirm_drive', Int32, self.confirm_drive_callback, queue_size=1, tcp_nodelay=True)
        rospy.Service('service_confirm_drive', Empty, self.confirm_drive_handler)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def reconfigure_callback(self, config, level):
        checkers = []
        if config.enable_goal_checker:          checkers.append(self.check_goal_stop)
        if config.enable_object_checker:        checkers.append(self.check_object_collision)
        if config.enable_traffic_light_checker: checkers.append(self.check_traffic_light_stop_line)
        if config.enable_crosswalk_checker:     checkers.append(self.check_pedestrian_crosswalk)
        if config.enable_yielding_checker:      checkers.append(self.check_yielding)
        if config.enable_give_way_checker:      checkers.append(self.check_give_way)
        if config.enable_bus_stop_checker:      checkers.append(self.check_bus_stop)
        if config.enable_trajectory_checker:    checkers.append(self.check_trajectory_collision)
        if config.enable_right_of_way_checker:  checkers.append(self.check_right_of_way)
        if config.enable_manual_yield_checker:  checkers.append(self.check_manual_yield)
        if config.enable_stop_sign_checker:     checkers.append(self.check_stop_sign)
        if config.enable_lane_boundary_checker: checkers.append(self.check_lane_boundary)
        # Atomic swap; _process reads self._checkers without a lock.
        self._checkers = checkers
        return config

    # ================================================================
    # Subscriber callbacks
    # ================================================================

    def predicted_objects_map_callback(self, msg):
        with self._predicted_objects_map_data_lock:
            self._predicted_objects_map = msg.objects
            self._predicted_objects_map_data = None

    def predicted_objects_callback(self, msg):
        with self._predicted_objects_data_lock:
            self._predicted_objects = msg.objects
            self._predicted_objects_data = None

    def tracked_objects_callback(self, msg):
        self._tracked_objects = msg.objects

    def traffic_light_status_callback(self, msg):
        self._regulated_stop_line_ids = [s.stop_line_id for s in msg.statuses if s.status in (StopLineStatus.STATUS_GO, StopLineStatus.STATUS_STOP)]
        self._status_stop_stop_line_ids = [s.stop_line_id for s in msg.statuses if s.status == StopLineStatus.STATUS_STOP]

    def current_velocity_callback(self, msg):
        self._current_speed = msg.twist.linear.x

    def current_pose_callback(self, msg):
        self._current_position = msg.pose.position

    # ================================================================
    # Cached properties
    # ================================================================

    def local_path_buffer(self):
        with self._local_path_buffer_lock:
            if self._local_path_buffer is None:
                self._local_path_buffer = self.local_path.linestring.buffer(self.safety_box_width / 2, cap_style="flat")
                shapely.prepare(self._local_path_buffer)
            return self._local_path_buffer

    def parallel_paths_buffer(self):
        with self._parallel_paths_buffer_lock:
            if self._parallel_paths_buffer is None:
                self._parallel_paths_buffer = self.local_path.linestring.buffer(self._object_buffer_width, cap_style="flat")
                shapely.prepare(self._parallel_paths_buffer)
            return self._parallel_paths_buffer

    def predicted_objects_map_data(self):
        with self._predicted_objects_map_data_lock:
            if self._predicted_objects_map_data is None:
                predicted_objects = self._predicted_objects_map
                if predicted_objects is not None:
                    trajectory_data = create_trajectory_buffers(predicted_objects)
                else:
                    trajectory_data = (np.array([], dtype=int), np.array([]), np.array([]))
                self._predicted_objects_map_data = (predicted_objects, *trajectory_data)
            return self._predicted_objects_map_data

    def predicted_objects_data(self):
        with self._predicted_objects_data_lock:
            if self._predicted_objects_data is None:
                predicted_objects = self._predicted_objects
                if predicted_objects is not None:
                    trajectory_data = create_trajectory_buffers(predicted_objects)
                else:
                    trajectory_data = (np.array([], dtype=int), np.array([]), np.array([]))
                self._predicted_objects_data = (predicted_objects, *trajectory_data)
            return self._predicted_objects_data

    # ================================================================
    # Main callback
    # ================================================================

    def local_path_callback(self, msg):
        if not msg.waypoints:
            collision_points_msg = msgify(PointCloud2, CollisionPoints.EMPTY)
            collision_points_msg.header = msg.header
            self.collision_points_pub.publish(collision_points_msg)
            return

        # Pre-compute local path and invalidate per-cycle caches
        self.local_path = PathWrapper(msg.waypoints)
        self._local_path_buffer = None
        self._parallel_paths_buffer = None

        # Snapshot the checker list once so a concurrent reconfigure_callback
        # swap doesn't change the set mid-cycle.
        checkers = self._checkers

        # Run checkers in parallel or sequentially
        all_points = [CollisionPoints.EMPTY]
        if self.parallel_checkers:
            for result in self._executor.map(lambda f: f(), checkers):
                all_points.extend(result)
        else:
            for checker in checkers:
                all_points.extend(checker())

        # Publish merged collision points
        merged = np.concatenate(all_points)
        collision_points_msg = msgify(PointCloud2, merged)
        collision_points_msg.header = msg.header
        self.collision_points_pub.publish(collision_points_msg)

    # ================================================================
    # Checker: goal_stop
    # ================================================================

    def global_path_callback(self, msg):
        if msg.waypoints:
            self._goal_point = msg.waypoints[-1].position
        else:
            self._goal_point = None

    def check_goal_stop(self):
        collision_points = CollisionPoints()
        goal_point = self._goal_point

        if goal_point is not None:
            if math.isclose(get_distance_between_two_points_2d(goal_point, self.local_path.waypoints[-2].position), 0.0):
                collision_points.add_point(x=goal_point.x, y=goal_point.y, z=goal_point.z,
                                           distance_to_stop=self._goal_braking_safety_distance,
                                           category=CollisionPoints.GOAL_POINT,
                                           deceleration=self.default_deceleration)

        return collision_points.points

    # ================================================================
    # Checker: object_collision
    # ================================================================

    def check_object_collision(self):
        tracked_objects = self._tracked_objects
        collision_points = CollisionPoints()

        if tracked_objects is None:
            rospy.logwarn_throttle(3, "%s - tracked objects not received!", rospy.get_name())
            return collision_points.points

        if tracked_objects:
            local_path_coords = shapely.get_coordinates(self.local_path.linestring)

            # First pass: classify objects and find closest moving in-lane object distance
            objects_on_path = []
            closest_moving_in_lane_distance = math.inf

            for obj in tracked_objects:
                object_polygon = shapely.polygons(np.array(obj.convex_hull).reshape(-1, 3))

                if self.parallel_paths_buffer().intersects(object_polygon):
                    in_lane = self.local_path_buffer().intersects(object_polygon)
                    obj_point = shapely.Point(obj.center.x, obj.center.y)
                    obj_distance = self.local_path.linestring.project(obj_point)
                    heading = calculate_headings(local_path_coords, obj_distance)
                    projected_speed = obj.velocity.x * math.cos(heading) + obj.velocity.y * math.sin(heading)
                    if projected_speed > self.stopped_speed_limit:
                        category = CollisionPoints.MOVING_OBJECT_ON_PATH
                        deceleration = self.default_deceleration
                        if in_lane:
                            closest_moving_in_lane_distance = min(closest_moving_in_lane_distance, obj_distance)
                    elif projected_speed < -self.stopped_speed_limit:
                        category = CollisionPoints.APPROACHING_OBJECT_ON_PATH
                        deceleration = self._object_approaching_deceleration
                    else:
                        category = CollisionPoints.OBJECT_ON_PATH
                        deceleration = self.default_deceleration

                    objects_on_path.append((obj, object_polygon, obj_distance, category, deceleration, in_lane))

            # Second pass: for in-lane objects behind the closest moving in-lane object, use shielded deceleration
            for obj, object_polygon, obj_distance, category, deceleration, in_lane in objects_on_path:
                if in_lane and obj_distance > closest_moving_in_lane_distance:
                    deceleration = self._object_shielded_deceleration

                cluster_points = get_boundary_points(object_polygon, self.collision_point_interval)
                collision_points.add_points(
                    cluster_points,
                    z=obj.center.z - obj.dimensions.z / 2,
                    vx=obj.velocity.x, vy=obj.velocity.y, vz=obj.velocity.z,
                    distance_to_stop=self._object_braking_safety_distance,
                    category=category, deceleration=deceleration)

        return collision_points.points

    # ================================================================
    # Checker: traffic_light_stop_line
    # ================================================================

    def check_traffic_light_stop_line(self):
        current_speed = self._current_speed
        status_stop_stop_line_ids = self._status_stop_stop_line_ids
        collision_points = CollisionPoints()

        if current_speed is None:
            rospy.logwarn_throttle(3, "%s - current velocity not received!", rospy.get_name())
            return collision_points.points

        if status_stop_stop_line_ids:
            local_path = self.local_path
            mask = np.isin(local_path.stop_line_ids, status_stop_stop_line_ids)
            waypoint_indices = np.where(mask)[0]

            for wp_idx in waypoint_indices:
                wp = local_path.waypoints[wp_idx]
                collision_points.add_stop_line(wp,
                    distance_to_stop=self._tfl_braking_safety_distance,
                    category=CollisionPoints.TRAFFIC_LIGHT_STOP_LINE,
                    deceleration=self.default_deceleration,
                    deceleration_limit=np.inf if current_speed < (self._tfl_force_stop_speed_limit / 3.6) else self._tfl_deceleration_limit)

        return collision_points.points

    # ================================================================
    # Checker: trajectory_collision
    # ================================================================

    def check_trajectory_collision(self):
        predicted_objects, obj_indices, trajectory_linestrings, trajectory_buffers = self.predicted_objects_map_data()
        current_speed = self._current_speed
        object_counter = defaultdict(int)
        collision_points = CollisionPoints()

        if predicted_objects is None:
            rospy.logwarn_throttle(3, "%s - predicted objects not received!", rospy.get_name())
            return collision_points.points

        if current_speed is None:
            rospy.logwarn_throttle(3, "%s - current velocity not received!", rospy.get_name())
            return collision_points.points

        if trajectory_linestrings.size > 0:
            local_path = self.local_path
            local_path_buffer = self.parallel_paths_buffer()
            intersection_mask = local_path_buffer.intersects(trajectory_buffers)

            for i in np.where(intersection_mask)[0]:
                obj = predicted_objects[obj_indices[i]]
                trajectory_linestring = trajectory_linestrings[i]
                trajectory_buffer = trajectory_buffers[i]

                if is_coming_from_behind(trajectory_linestring, obj.dimensions.y, local_path.linestring, self.safety_box_width, self.from_behind_heading_limit):
                    continue
                if is_coming_from_behind(local_path.linestring, self.safety_box_width, trajectory_linestring, obj.dimensions.y, self.from_behind_heading_limit):
                    continue

                trajectory_colliding = False

                trajectory_intersection_result = local_path_buffer.intersection(trajectory_buffer)
                trajectory_intersection_densified = shapely.segmentize(trajectory_intersection_result, max_segment_length=self.collision_point_interval)
                trajectory_intersection_coords = shapely.get_coordinates(trajectory_intersection_densified)
                collision_area_points = shapely.points(trajectory_intersection_coords)
                collision_area_distances = local_path.linestring.project(collision_area_points)

                collision_distance_from_ego_front = collision_area_distances - self._distance_to_car_front
                if current_speed <= self.stopped_speed_limit:
                    if min(collision_distance_from_ego_front) < self._traj_no_time_collision_check_distance:
                        trajectory_colliding = True
                else:
                    ego_arrival_times = calculate_time_to_destination(current_speed, 0, collision_distance_from_ego_front)
                    ego_leaving_times = calculate_time_to_destination(current_speed, 0, collision_distance_from_ego_front + self.safety_box_length)
                    ego_arrival_times -= self._traj_safety_time_ego_front
                    ego_leaving_times += self._traj_safety_time_ego_rear

                    obj_speed = get_speed_from_velocity(obj.velocity)
                    obj_accel = get_speed_from_velocity(obj.acceleration)
                    collision_distance_from_obj_front = trajectory_linestring.project(collision_area_points)
                    obj_arrival_times = calculate_time_to_destination(obj_speed, obj_accel, collision_distance_from_obj_front)
                    obj_leaving_times = calculate_time_to_destination(obj_speed, obj_accel, collision_distance_from_obj_front + obj.dimensions.x)

                    collision_mask = (ego_arrival_times <= obj_leaving_times) & (ego_leaving_times >= obj_arrival_times)
                    trajectory_intersection_coords = trajectory_intersection_coords[collision_mask]
                    collision_distance_from_ego_front = collision_distance_from_ego_front[collision_mask]

                    if trajectory_intersection_coords.size > 0:
                        count = self._traj_object_counter[obj.id] + 1
                        object_counter[obj.id] = count
                        if count > self._traj_counter_min_limit:
                            trajectory_colliding = True

                if trajectory_colliding:
                    collision_points.add_points(
                        trajectory_intersection_coords,
                        z=obj.center.z - obj.dimensions.z / 2,
                        distance_to_stop=self._traj_braking_safety_distance,
                        category=CollisionPoints.COLLIDING_TRAJECTORY,
                        deceleration=self._traj_deceleration)

        self._traj_object_counter = object_counter

        return collision_points.points

    # ================================================================
    # Checker: right_of_way
    # ================================================================

    def check_right_of_way(self):
        predicted_objects, obj_indices, trajectory_linestrings, trajectory_buffers = self.predicted_objects_map_data()
        collision_points = CollisionPoints()
        stop_line_statuses = StopLineStatusArray()
        stop_line_statuses.type = StopLineStatusArray.RIGHT_OF_WAY

        if predicted_objects is None:
            rospy.logwarn_throttle(3, "%s - predicted objects not received!", rospy.get_name())
            return collision_points.points

        regulated_stop_line_ids = self._regulated_stop_line_ids
        local_path = self.local_path

        stop_line_distances, stop_line_ids, stop_line_types, stop_line_wp_indices = local_path.get_stop_lines(
            [Waypoint.STOP_LINE_YIELD, Waypoint.STOP_LINE_YIELD_STOP, Waypoint.STOP_LINE_YIELD_MANUAL, Waypoint.STOP_LINE_YIELD_RIGHT],
            exclude_ids=regulated_stop_line_ids)

        for stop_line_id, stop_line_distance, stop_line_type, wp_idx in zip(stop_line_ids, stop_line_distances, stop_line_types, stop_line_wp_indices):
            if stop_line_id not in self._right_of_way_polygons:
                continue

            if stop_line_type == Waypoint.STOP_LINE_YIELD_RIGHT:
                has_right_turn = local_path.check_turn_signal_in_range(stop_line_distance, stop_line_distance + self._right_of_way_right_turn_check_range, Waypoint.TURN_RIGHT)
                if has_right_turn:
                    continue

            intersection_found = False

            for obj_idx, obj in enumerate(predicted_objects):
                traj_indices = np.where(obj_indices == obj_idx)[0]
                for i in traj_indices:
                    trajectory_buffer = trajectory_buffers[i]
                    trajectory_linestring = trajectory_linestrings[i]

                    trajectory_intersection_mask = shapely.intersects(self._right_of_way_polygons[stop_line_id], trajectory_buffer)

                    if trajectory_intersection_mask.any():
                        for heading, polygon in zip(self._right_of_way_headings[stop_line_id][trajectory_intersection_mask],
                                                     self._right_of_way_polygons[stop_line_id][trajectory_intersection_mask]):
                            trajectory_intersection_result = polygon.intersection(trajectory_buffer)
                            trajectory_intersection_coords = shapely.get_coordinates(trajectory_intersection_result)
                            intersection_points = shapely.points(trajectory_intersection_coords)
                            intersection_distance = min(trajectory_linestring.project(intersection_points))
                            trajectory_heading_at_intersection = calculate_linestring_heading_at_distance(trajectory_linestring, intersection_distance)
                            heading_difference = math.degrees(get_angle_between_two_headings(trajectory_heading_at_intersection, heading))
                            if heading_difference < self._right_of_way_heading_alignment_limit:
                                intersection_found = True
                                break
                        if intersection_found:
                            break
                if intersection_found:
                    break

                object_polygon = shapely.polygons(np.array(obj.convex_hull).reshape(-1, 3))
                object_intersection_mask = shapely.intersects(self._right_of_way_polygons[stop_line_id], object_polygon)
                if object_intersection_mask.any():
                    object_speed = get_speed_from_velocity(obj.velocity)
                    headings = self._right_of_way_headings[stop_line_id][object_intersection_mask]
                    obj_headings = np.full(headings.shape, obj.heading)
                    heading_differences = np.degrees(get_angle_between_two_headings(obj_headings, headings))
                    if object_speed <= self.stopped_speed_limit or np.any(heading_differences < self._right_of_way_heading_alignment_limit):
                        intersection_found = True
                        break

            if intersection_found:
                wp = local_path.waypoints[wp_idx]
                collision_points.add_stop_line(wp,
                    distance_to_stop=self._right_of_way_braking_safety_distance,
                    category=CollisionPoints.RIGHT_OF_WAY,
                    deceleration=self.default_deceleration,
                    deceleration_limit=self._right_of_way_deceleration_limit)

            stop_line_status = StopLineStatus()
            stop_line_status.stop_line_id = stop_line_id
            stop_line_status.status = StopLineStatus.STATUS_STOP if intersection_found else StopLineStatus.STATUS_GO
            stop_line_statuses.statuses.append(stop_line_status)

        self.stop_line_status_pub.publish(stop_line_statuses)
        return collision_points.points

    # ================================================================
    # Checker: yielding
    # ================================================================

    def check_yielding(self):
        predicted_objects, obj_indices, trajectory_linestrings, trajectory_buffers = self.predicted_objects_map_data()
        collision_points = CollisionPoints()
        stop_line_statuses = StopLineStatusArray()
        stop_line_statuses.type = StopLineStatusArray.YIELD

        if predicted_objects is None:
            rospy.logwarn_throttle(3, "%s - predicted objects not received!", rospy.get_name())
            return collision_points.points

        regulated_stop_line_ids = self._regulated_stop_line_ids

        local_path = self.local_path

        stop_line_distances, stop_line_ids, _, stop_line_wp_indices = local_path.get_stop_lines(
            [Waypoint.STOP_LINE_YIELD, Waypoint.STOP_LINE_YIELD_STOP, Waypoint.STOP_LINE_YIELD_MANUAL],
            exclude_ids=regulated_stop_line_ids)

        if stop_line_ids.size > 0:
            # compute yielding area end distances: from each stop line to the next, max yielding_distance_limit
            yielding_area_end_distances = np.minimum(
                stop_line_distances + self._yielding_distance_limit,
                np.append(stop_line_distances[1:], np.inf)
            )

            for stop_line_id, start_dist, end_dist, stop_line_wp_idx in zip(stop_line_ids, stop_line_distances, yielding_area_end_distances, stop_line_wp_indices):
                yielding_found = False
                yielding_area_linestring = linesubstring(local_path.linestring, start_dist, end_dist)

                if trajectory_linestrings.size > 0:
                    yielding_area_buffer = yielding_area_linestring.buffer(self.safety_box_width / 2, cap_style="flat")
                    shapely.prepare(yielding_area_buffer)
                    intersection_mask = yielding_area_buffer.intersects(trajectory_buffers)

                    for i in np.where(intersection_mask)[0]:
                        obj = predicted_objects[obj_indices[i]]

                        # ignore objects coming from behind ego (object trajectory ends behind the yielding area, same direction)
                        if is_coming_from_behind(trajectory_linestrings[i], obj.dimensions.y, local_path.linestring, self.safety_box_width, self.from_behind_heading_limit):
                            continue
                        # ignore objects ego is following (object trajectory starts behind ego and runs in the same direction)
                        if is_coming_from_behind(local_path.linestring, self.safety_box_width, trajectory_linestrings[i], obj.dimensions.y, self.from_behind_heading_limit):
                            continue

                        wp = local_path.waypoints[stop_line_wp_idx]
                        collision_points.add_stop_line(wp,
                            distance_to_stop=self._yielding_braking_safety_distance,
                            category=CollisionPoints.YIELDING_TRAJECTORY,
                            deceleration=self.default_deceleration,
                            deceleration_limit=self._yielding_deceleration_limit)

                        yielding_found = True
                        break

                stop_line_status = StopLineStatus()
                stop_line_status.stop_line_id = stop_line_id
                stop_line_status.status = StopLineStatus.STATUS_STOP if yielding_found else StopLineStatus.STATUS_GO
                stop_line_statuses.statuses.append(stop_line_status)

        self.stop_line_status_pub.publish(stop_line_statuses)
        return collision_points.points

    # ================================================================
    # Checker: give_way
    # ================================================================

    def check_give_way(self):
        predicted_objects, obj_indices, trajectory_linestrings, trajectory_buffers = self.predicted_objects_map_data()
        object_counter = defaultdict(int)
        collision_points = CollisionPoints()

        if predicted_objects is None:
            rospy.logwarn_throttle(3, "%s - predicted objects not received!", rospy.get_name())
            return collision_points.points

        if trajectory_linestrings.size > 0:
            local_path = self.local_path
            local_path_buffer = self.local_path_buffer()
            intersection_mask = local_path_buffer.intersects(trajectory_buffers)

            for i in np.where(intersection_mask)[0]:
                obj = predicted_objects[obj_indices[i]]
                trajectory_linestring = trajectory_linestrings[i]
                trajectory_buffer = trajectory_buffers[i]

                if is_coming_from_behind(trajectory_linestring, obj.dimensions.y, local_path.linestring, self.safety_box_width, self.from_behind_heading_limit):
                    continue

                if is_coming_from_behind(local_path.linestring, self.safety_box_width, trajectory_linestring, obj.dimensions.y, self.from_behind_heading_limit):
                    continue

                trajectory_intersection_result = local_path_buffer.intersection(trajectory_buffer)
                trajectory_intersection_coords = shapely.get_coordinates(trajectory_intersection_result)
                trajectory_intersection_points = shapely.points(trajectory_intersection_coords)

                distances = local_path.linestring.project(trajectory_intersection_points)
                intersection_distance = min(distances)
                if local_path.get_priority_at_distance(intersection_distance):
                    continue

                obj_center = shapely.Point(obj.center.x, obj.center.y, obj.center.z)
                from_right = side_of_linestring(local_path.linestring, obj_center) > 0
                turn_signal = local_path.get_turn_signal_at_distance(intersection_distance)

                if (from_right and ((self._give_way_check_right_turn and turn_signal == Waypoint.TURN_RIGHT) or (turn_signal == Waypoint.TURN_STRAIGHT))) or (
                    self._give_way_check_left_turn and turn_signal == Waypoint.TURN_LEFT):
                    count = self._give_way_object_counter[obj.id]
                    object_counter[obj.id] = count + 1
                    if count >= self._give_way_counter_min_limit:
                        waypoint_index = local_path.get_waypoint_index_at_distance(intersection_distance) - 1
                        wp = local_path.waypoints[waypoint_index]

                        collision_points.add_stop_line(wp,
                            distance_to_stop=self._give_way_braking_safety_distance,
                            category=CollisionPoints.GIVE_WAY,
                            deceleration=self.default_deceleration,
                            deceleration_limit=self._give_way_deceleration_limit)

        self._give_way_object_counter = object_counter

        return collision_points.points

    # ================================================================
    # Checker: pedestrian_crosswalk
    # ================================================================

    def check_pedestrian_crosswalk(self):
        predicted_objects, obj_indices, trajectory_linestrings, trajectory_buffers = self.predicted_objects_data()
        current_object_crosswalk_counter = defaultdict(int)
        collision_points = CollisionPoints()

        if predicted_objects is None:
            rospy.logwarn_throttle(3, "%s - predicted objects not received!", rospy.get_name())
            return collision_points.points

        if self._crosswalk_polygons.size > 0 and predicted_objects:
            local_path = self.local_path

            trimmed_local_path = linesubstring(local_path.linestring, self._distance_to_car_front, local_path.linestring.length)
            shapely.prepare(trimmed_local_path)
            mask = trimmed_local_path.intersects(self._crosswalk_polygons)
            crosswalks_on_local_path = self._crosswalks[mask]

            if crosswalks_on_local_path.size > 0:

                blocked_crosswalk_indices = set()
                for obj_idx, obj in enumerate(predicted_objects):
                    if self._crosswalk_ignore_static_obstacles and not obj.candidate_trajectories.paths:
                        continue

                    object_position = shapely.Point(obj.centroid.x, obj.centroid.y)
                    object_distance_from_local_path_start = local_path.linestring.project(object_position)

                    if math.isclose(object_distance_from_local_path_start, 0.0, abs_tol=0.001):
                        continue

                    object_polygon = shapely.polygons(np.array(obj.convex_hull).reshape(-1, 3))
                    object_to_path_heading = local_path.get_heading_towards_path(object_position)
                    object_path_approach_angle = math.degrees(get_angle_between_two_headings(obj.heading, object_to_path_heading))

                    traj_indices = np.where(obj_indices == obj_idx)[0]

                    for crosswalk_idx, crosswalk in enumerate(crosswalks_on_local_path):
                        if crosswalk_idx in blocked_crosswalk_indices:
                            continue

                        crosswalk_polygon = crosswalk['polygon']

                        # OBJECT ON CROSSWALK
                        if crosswalk_polygon.intersects(object_polygon):
                            if not self._crosswalk_ignore_static_obstacles or self._crosswalk_is_approaching_or_departing(object_path_approach_angle, local_path, object_polygon):
                                collision_points.add_points(
                                    self._crosswalk_get_nearest_boundary_coords(crosswalk, local_path),
                                    z=obj.center.z - obj.dimensions.z / 2,
                                    distance_to_stop=self._crosswalk_braking_safety_distance,
                                    category=CollisionPoints.OBJECT_ON_CROSSWALK,
                                    deceleration=self.default_deceleration)
                                blocked_crosswalk_indices.add(crosswalk_idx)

                        # TRAJECTORY ALIGNING WITH CROSSWALK
                        elif traj_indices.size > 0:
                            crosswalk_traj_mask = crosswalk_polygon.intersects(trajectory_buffers[traj_indices])

                            for j in traj_indices[crosswalk_traj_mask]:
                                trajectory_linestring = trajectory_linestrings[j]
                                trajectory_buffer = trajectory_buffers[j]

                                trajectory_crosswalk_intersection = crosswalk_polygon.intersection(trajectory_buffer)
                                intersection_coords = shapely.get_coordinates(trajectory_crosswalk_intersection)
                                intersection_points = shapely.points(intersection_coords)

                                object_to_intersection_distances = trajectory_linestring.project(intersection_points)
                                min_index = np.argmin(object_to_intersection_distances)

                                trajectory_heading = calculate_linestring_heading_at_distance(trajectory_linestring, object_to_intersection_distances[min_index])
                                trajectory_to_path_heading = local_path.get_heading_towards_path(intersection_points[min_index])
                                trajectory_approach_angle = math.degrees(get_angle_between_two_headings(trajectory_heading, trajectory_to_path_heading))

                                if self._crosswalk_is_approaching_or_departing(trajectory_approach_angle, local_path, trajectory_buffer):
                                    crosswalk_id = (crosswalk_polygon.centroid.x, crosswalk_polygon.centroid.y)
                                    key = (crosswalk_id, obj.id)
                                    current_object_crosswalk_counter[key] = self._crosswalk_object_counter[key] + 1

                                    if current_object_crosswalk_counter[key] >= self._crosswalk_prediction_counter_min_limit:
                                        collision_points.add_points(
                                            self._crosswalk_get_nearest_boundary_coords(crosswalk, local_path),
                                            z=obj.center.z - obj.dimensions.z / 2,
                                            distance_to_stop=self._crosswalk_braking_safety_distance,
                                            category=CollisionPoints.TRAJECTORY_ON_CROSSWALK,
                                            deceleration=self.default_deceleration,
                                            deceleration_limit=self._crosswalk_deceleration_limit)
                                        blocked_crosswalk_indices.add(crosswalk_idx)

            self._crosswalk_object_counter = current_object_crosswalk_counter

        return collision_points.points

    def _crosswalk_get_nearest_boundary_coords(self, crosswalk, local_path):
        left_distance = local_path.linestring.project(crosswalk['left_centroid'])
        right_distance = local_path.linestring.project(crosswalk['right_centroid'])
        if left_distance < right_distance:
            return crosswalk['left_coords']
        return crosswalk['right_coords']

    def _crosswalk_is_approaching_or_departing(self, approach_angle, local_path, polygon):
        return approach_angle < self._crosswalk_crossing_angle_max_limit or \
               (180 - approach_angle < self._crosswalk_crossing_angle_max_limit and
                shapely.dwithin(local_path.linestring, polygon, self._crosswalk_wide_safety_box_width / 2))

    # ================================================================
    # Checker: bus_stop
    # ================================================================

    def check_bus_stop(self):
        predicted_objects, obj_indices, trajectory_linestrings, trajectory_buffers = self.predicted_objects_map_data()
        collision_points = CollisionPoints()

        if predicted_objects is None:
            rospy.logwarn_throttle(3, "%s - predicted objects not received!", rospy.get_name())
            return collision_points.points

        if trajectory_linestrings.size > 0:
            # filter to bus objects using shared trajectory data
            bus_mask = np.array([predicted_objects[idx].label == "bus" for idx in obj_indices])
            if bus_mask.any():
                bus_obj_indices = obj_indices[bus_mask]
                bus_trajectory_linestrings = trajectory_linestrings[bus_mask]
                bus_trajectory_buffers = trajectory_buffers[bus_mask]

                local_path = self.local_path
                local_path_buffer = self.local_path_buffer()
                intersection_mask = local_path_buffer.intersects(bus_trajectory_buffers)

                for i in np.where(intersection_mask)[0]:
                    obj = predicted_objects[bus_obj_indices[i]]
                    trajectory_linestring = bus_trajectory_linestrings[i]

                    if is_coming_from_behind(trajectory_linestring, obj.dimensions.y, local_path.linestring, self.safety_box_width, self.from_behind_heading_limit):
                        continue

                    object_polygon = shapely.polygons(np.array(obj.convex_hull).reshape(-1, 3))
                    object_polygon_points = shapely.get_coordinates(object_polygon)[:-1]
                    collision_points.add_points(
                        object_polygon_points,
                        z=obj.center.z - obj.dimensions.z / 2,
                        vx=obj.velocity.x, vy=obj.velocity.y, vz=obj.velocity.z,
                        distance_to_stop=self._bus_stop_braking_safety_distance,
                        category=CollisionPoints.GIVE_WAY_BUS,
                        deceleration=self.default_deceleration,
                        deceleration_limit=self._bus_stop_deceleration_limit)

        return collision_points.points

    # ================================================================
    # Checker: manual_yield
    # ================================================================

    def confirm_drive_callback(self, msg):
        if msg.data == -1:
            return
        self._manual_yield_timer = rospy.Time.now()
        self._manual_yield_ignore_stop_line_id = self._manual_yield_current_closest_stop_line_id
        self.log_message_pub.publish(Log(message="Allow crossing the yield line", color="white", instant=True))
        rospy.loginfo("Removed forced stop for stop line id %d for %d seconds",
                      self._manual_yield_ignore_stop_line_id, self._manual_yield_keep_stop_line_for)

    def confirm_drive_handler(self, msg):
        self.confirm_drive_pub.publish(Int32(self._manual_yield_current_closest_stop_line_id))
        return EmptyResponse()

    def check_manual_yield(self):
        regulated_stop_line_ids = self._regulated_stop_line_ids
        collision_points = CollisionPoints()
        stop_line_statuses = StopLineStatusArray()
        stop_line_statuses.type = StopLineStatusArray.YIELD_MANUAL

        local_path = self.local_path

        _, stop_line_ids, _, stop_line_wp_indices = local_path.get_stop_lines(
            [Waypoint.STOP_LINE_YIELD_MANUAL], exclude_ids=regulated_stop_line_ids)

        closest_stop_line_id = -1

        if stop_line_ids.size > 0:
            closest_stop_line_id = stop_line_ids[0]

            for stop_line_id, wp_idx in zip(stop_line_ids, stop_line_wp_indices):
                if stop_line_id != self._manual_yield_ignore_stop_line_id:
                    wp = local_path.waypoints[wp_idx]
                    collision_points.add_stop_line(wp,
                        distance_to_stop=self._manual_yield_braking_safety_distance,
                        category=CollisionPoints.STOP_LINE_FORCED_STOP,
                        deceleration=self.default_deceleration)

                status = StopLineStatus()
                status.stop_line_id = stop_line_id
                status.status = StopLineStatus.STATUS_STOP if stop_line_id != self._manual_yield_ignore_stop_line_id else StopLineStatus.STATUS_GO
                stop_line_statuses.statuses.append(status)

        self._manual_yield_current_closest_stop_line_id = closest_stop_line_id

        if self._manual_yield_ignore_stop_line_id != -1 and (
            self._manual_yield_current_closest_stop_line_id != self._manual_yield_ignore_stop_line_id or
            self._manual_yield_timer + rospy.Duration(self._manual_yield_keep_stop_line_for) < rospy.Time.now()):
            self._manual_yield_ignore_stop_line_id = -1
            self.confirm_drive_pub.publish(Int32(self._manual_yield_ignore_stop_line_id))

        self.stop_line_status_pub.publish(stop_line_statuses)
        return collision_points.points

    # ================================================================
    # Checker: stop_sign
    # ================================================================

    def check_stop_sign(self):
        current_position = self._current_position
        current_speed = self._current_speed
        regulated_stop_line_ids = self._regulated_stop_line_ids
        collision_points = CollisionPoints()
        stop_line_statuses = StopLineStatusArray()
        stop_line_statuses.type = StopLineStatusArray.YIELD_STOP

        if current_position is None:
            rospy.logwarn_throttle(3, "%s - current pose not received!", rospy.get_name())
            return collision_points.points

        if current_speed is None:
            rospy.logwarn_throttle(3, "%s - current velocity not received!", rospy.get_name())
            return collision_points.points

        local_path = self.local_path

        stop_line_distances, stop_line_ids, _, stop_line_wp_indices = local_path.get_stop_lines(
            [Waypoint.STOP_LINE_YIELD_STOP], exclude_ids=regulated_stop_line_ids)

        if stop_line_ids.size > 0:
            current_position = shapely.Point(current_position.x, current_position.y, current_position.z)
            ego_distance_from_local_path_start = local_path.linestring.project(current_position)
            ego_front_from_local_path_start = ego_distance_from_local_path_start + self._distance_to_car_front

            distances_from_ego = stop_line_distances - ego_front_from_local_path_start

            for stop_line_id, stop_line_distance, wp_idx in zip(stop_line_ids, distances_from_ego, stop_line_wp_indices):
                if stop_line_distance > self._stop_sign_stopping_distance:
                    if stop_line_id in self._stop_sign_stopped_lines:
                        stop_status = StopLineStatus.STATUS_GO
                    elif current_speed < self.ego_vehicle_stopped_speed_limit and stop_line_distance < self.safety_box_length / 2:
                        self._stop_sign_stopped_lines.add(stop_line_id)
                        stop_status = StopLineStatus.STATUS_GO
                    else:
                        wp = local_path.waypoints[wp_idx]
                        collision_points.add_stop_line(wp,
                            distance_to_stop=self._stop_sign_braking_safety_distance,
                            category=CollisionPoints.STOP_SIGN_STOP,
                            deceleration=self.default_deceleration)
                        stop_status = StopLineStatus.STATUS_STOP
                else:
                    stop_status = StopLineStatus.STATUS_GO

                status = StopLineStatus()
                status.stop_line_id = stop_line_id
                status.status = stop_status
                stop_line_statuses.statuses.append(status)

        self._stop_sign_stopped_lines &= set(stop_line_ids)

        self.stop_line_status_pub.publish(stop_line_statuses)
        return collision_points.points

    # ================================================================
    # Checker: lane_boundary
    # ================================================================

    def check_lane_boundary(self):
        current_position = self._current_position
        collision_points = CollisionPoints()

        if current_position is None:
            rospy.logwarn_throttle(3, "%s - current pose not received!", rospy.get_name())
            return collision_points.points

        for waypoint in self.local_path.waypoints:
            if get_distance_between_two_points_2d(current_position, waypoint.position) < self._lane_boundary_min_collision_point_distance:
                continue

            if waypoint.left_boundary_type not in (Waypoint.DASHED, Waypoint.SOLID_DASHED, Waypoint.VIRTUAL):
                collision_points.add_point(x=waypoint.left_boundary_point.x,
                                           y=waypoint.left_boundary_point.y,
                                           z=waypoint.position.z,
                                           distance_to_stop=self._lane_boundary_braking_safety_distance,
                                           category=CollisionPoints.LANE_BOUNDARY,
                                           deceleration=self.default_deceleration)

            if waypoint.right_boundary_type not in (Waypoint.DASHED, Waypoint.DASHED_SOLID, Waypoint.VIRTUAL):
                collision_points.add_point(x=waypoint.right_boundary_point.x,
                                           y=waypoint.right_boundary_point.y,
                                           z=waypoint.position.z,
                                           distance_to_stop=self._lane_boundary_braking_safety_distance,
                                           category=CollisionPoints.LANE_BOUNDARY,
                                           deceleration=self.default_deceleration)

        return collision_points.points

    # ================================================================
    # Run
    # ================================================================

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('collision_checker')
    node = CollisionChecker()
    node.run()
