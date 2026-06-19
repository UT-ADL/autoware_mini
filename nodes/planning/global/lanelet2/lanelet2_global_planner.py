#!/usr/bin/env python3

import math
import itertools
import threading
from datetime import datetime, timezone
import numpy as np
import shapely
import rospy
import lanelet2

import tf2_ros
from geometry_msgs.msg import PoseStamped, TwistStamped, Point
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Empty, EmptyResponse
from nav_msgs.srv import GetPlan, GetPlanResponse
from visualization_msgs.msg import MarkerArray, Marker

from autoware_mini.msg import Path, Waypoint
from autoware_mini.lanelet2 import load_lanelet2_map, find_following_lane_change_lanelet, get_stop_lines, get_traffic_light_stop_lines
from autoware_mini.shapely import ensure_points
from autoware_mini.path import PathWrapper
from autoware_mini.transform import transform_pose
from autoware_mini.geometry import get_distance_between_two_points_2d

LANELET_TURN_DIRECTION_TO_WAYPOINT_TURN_SIGNAL_MAP = {
    "straight": Waypoint.TURN_STRAIGHT,
    "left": Waypoint.TURN_LEFT,
    "right": Waypoint.TURN_RIGHT
}

LANELET_BOUNDARY_TYPE_MAP = {
    "solid": Waypoint.SOLID,
    "solid_solid": Waypoint.SOLID_SOLID,
    "dashed": Waypoint.DASHED,
    "dashed_solid": Waypoint.DASHED_SOLID,
    "solid_dashed": Waypoint.SOLID_DASHED,
    "keepout": Waypoint.KEEPOUT,
    "zebra_marking": Waypoint.PEDESTRIAN_MARKING,
    "pedestrian_marking": Waypoint.PEDESTRIAN_MARKING,
    "curbstone": Waypoint.CURBSTONE,
    "road_border": Waypoint.ROAD_BORDER,
    "virtual": Waypoint.VIRTUAL
}

STOP_LINE_TYPE_MAP = {
    "yield": Waypoint.STOP_LINE_YIELD,
    "yield_stop": Waypoint.STOP_LINE_YIELD_STOP,
    "yield_manual": Waypoint.STOP_LINE_YIELD_MANUAL,
    "yield_right": Waypoint.STOP_LINE_YIELD_RIGHT,
    "priority": Waypoint.STOP_LINE_PRIORITY,
    "speed_bump": Waypoint.SPEED_BUMP,
}

SLOWDOWN_STOP_LINE_TYPES = {Waypoint.STOP_LINE_YIELD, Waypoint.STOP_LINE_YIELD_RIGHT, Waypoint.STOP_LINE_YIELD_MANUAL, Waypoint.SPEED_BUMP}

RED = ColorRGBA(1.0, 0.0, 0.0, 0.8)
GREEN = ColorRGBA(0.0, 1.0, 0.0, 0.8)

class Lanelet2GlobalPlanner:

    def __init__(self):

        # Parameters
        self.output_frame = rospy.get_param("output_frame")
        self.distance_to_goal_limit = rospy.get_param("distance_to_goal_limit")
        self.distance_to_centerline_limit = rospy.get_param("~distance_to_centerline_limit")
        self.enable_reroute = rospy.get_param("~enable_reroute")
        self.reroute_limit = rospy.get_param("~reroute_limit")
        self.reroute_frequency = rospy.get_param("~reroute_frequency")
        self.speed_limit = rospy.get_param("speed_limit")
        self.ego_vehicle_stopped_speed_limit = rospy.get_param("ego_vehicle_stopped_speed_limit")
        self.lane_change = rospy.get_param("~lane_change")
        self.lanelet_search_radius = rospy.get_param("~lanelet_search_radius")
        self.lane_change_base_length = rospy.get_param("lane_change_base_length")
        self.lane_change_perlane_length = rospy.get_param("lane_change_perlane_length")
        self.waypoint_interval = rospy.get_param("waypoint_interval")
        lanelet2_map_path = rospy.get_param("~lanelet2_map_path")
        self.routing_cost = rospy.get_param("~routing_cost")
        self.yield_slowdown_factor = rospy.get_param("~yield_slowdown_factor")
        self.enable_road_closures = rospy.get_param("~enable_road_closures")

        if self.routing_cost == 'distance':
            routing_costs = [lanelet2.routing.RoutingCostDistance(10)]
        elif self.routing_cost == 'travel_time':
            routing_costs = [lanelet2.routing.RoutingCostTravelTime(5)]
        else:
            raise ValueError(f"{rospy.get_name()} - 'routing_cost' must be one of 'distance' or 'travel_time', not '{self.routing_cost}'")

        # Internal variables
        self.lock = threading.Lock()
        self.goal_points = []
        self.goal_lanelets = []
        self.current_position = None
        self.current_speed = None
        self.route_linestring = None
        self.cancel_requested = False
        self.current_waypoints = []

        self.tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.lanelet2_map = load_lanelet2_map(lanelet2_map_path, print_errors=True)
        if self.enable_road_closures:
            self.block_closed_lanelets()

        # stop lines
        stop_line_geoms, stop_line_subtypes, stop_line_speeds = get_stop_lines(
            self.lanelet2_map, subtypes=STOP_LINE_TYPE_MAP, return_subtypes=True, return_speeds=True)
        self.stop_line_ids = np.array(list(stop_line_geoms.keys()))
        self.stop_line_geoms = np.array(list(stop_line_geoms.values()))
        self.stop_line_types = np.array([STOP_LINE_TYPE_MAP[stop_line_subtypes[sl_id]] for sl_id in self.stop_line_ids])
        self.stop_line_speeds = np.array([stop_line_speeds[sl_id] for sl_id in self.stop_line_ids])
        self.traffic_light_stop_line_ids = set(get_traffic_light_stop_lines(self.lanelet2_map).keys())

        # traffic rules
        traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                  lanelet2.traffic_rules.Participants.VehicleTaxi)

        # routing graph
        self.graph = lanelet2.routing.RoutingGraph(self.lanelet2_map, traffic_rules, routing_costs)

        # Publishers
        self.waypoints_pub = rospy.Publisher('lanelet2_global_path', Path, queue_size=10, latch=True, tcp_nodelay=True)
        self.target_lane_pub = rospy.Publisher('target_lane_markers', MarkerArray, queue_size=10, latch=True, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/localization/current_velocity', TwistStamped, self.current_velocity_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback, queue_size=None, tcp_nodelay=True)
        
        # Services
        rospy.Service('cancel_route', Empty, self.cancel_route_handler)
        rospy.Service('get_plan', GetPlan, self.get_plan_handler)

        # Timers
        if self.enable_reroute:
            rospy.Timer(rospy.Duration(self.reroute_frequency), self.reroute_callback)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def goal_callback(self, msg):
        rospy.loginfo("%s - goal position (%f, %f, %f) orientation (%f, %f, %f, %f) in %s frame", rospy.get_name(),
                    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                    msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,
                    msg.pose.orientation.w, msg.header.frame_id)

        if self.current_position is None:
            # TODO handle if current_pose gets lost at later stage - see current_pose_callback
            rospy.logwarn("%s - current_pose not available", rospy.get_name())
            return
        
        if msg.header.frame_id != "map":
            # Convert non-map frame to map frame
            transform = self.tf_buffer.lookup_transform("map", msg.header.frame_id, msg.header.stamp, rospy.Duration(0.06))
            # Construct PoseStamped from the extracted position and heading
            msg.pose = transform_pose(transform, msg.pose)

        result = self.compute_path(self.current_position, msg.pose.position, self.goal_lanelets)
        if result is None:
            return

        waypoints, stops, lanelet_candidates, route_linestring = result

        # Publish target lanelets for visualization
        self.publish_target_lanelets(stops[0], stops[-1])

        # Update member variables
        with self.lock:
            self.goal_points = self.goal_points + [msg.pose.position]
            self.goal_lanelets = lanelet_candidates[1:]
            self.route_linestring = route_linestring
            self.current_waypoints = waypoints
            self.publish_waypoints(waypoints)
        rospy.loginfo("%s - global path published", rospy.get_name())

    def clear_route(self):
        with self.lock:
            self.goal_points = []
            self.goal_lanelets = []
            self.route_linestring = None
            self.cancel_requested = False
            self.current_waypoints = []
            self.publish_waypoints([])

    def current_pose_callback(self, msg):
        self.current_position = msg.pose.position

        # Check if cancel was requested and vehicle has stopped
        if self.cancel_requested and self.current_speed is not None and self.current_speed < self.ego_vehicle_stopped_speed_limit:
            self.clear_route()
            rospy.loginfo("%s - vehicle stopped, route cancelled!", rospy.get_name())
            return

        if not self.goal_points:
            return

        d = get_distance_between_two_points_2d(self.current_position, self.goal_points[0])
        if d < self.distance_to_goal_limit:
            if len(self.goal_points) > 1:
                # Remove passed intermediate goal point
                with self.lock:
                    self.goal_points = self.goal_points[1:]
                    self.goal_lanelets = self.goal_lanelets[1:]
                rospy.loginfo("%s - intermediate goal reached, %d goal(s) remaining", rospy.get_name(), len(self.goal_points))
            elif self.current_speed is not None and self.current_speed < self.ego_vehicle_stopped_speed_limit:
                # Final goal reached
                self.clear_route()
                rospy.loginfo("%s - goal reached, clearing path!", rospy.get_name())

    def current_velocity_callback(self, msg):
        self.current_speed = msg.twist.linear.x

    def reroute_callback(self, event):
        try:
            # Snapshot shared state under lock
            with self.lock:
                goal_points = self.goal_points
                goal_lanelets = self.goal_lanelets
                route_linestring = self.route_linestring

            if self.current_position is None or not goal_points or route_linestring is None:
                return

            current_point = shapely.Point(self.current_position.x, self.current_position.y)
            if shapely.distance(route_linestring, current_point) <= self.reroute_limit:
                return

            rospy.loginfo("%s - off route, replanning", rospy.get_name())
            result = self.compute_path(self.current_position, goal_points[-1], goal_lanelets[:-1])

            if result is not None:
                waypoints, stops, lanelet_candidates, route_linestring = result
                self.publish_target_lanelets(stops[0], stops[-1])
                with self.lock:
                    self.goal_lanelets = lanelet_candidates[1:]
                    self.route_linestring = route_linestring
                    self.current_waypoints = waypoints
                    self.publish_waypoints(waypoints)
                rospy.loginfo("%s - replanned route published", rospy.get_name())
            else:
                rospy.logwarn("%s - replanning failed", rospy.get_name())

        except Exception as e:
            rospy.logerr("%s - reroute failed: %s", rospy.get_name(), str(e))

    def cancel_route_handler(self, msg):
        try:
            # If vehicle is already stopped, not moving, or cancel already requested, clear path immediately
            if self.current_speed is None or self.current_speed < self.ego_vehicle_stopped_speed_limit or self.cancel_requested:
                self.clear_route()
                rospy.loginfo("%s - route cancelled!", rospy.get_name())
            else:
                # Vehicle is moving - keep path but set all speeds to 0 for safe stopping
                with self.lock:
                    self.cancel_requested = True

                    # Set the speed of waypoints to 0
                    for wp in self.current_waypoints:
                        wp.speed = 0.0

                    self.publish_waypoints(self.current_waypoints)
                rospy.loginfo("%s - route cancellation requested, stopping vehicle before clearing path", rospy.get_name())
            return EmptyResponse()
        except Exception as e:
            rospy.logerr("%s - cancel_route service failed: %s", rospy.get_name(), str(e))
            return EmptyResponse()

    def get_plan_handler(self, req):
        try:
            rospy.loginfo("%s - get_plan request: start (%f, %f), goal (%f, %f)", rospy.get_name(),
                          req.start.pose.position.x, req.start.pose.position.y,
                          req.goal.pose.position.x, req.goal.pose.position.y)

            response = GetPlanResponse()
            response.plan.header.frame_id = self.output_frame
            response.plan.header.stamp = rospy.Time.now()

            result = self.compute_path(req.start.pose.position, req.goal.pose.position, [])
            if result is None:
                return response

            waypoints = result[0]

            # Already at destination: return a single pose at the goal so callers see a non-empty successful route
            if not waypoints:
                pose = PoseStamped()
                pose.header.frame_id = self.output_frame
                pose.pose.position = req.goal.pose.position
                pose.pose.orientation.w = 1.0
                response.plan.poses.append(pose)
                rospy.loginfo("%s - get_plan response with 1 pose (already at destination)", rospy.get_name())
                return response

            # Convert our Waypoints to nav_msgs/Path poses
            # Speed is encoded in orientation.x (hack: GetPlan uses PoseStamped which has no speed field)
            for wp in waypoints:
                pose = PoseStamped()
                pose.header.frame_id = self.output_frame
                pose.pose.position.x = wp.position.x
                pose.pose.position.y = wp.position.y
                pose.pose.position.z = wp.position.z
                pose.pose.orientation.x = wp.speed
                pose.pose.orientation.w = 1.0
                response.plan.poses.append(pose)

            rospy.loginfo("%s - get_plan response with %d poses", rospy.get_name(), len(response.plan.poses))
            return response
        except Exception as e:
            rospy.logerr("%s - get_plan service failed: %s", rospy.get_name(), str(e))
            return GetPlanResponse()

    def compute_path(self, start_point, goal_point, intermediate_lanelet_candidates):
        # Get nearest lanelets to start point
        start_lanelet_point = lanelet2.core.BasicPoint2d(start_point.x, start_point.y)
        start_lanelet_candidates = lanelet2.geometry.findWithin2d(self.lanelet2_map.laneletLayer, start_lanelet_point, self.lanelet_search_radius)
        if not start_lanelet_candidates:
            rospy.logerr("%s - no lanelet found near start point", rospy.get_name())
            return None
        start_lanelet_candidates = [candidate[1] for candidate in start_lanelet_candidates]

        # Get nearest lanelets to goal point
        goal_lanelet_point = lanelet2.core.BasicPoint2d(goal_point.x, goal_point.y)
        goal_lanelet_candidates = lanelet2.geometry.findWithin2d(self.lanelet2_map.laneletLayer, goal_lanelet_point, self.lanelet_search_radius)
        if not goal_lanelet_candidates:
            rospy.logerr("%s - no lanelet found near goal point", rospy.get_name())
            return None
        goal_lanelet_candidates = [candidate[1] for candidate in goal_lanelet_candidates]

        # Build full lanelet candidates list
        lanelet_candidates = [start_lanelet_candidates] + intermediate_lanelet_candidates + [goal_lanelet_candidates]

        # Find shortest path and shortest route
        route, stops = self.get_shortest_route(lanelet_candidates)
        if route is None:
            rospy.logerr("%s - no route found", rospy.get_name())
            return None
        path = route.shortestPath()
        if path is None:
            rospy.logerr("%s - no path found", rospy.get_name())
            return None

        # Convert lanelet path to waypoints
        waypoints = self.convert_to_waypoints(path, route)
        if waypoints is None:
            rospy.logerr("%s - route contained an impossible lane change", rospy.get_name())
            return None

        # Mark or insert stop line waypoints at exact intersection positions
        self.mark_stop_lines_on_waypoints(waypoints)
        global_path = PathWrapper(waypoints)

        # Find distance to start and goal waypoints
        start_point = shapely.Point(start_point.x, start_point.y, start_point.z)
        start_point_distance = global_path.linestring.project(start_point)

        goal_point = shapely.Point(goal_point.x, goal_point.y, goal_point.z)
        goal_point_distance = global_path.linestring.project(goal_point)

        # Interpolate point coordinates
        start_on_path = global_path.linestring.interpolate(start_point_distance)
        goal_on_path = global_path.linestring.interpolate(goal_point_distance)

        if shapely.distance(start_on_path, start_point) > self.distance_to_centerline_limit:
            rospy.logerr("%s - start point too far from centerline", rospy.get_name())
            return None

        if shapely.distance(goal_on_path, goal_point) > self.distance_to_centerline_limit:
            rospy.logerr("%s - goal point too far from centerline", rospy.get_name())
            return None

        start_lanelet = path[0]
        goal_lanelet = path[-1]
        if start_lanelet.id == goal_lanelet.id and start_point_distance > goal_point_distance:
            rospy.logerr("%s - goal point can't be on the same lanelet before start point", rospy.get_name())
            return None

        # Trim the global path
        waypoints = global_path.extract_waypoints(start_point_distance, goal_point_distance, trim=True, copy=True)

        # If there is only one goal candidate, fix the lanelets to be the stops on the best found route
        if len(lanelet_candidates[-1]) == 1:
            lanelet_candidates = [[lanelet] for lanelet in stops]

        return waypoints, stops, lanelet_candidates, global_path.linestring

    def get_shortest_route(self, lanelet_candidates):
        shortest_route = None
        shortest_stops = None
        shortest_distance = math.inf
        possible_routes = list(itertools.product(*lanelet_candidates))
        for possible_route in possible_routes:
            route = self.graph.getRouteVia(possible_route[0], possible_route[1:-1], possible_route[-1], 0, self.lane_change)
            if route is None:
                continue

            route_length = route.length2d()
            if route_length < shortest_distance:
                shortest_distance = route_length
                shortest_route = route
                shortest_stops = possible_route

        return shortest_route, shortest_stops

    def convert_to_waypoints(self, lanelet_sequence, route):
        waypoints = []

        last_lanelet = False
        lanechange_state = 0

        for i, lanelet in enumerate(lanelet_sequence):
            if i == len(lanelet_sequence)-1:
                last_lanelet = True

            # Check for lane change
            left_rel = route.leftRelation(lanelet)
            right_rel = route.rightRelation(lanelet)
            if not last_lanelet and left_rel is not None and left_rel.lanelet == lanelet_sequence[i+1]:
                turn_signal = Waypoint.TURN_LEFT
                lanechange_state += 1
            elif not last_lanelet and right_rel is not None and right_rel.lanelet == lanelet_sequence[i+1]:
                turn_signal = Waypoint.TURN_RIGHT
                lanechange_state += 1
            else:
                turn_signal = None
                lanechange_state = 0

            # Make sure we have enough space to perform the lane change
            if lanechange_state > 0:
                following_lanelet = lanelet
                following_lanelets_length = lanelet2.geometry.length2d(following_lanelet)

                # Extend the lanelet with following lanelets until the desired lane change length is reached
                while following_lanelets_length < self.lane_change_base_length + lanechange_state * self.lane_change_perlane_length:
                    # Find a suitable following lanelet
                    if turn_signal == Waypoint.TURN_LEFT:
                        following_lanelet = find_following_lane_change_lanelet(following_lanelet, route, True)
                    elif turn_signal == Waypoint.TURN_RIGHT:
                        following_lanelet = find_following_lane_change_lanelet(following_lanelet, route, False)
                    else:
                        following_lanelet = None
                    
                    # If there is no following lanelet then the lane change is impossible
                    if following_lanelet is None:
                        return None

                    following_lanelets_length += lanelet2.geometry.length2d(following_lanelet)

            # Fetch steering state from lanelet attributes
            if turn_signal is None:
                if 'turn_direction' in lanelet.attributes:
                    turn_signal = LANELET_TURN_DIRECTION_TO_WAYPOINT_TURN_SIGNAL_MAP[lanelet.attributes['turn_direction']]
                else:
                    turn_signal = Waypoint.TURN_STRAIGHT

            priority = False
            # Fetch priority from lanelet attributes, if not specified set priority to True for straight driving lanelets
            if 'priority' in lanelet.attributes:
                priority = lanelet.attributes['priority'].lower() == 'yes'
            elif turn_signal == Waypoint.TURN_STRAIGHT:
                priority = True

            # Fetch speed from lanelet attributes
            speed_limit = self.speed_limit / 3.6
            if 'speed_limit' in lanelet.attributes:
                speed_limit = min(speed_limit, float(lanelet.attributes['speed_limit']) / 3.6)
            speed = speed_limit
            if 'speed_ref' in lanelet.attributes:
                speed = min(speed, float(lanelet.attributes['speed_ref']) / 3.6)

            # Loop over the current lanelet's centerline points
            for idx, point in enumerate(lanelet.centerline):
                if not last_lanelet and lanechange_state == 0 and idx == len(lanelet.centerline)-1:
                    # Skip last point on every lanelet (except last and lane change lanelets), because it is the same as the first point of the following lanelet
                    break

                left_point = lanelet2.geometry.project(lanelet.leftBound, point.basicPoint())
                right_point = lanelet2.geometry.project(lanelet.rightBound, point.basicPoint())

                waypoint = Waypoint()
                waypoint.position.x = point.x
                waypoint.position.y = point.y
                waypoint.position.z = point.z
                waypoint.lanechange_state = lanechange_state
                waypoint.priority = priority
                waypoint.turn_signal = turn_signal
                waypoint.speed = speed
                waypoint.speed_limit = speed_limit
                waypoint.left_boundary_point.x = left_point.x
                waypoint.left_boundary_point.y = left_point.y
                waypoint.left_boundary_point.z = left_point.z
                waypoint.right_boundary_point.x = right_point.x
                waypoint.right_boundary_point.y = right_point.y
                waypoint.right_boundary_point.z = right_point.z
                waypoint.left_boundary_type = self.get_boundary_type(lanelet.leftBound)
                waypoint.right_boundary_type = self.get_boundary_type(lanelet.rightBound)

                waypoints.append(waypoint)

        return waypoints

    def mark_stop_lines_on_waypoints(self, waypoints):
        """Mark waypoints at stop line intersection positions with stop_line_id and stop_line_type."""

        path = PathWrapper(waypoints)

        # Find which stop lines intersect the path
        mask = path.linestring.intersects(self.stop_line_geoms)
        if not mask.any():
            return

        intersecting_ids = self.stop_line_ids[mask]
        intersecting_geoms = self.stop_line_geoms[mask]
        intersecting_types = self.stop_line_types[mask]
        intersecting_speeds = self.stop_line_speeds[mask]

        # Compute intersection points and distances
        intersection_results = path.linestring.intersection(intersecting_geoms)
        intersection_points = ensure_points(intersection_results)
        intersection_distances = path.linestring.project(intersection_points)

        # Insert new waypoints in reverse distance order to preserve indices
        insertions = sorted(zip(intersection_distances, intersecting_ids, intersecting_types, intersecting_speeds), reverse=True)
        for sl_distance, sl_id, sl_type, sl_speed in insertions:
            idx = path.get_waypoint_index_at_distance(sl_distance, side="left")

            waypoint = Waypoint()
            waypoint.position = path.get_point_at_distance(sl_distance)
            waypoint.speed = float(path.get_speed_at_distance(sl_distance))
            waypoint.speed_limit = float(path.get_speed_limit_at_distance(sl_distance))
            waypoint.turn_signal = int(path.get_turn_signal_at_distance(sl_distance))
            waypoint.priority = bool(path.get_priority_at_distance(sl_distance))
            waypoint.left_boundary_point = path.get_left_boundary_point_at_distance(sl_distance)
            waypoint.right_boundary_point = path.get_right_boundary_point_at_distance(sl_distance)
            waypoint.left_boundary_type = int(path.get_left_boundary_type_at_distance(sl_distance))
            waypoint.right_boundary_type = int(path.get_right_boundary_type_at_distance(sl_distance))
            waypoint.stop_line_id = int(sl_id)
            waypoint.stop_line_type = int(sl_type)

            # Apply slowdown speed
            if int(sl_type) in SLOWDOWN_STOP_LINE_TYPES and int(sl_id) not in self.traffic_light_stop_line_ids:
                waypoint.speed = min(waypoint.speed, waypoint.speed_limit * self.yield_slowdown_factor)
                if np.isfinite(sl_speed):
                    waypoint.speed = min(waypoint.speed, float(sl_speed) / 3.6)

            waypoints.insert(idx, waypoint)

    def get_boundary_type(self, boundary):
        if boundary.attributes["type"] == "line_thin" or boundary.attributes["type"] == "line_thick":
            if boundary.attributes["subtype"] in LANELET_BOUNDARY_TYPE_MAP:
                return LANELET_BOUNDARY_TYPE_MAP[boundary.attributes["subtype"]]
            else:
                return Waypoint.SOLID
        else:
            if boundary.attributes["type"] in LANELET_BOUNDARY_TYPE_MAP:
                return LANELET_BOUNDARY_TYPE_MAP[boundary.attributes["type"]]
            else:
                return Waypoint.ROAD_BORDER

    def publish_waypoints(self, waypoints):
        path = Path()
        path.header.frame_id = self.output_frame
        path.header.stamp = rospy.Time.now()
        path.waypoints = waypoints

        self.waypoints_pub.publish(path)

    def publish_target_lanelets(self, start_lanelet, goal_lanelet):

        marker_array = MarkerArray()

        # create correct ones
        marker = self.create_target_lanelet_marker()
        marker.ns = "start_lanelet"
        marker.color = GREEN
        for point in lanelet2.geometry.to2D(start_lanelet.centerline):
            marker.points.append(Point(x=point.x, y=point.y, z=0.0))
        marker_array.markers.append(marker)

        marker = self.create_target_lanelet_marker()
        marker.ns = "goal_lanelet"
        marker.color = RED
        for point in lanelet2.geometry.to2D(goal_lanelet.centerline):
            marker.points.append(Point(x=point.x, y=point.y, z=0.0))
        marker_array.markers.append(marker)

        self.target_lane_pub.publish(marker_array)

    def create_target_lanelet_marker(self):
        marker = Marker()
        marker.header.frame_id = self.output_frame
        marker.header.stamp = rospy.Time.now()
        marker.action = Marker.ADD
        marker.type = Marker.POINTS
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        return marker
    
    def block_closed_lanelets(self):
        now = datetime.now(timezone.utc)

        # Create an array of shapely polygons from road closure areas
        road_closure_polygons = []
        for polygon in self.lanelet2_map.polygonLayer:
            if polygon.attributes["type"] == "road_closure":
                # check if the road closure is currently active
                start_time = datetime.fromisoformat(polygon.attributes["start_time"])
                end_time = datetime.fromisoformat(polygon.attributes["end_time"])
                if start_time <= now <= end_time:
                    road_closure_polygons.append(shapely.polygons([(point.x, point.y) for point in polygon]))

        # Create an array of shapely polygons from lanelets
        lanelet_ids = []
        lanelet_polygons = []
        for lanelet in self.lanelet2_map.laneletLayer:
            lanelet_ids.append(lanelet.id)
            lanelet_polygons.append(shapely.polygons([(point.x, point.y) for point in lanelet.polygon2d()]))

        road_closure_polygons = np.array(road_closure_polygons)[:, np.newaxis]
        lanelet_polygons = np.array(lanelet_polygons)[np.newaxis, :]
    
        # Find lanelets that intersect with road closure areas
        intersects_matrix = shapely.intersects(road_closure_polygons, lanelet_polygons)
        closed_lanelet_idxs = np.where(np.any(intersects_matrix, axis=0))[0]

        # Set speed limit of closed lanelets to 0
        for idx in closed_lanelet_idxs:
            lanelet = self.lanelet2_map.laneletLayer.get(lanelet_ids[idx])
            lanelet.attributes["speed_limit"] = "0"

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('lanelet2_global_planner', log_level=rospy.INFO)
    node = Lanelet2GlobalPlanner()
    node.run()