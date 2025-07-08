#!/usr/bin/env python3
import math
import itertools
import shapely
import rospy
import lanelet2
from lanelet2.core import BasicPoint2d
from lanelet2.geometry import to2D, findWithin2d, length2d, distance as lanelet2_distance
from lanelet2.routing import RoutingCostDistance, RoutingCostTravelTime
from geometry_msgs.msg import PoseStamped, TwistStamped, Point
from autoware_mini.msg import Path, Waypoint
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Empty, EmptyResponse
from visualization_msgs.msg import MarkerArray, Marker

from autoware_mini.geometry import get_heading_between_two_points
from autoware_mini.lanelet2 import load_lanelet2_map, find_following_lane_change_lanelet
from autoware_mini.path import PathWrapper

LANELET_TURN_DIRECTION_TO_WAYPOINT_STATE_MAP = {
    "straight": Waypoint.STR_STRAIGHT,
    "left": Waypoint.STR_LEFT,
    "right": Waypoint.STR_RIGHT
}

RED = ColorRGBA(1.0, 0.0, 0.0, 0.8)
GREEN = ColorRGBA(0.0, 1.0, 0.0, 0.8)

class Lanelet2GlobalPlanner:

    def __init__(self):

        # Parameters
        self.output_frame = rospy.get_param("output_frame")
        self.distance_to_goal_limit = rospy.get_param("distance_to_goal_limit")
        self.distance_to_centerline_limit = rospy.get_param("~distance_to_centerline_limit")
        self.speed_limit = rospy.get_param("speed_limit")
        self.ego_vehicle_stopped_speed_limit = rospy.get_param("ego_vehicle_stopped_speed_limit")
        self.lane_change = rospy.get_param("~lane_change")
        self.lanelet_search_radius = rospy.get_param("~lanelet_search_radius")
        self.lane_change_base_length = rospy.get_param("lane_change_base_length")
        self.lane_change_perlane_length = rospy.get_param("lane_change_perlane_length")
        self.waypoint_interval = rospy.get_param("waypoint_interval")
        lanelet2_map_path = rospy.get_param("~lanelet2_map_path")
        self.routing_cost = rospy.get_param("~routing_cost")

        if self.routing_cost == 'distance':
            routing_costs = [RoutingCostDistance(10)]
        elif self.routing_cost == 'travel_time':
            routing_costs = [RoutingCostTravelTime(5)]
        else:
            raise ValueError(f"{rospy.get_name()} - 'routing_cost' must be one of 'distance' or 'travel_time', not '{self.routing_cost}'")

        # Internal variables
        self.lanelet_candidates = []
        self.current_location = None
        self.current_speed = None
        self.goal_point = None

        self.lanelet2_map = load_lanelet2_map(lanelet2_map_path)

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
        rospy.Service('cancel_route', Empty, self.cancel_route_callback)

    def goal_callback(self, msg):
        rospy.loginfo("%s - goal position (%f, %f, %f) orientation (%f, %f, %f, %f) in %s frame", rospy.get_name(),
                    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                    msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,
                    msg.pose.orientation.w, msg.header.frame_id)

        if self.current_location is None:
            # TODO handle if current_pose gets lost at later stage - see current_pose_callback
            rospy.logwarn("%s - current_pose not available", rospy.get_name())
            return

        # Using current pose as start point
        start_point = shapely.Point(self.current_location.x, self.current_location.y)
        # Get nearest lanelets to start point
        start_lanelet_candidates = findWithin2d(self.lanelet2_map.laneletLayer, BasicPoint2d(start_point.x, start_point.y), self.lanelet_search_radius)
        # If no lanelet found near start point, return
        if len(start_lanelet_candidates) == 0:
            rospy.logerr("%s - no lanelet found near start point", rospy.get_name())
            return
        # Extract lanelet objects from candidates
        start_lanelet_candidates = [start_lanelet[1] for start_lanelet in start_lanelet_candidates]
        lanelet_candidates = [start_lanelet_candidates] + self.lanelet_candidates[1:]

        new_goal = shapely.Point(msg.pose.position.x, msg.pose.position.y)
        # Get nearest lanelets to goal point
        goal_lanelet_candidates = findWithin2d(self.lanelet2_map.laneletLayer, BasicPoint2d(new_goal.x, new_goal.y), self.lanelet_search_radius)
        # If no lanelet found near goal point, return
        if len(goal_lanelet_candidates) == 0:
            rospy.logerr("%s - no lanelet found near goal point", rospy.get_name())
            return
        # Extract lanelet objects from candidates
        goal_lanelet_candidates = [goal_lanelet[1] for goal_lanelet in goal_lanelet_candidates]
        # Add current goal candidates to lanelet candidates list
        lanelet_candidates.append(goal_lanelet_candidates)

        # Find shortest path and shortest route
        route, stops = self.get_shortest_route(lanelet_candidates)
        if route is None:
            rospy.logerr("%s - no route found, try new goal!", rospy.get_name())
            return
        path = route.shortestPath()
        if path is None:
            rospy.logerr("%s - no path found, try new goal!", rospy.get_name())
            return

        # Publish target lanelets for visualization
        start_lanelet = path[0]
        goal_lanelet = path[-1]
        self.publish_target_lanelets(start_lanelet, goal_lanelet)

        # Convert lanelet path to waypoints
        waypoints = self.convert_to_waypoints(path, route)
        if waypoints is None:
            rospy.logerr("%s - route contained an impossible lane change!", rospy.get_name())
            return

        global_path = PathWrapper(waypoints, velocities=True, blinkers=True)

        # Find distance to start and goal waypoints
        start_point_distance = global_path.linestring.project(start_point)
        new_goal_point_distance = global_path.linestring.project(new_goal)

        # Interpolate point coordinates
        start_on_path = global_path.linestring.interpolate(start_point_distance)
        new_goal_on_path = global_path.linestring.interpolate(new_goal_point_distance)

        if shapely.distance(start_on_path, start_point) > self.distance_to_centerline_limit:
            rospy.logerr("%s - start point too far from centerline", rospy.get_name())
            return

        if shapely.distance(new_goal_on_path, new_goal) > self.distance_to_centerline_limit:
            rospy.logerr("%s - goal point too far from centerline", rospy.get_name())
            return

        if start_lanelet.id == goal_lanelet.id and start_point_distance > new_goal_point_distance:
            rospy.logerr("%s - goal point can't be on the same lanelet before start point", rospy.get_name())
            return

        # If there is only one goal candidate, we can fix the preceding lanelets to be the stops on the best found route
        if len(goal_lanelet_candidates) == 1:
            lanelet_candidates = [[lanelet] for lanelet in stops]

        # Update member variables
        self.goal_point = new_goal_on_path
        self.lanelet_candidates = lanelet_candidates
        rospy.logdebug("Lanelet candidates: " + str(list(map(len, lanelet_candidates))))

        # Trim the global path 
        trimmed_waypoints = global_path.extract_waypoints(start_point_distance, new_goal_point_distance, trim=True, copy=True)

        # Publish the global path
        self.publish_waypoints(trimmed_waypoints)
        rospy.loginfo("%s - global path published", rospy.get_name())

    def current_pose_callback(self, msg):
        self.current_location = shapely.Point(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

        if self.goal_point != None:
            d = shapely.distance(self.current_location, self.goal_point)
            if d < self.distance_to_goal_limit and self.current_speed < self.ego_vehicle_stopped_speed_limit:
                self.goal_point = None
                self.lanelet_candidates = []
                self.publish_waypoints([])
                rospy.loginfo("%s - goal reached, clearing path!", rospy.get_name())

    def current_velocity_callback(self, msg):
        self.current_speed = msg.twist.linear.x

    def cancel_route_callback(self, msg):
        self.goal_point = None
        self.lanelet_candidates = []
        self.publish_waypoints([])
        rospy.loginfo("%s - route cancelled!", rospy.get_name())
        return EmptyResponse()

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
                blinker = Waypoint.STR_LEFT
                lanechange_state += 1
            elif not last_lanelet and right_rel is not None and right_rel.lanelet == lanelet_sequence[i+1]:
                blinker = Waypoint.STR_RIGHT
                lanechange_state += 1
            else:
                blinker = None
                lanechange_state = 0

            # Make sure we have enough space to perform the lane change
            if lanechange_state > 0:
                following_lanelet = lanelet
                following_lanelets_length = length2d(following_lanelet)

                # Extend the lanelet with following lanelets until the desired lane change length is reached
                while following_lanelets_length < self.lane_change_base_length + lanechange_state * self.lane_change_perlane_length:
                    # Find a suitable following lanelet
                    if blinker == Waypoint.STR_LEFT:
                        following_lanelet = find_following_lane_change_lanelet(following_lanelet, route, True)
                    elif blinker == Waypoint.STR_RIGHT:
                        following_lanelet = find_following_lane_change_lanelet(following_lanelet, route, False)
                    else:
                        following_lanelet = None
                    
                    # If there is no following lanelet then the lane change is impossible
                    if following_lanelet is None:
                        return None

                    following_lanelets_length += length2d(following_lanelet)

            # Fetch steering state from lanelet attributes
            if blinker is None:
                if 'turn_direction' in lanelet.attributes:
                    blinker = LANELET_TURN_DIRECTION_TO_WAYPOINT_STATE_MAP[lanelet.attributes['turn_direction']]
                else:
                    blinker = Waypoint.STR_STRAIGHT

            # Fetch speed from lanelet attributes
            speed = self.speed_limit / 3.6
            if 'speed_limit' in lanelet.attributes:
                speed = min(speed, float(lanelet.attributes['speed_limit']) / 3.6)
            if 'speed_ref' in lanelet.attributes:
                speed = min(speed, float(lanelet.attributes['speed_ref']) / 3.6)

            # Loop over the current lanelet's centerline points
            for idx, point in enumerate(lanelet.centerline):
                if not last_lanelet and idx == len(lanelet.centerline)-1:
                    # Skip last point on every lanelet (except last), because it is the same as the first point of the following lanelet
                    break

                if last_lanelet and idx == len(lanelet.centerline)-1:
                    # use heading of previous point - last point of last lanelet has no following point
                    heading = get_heading_between_two_points(lanelet.centerline[idx-1], lanelet.centerline[idx])
                else:
                    heading = get_heading_between_two_points(lanelet.centerline[idx], lanelet.centerline[idx+1])

                waypoint = Waypoint()
                waypoint.position.x = point.x
                waypoint.position.y = point.y
                waypoint.position.z = point.z
                waypoint.lanechange_state = lanechange_state
                waypoint.blinker_state = blinker
                waypoint.heading = heading
                waypoint.speed = speed
                waypoint.left_width = lanelet2_distance(point, lanelet.leftBound)
                waypoint.right_width = lanelet2_distance(point, lanelet.rightBound)

                waypoints.append(waypoint)

        return waypoints

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
        for point in to2D(start_lanelet.centerline):
            marker.points.append(Point(point.x, point.y, 0.0))
        marker_array.markers.append(marker)

        marker = self.create_target_lanelet_marker()
        marker.ns = "goal_lanelet"
        marker.color = RED
        for point in to2D(goal_lanelet.centerline):
            marker.points.append(Point(point.x, point.y, 0.0))
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

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('lanelet2_global_planner', log_level=rospy.INFO)
    node = Lanelet2GlobalPlanner()
    node.run()