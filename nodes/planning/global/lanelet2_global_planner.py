#!/usr/bin/env python3
import math
import copy
import itertools

import rospy
import lanelet2
from lanelet2.core import BasicPoint2d
from lanelet2.geometry import to2D, findWithin2d, length2d, distance as lanelet2_distance
from shapely import distance, Point as ShapelyPoint

from geometry_msgs.msg import PoseStamped, TwistStamped, Point
from autoware_msgs.msg import Lane, Waypoint, WaypointState
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Empty, EmptyResponse
from visualization_msgs.msg import MarkerArray, Marker

from helpers.geometry import get_heading_between_two_points, get_orientation_from_heading
from helpers.lanelet2 import load_lanelet2_map
from helpers.path import Path

LANELET_TURN_DIRECTION_TO_WAYPOINT_STATE_MAP = {
    "straight": WaypointState.STR_STRAIGHT,
    "left": WaypointState.STR_LEFT,
    "right": WaypointState.STR_RIGHT
}

RED = ColorRGBA(1.0, 0.0, 0.0, 0.8)
GREEN = ColorRGBA(0.0, 1.0, 0.0, 0.8)

class Lanelet2GlobalPlanner:

    def __init__(self):

        # Parameters
        self.output_frame = rospy.get_param("~output_frame")
        self.distance_to_goal_limit = rospy.get_param("~distance_to_goal_limit")
        self.distance_to_centerline_limit = rospy.get_param("~distance_to_centerline_limit")
        self.speed_limit = rospy.get_param("~speed_limit")
        self.ego_vehicle_stopped_speed_limit = rospy.get_param("~ego_vehicle_stopped_speed_limit")
        self.lane_change = rospy.get_param("~lane_change")
        self.lanelet_search_radius = rospy.get_param("~lanelet_search_radius")

        lanelet2_map_name = rospy.get_param("~lanelet2_map_name")
        coordinate_transformer = rospy.get_param("/localization/coordinate_transformer")
        use_custom_origin = rospy.get_param("/localization/use_custom_origin")
        utm_origin_lat = rospy.get_param("/localization/utm_origin_lat")
        utm_origin_lon = rospy.get_param("/localization/utm_origin_lon")

        # Internal variables
        self.lanelet_candidates = []
        self.current_location = None
        self.current_speed = None
        self.start_point = None
        self.goal_point = None

        self.lanelet2_map = load_lanelet2_map(lanelet2_map_name, coordinate_transformer, use_custom_origin, utm_origin_lat, utm_origin_lon)

        # traffic rules
        traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                  lanelet2.traffic_rules.Participants.VehicleTaxi)

        # routing graph
        self.graph = lanelet2.routing.RoutingGraph(self.lanelet2_map, traffic_rules)

        # Publishers
        self.waypoints_pub = rospy.Publisher('global_path', Lane, queue_size=10, latch=True, tcp_nodelay=True)
        self.target_lane_pub = rospy.Publisher('target_lane_markers', MarkerArray, queue_size=10, latch=True, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback, queue_size=None, tcp_nodelay=True)
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/localization/current_velocity', TwistStamped, self.current_velocity_callback, queue_size=1, tcp_nodelay=True)

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
        if self.start_point is None:
            start_point = ShapelyPoint(self.current_location.x, self.current_location.y)
            # Get nearest lanelets to start point
            start_lanelet_candidates = findWithin2d(self.lanelet2_map.laneletLayer, BasicPoint2d(start_point.x, start_point.y), self.lanelet_search_radius)
            # If no lanelet found near start point, return
            if len(start_lanelet_candidates) == 0:
                rospy.logerr("%s - no lanelet found near start point", rospy.get_name())
                return
            # Extract lanelet objects from candidates
            start_lanelet_candidates = [start_lanelet[1] for start_lanelet in start_lanelet_candidates]
            lanelet_candidates = [start_lanelet_candidates]
        else:
            start_point = self.start_point
            lanelet_candidates = copy.copy(self.lanelet_candidates)
        
        new_goal = ShapelyPoint(msg.pose.position.x, msg.pose.position.y)
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
        path, route = self.get_shortest_path_with_route(lanelet_candidates)
        if path is None:
            rospy.logerr("%s - no route found, try new goal!", rospy.get_name())
            return

        # Publish target lanelets for visualization
        start_lanelet = path[0]
        goal_lanelet = path[-1]
        self.publish_target_lanelets(start_lanelet, goal_lanelet)
        
        global_path = Path(self.convert_to_waypoints(path), velocities=True, blinkers=True)

        # Find distance to start and goal waypoints
        start_point_distance = global_path.linestring.project(start_point)
        new_goal_point_distance = global_path.linestring.project(new_goal)
        # interpolate point coordinates
        start_on_path = global_path.linestring.interpolate(start_point_distance)
        new_goal_on_path = global_path.linestring.interpolate(new_goal_point_distance)

        if distance(start_on_path, start_point) > self.distance_to_centerline_limit:
            rospy.logerr("%s - start point too far from centerline", rospy.get_name())
            return

        if distance(new_goal_on_path, new_goal) > self.distance_to_centerline_limit:
            rospy.logerr("%s - goal point too far from centerline", rospy.get_name())
            return

        if start_lanelet.id == goal_lanelet.id and start_point_distance > new_goal_point_distance:
            rospy.logerr("%s - goal point can't be on the same lanelet before start point", rospy.get_name())
            return

        # If there is only one goal candidate, we can fix the preceding lanelets to be the best found route
        if len(goal_lanelet_candidates) == 1:
            lanelet_candidates = [[lanelet] for lanelet in route]

        # update member variables
        self.goal_point = new_goal_on_path
        self.start_point = start_point
        self.lanelet_candidates = lanelet_candidates
        rospy.logdebug("Lanelet candidates: " + str(list(map(len, lanelet_candidates))))

        # publish the global path
        waypoints = global_path.extract_waypoints(start_point_distance, new_goal_point_distance, trim=True)
        self.publish_waypoints(waypoints)
        rospy.loginfo("%s - global path published", rospy.get_name())


    def current_pose_callback(self, msg):
        self.current_location = ShapelyPoint(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

        if self.goal_point != None:
            d = distance(self.current_location, self.goal_point)
            if d < self.distance_to_goal_limit and self.current_speed < self.ego_vehicle_stopped_speed_limit:
                self.goal_point = None
                self.start_point = None
                self.lanelet_candidates = []
                self.publish_waypoints([])
                rospy.loginfo("%s - goal reached, clearing path!", rospy.get_name())

    def current_velocity_callback(self, msg):
        self.current_speed = msg.twist.linear.x

    def cancel_route_callback(self, msg):
        self.goal_point = None
        self.start_point = None
        self.lanelet_candidates = []
        self.publish_waypoints([])
        rospy.loginfo("%s - route cancelled!", rospy.get_name())
        return EmptyResponse()
    
    def get_shortest_path_with_route(self, lanelet_candidates):
        shortest_path = None
        shortest_route = None
        shortest_distance = math.inf
        possible_routes = list(itertools.product(*lanelet_candidates))
        for possible_route in possible_routes:
            path = self.graph.shortestPathWithVia(possible_route[0], possible_route[1:-1], possible_route[-1], 0, self.lane_change)
            if path is not None:
                path_length = sum(map(length2d, path))
                if path_length < shortest_distance:
                    shortest_distance = path_length
                    shortest_route = possible_route
                    shortest_path = path

        return shortest_path, shortest_route

    def convert_to_waypoints(self, lanelet_sequence):
        waypoints = []

        last_lanelet = False

        for i , lanelet in enumerate(lanelet_sequence):
            if 'turn_direction' in lanelet.attributes:
                blinker = LANELET_TURN_DIRECTION_TO_WAYPOINT_STATE_MAP[lanelet.attributes['turn_direction']]
            else:
                blinker = WaypointState.STR_STRAIGHT

            if i == len(lanelet_sequence)-1:
                last_lanelet = True

            speed = self.speed_limit / 3.6
            if 'speed_limit' in lanelet.attributes:
                speed = min(speed, float(lanelet.attributes['speed_limit']) / 3.6)
            if 'speed_ref' in lanelet.attributes:
                speed = min(speed, float(lanelet.attributes['speed_ref']) / 3.6)

            # loop over centerline points use enumerate to get index
            for idx, point in enumerate(lanelet.centerline):
                if not last_lanelet and idx == len(lanelet.centerline)-1:
                    # skip last point on every lanelet (except last), because it is the same as the first point of the following lanelet
                    break

                # calculate quaternion for orientation
                if last_lanelet and idx == len(lanelet.centerline)-1:
                    # use heading of previous point - last point of last lanelet has no following point
                    heading = get_heading_between_two_points(lanelet.centerline[idx-1], lanelet.centerline[idx])
                else:
                    heading = get_heading_between_two_points(lanelet.centerline[idx], lanelet.centerline[idx+1])

                waypoint = Waypoint()
                waypoint.pose.pose.position.x = point.x
                waypoint.pose.pose.position.y = point.y
                waypoint.pose.pose.position.z = point.z
                waypoint.pose.pose.orientation = get_orientation_from_heading(heading)
                waypoint.wpstate.steering_state = blinker
                waypoint.twist.twist.linear.x = speed
                waypoint.dtlane.lw = lanelet2_distance(point, lanelet.leftBound)
                waypoint.dtlane.rw = lanelet2_distance(point, lanelet.rightBound)

                waypoints.append(waypoint)

        return waypoints


    def publish_waypoints(self, waypoints):

        lane = Lane()        
        lane.header.frame_id = self.output_frame
        lane.header.stamp = rospy.Time.now()
        lane.waypoints = waypoints
        
        self.waypoints_pub.publish(lane)


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