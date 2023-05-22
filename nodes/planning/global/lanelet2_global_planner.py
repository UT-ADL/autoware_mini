#!/usr/bin/env python3

import rospy
import copy
import lanelet2
import numpy as np
from lanelet2.io import Origin, load
from lanelet2.projection import UtmProjector
from lanelet2.core import BasicPoint2d
from lanelet2.geometry import to2D, findNearest

from sklearn.neighbors import NearestNeighbors

from geometry_msgs.msg import PoseStamped, Point
from autoware_msgs.msg import Lane, Waypoint, WaypointState
from std_msgs.msg import Bool, ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker

from helpers.geometry import get_heading_between_two_points, get_distance_between_two_points_2d, get_orientation_from_heading
from helpers.waypoints import get_closest_point_on_path

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
        self.wp_left_width = rospy.get_param("~wp_left_width")
        self.wp_right_width = rospy.get_param("~wp_right_width")
        self.nearest_neighbor_search = rospy.get_param("~nearest_neighbor_search")

        lanelet2_map_name = rospy.get_param("~lanelet2_map_name")
        coordinate_transformer = rospy.get_param("/localization/coordinate_transformer")
        use_custom_origin = rospy.get_param("/localization/use_custom_origin")
        utm_origin_lat = rospy.get_param("/localization/utm_origin_lat")
        utm_origin_lon = rospy.get_param("/localization/utm_origin_lon")

        # Internal variables
        self.current_location = None
        self.goal_point = None
        self.waypoints = []

        # Load lanelet map
        if coordinate_transformer == "utm":
                projector = UtmProjector(Origin(utm_origin_lat, utm_origin_lon), use_custom_origin, False)
        else:
            rospy.logfatal("%s - only utm and custom origin currently supported for lanelet2 map loading", rospy.get_name())
            exit(1)

        self.lanelet2_map = load(lanelet2_map_name, projector)

        # traffic rules
        traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                  lanelet2.traffic_rules.Participants.Vehicle)

        # routing graph
        self.graph = lanelet2.routing.RoutingGraph(self.lanelet2_map, traffic_rules)

        # Publishers
        self.waypoints_pub = rospy.Publisher('global_path', Lane, queue_size=1, latch=True)
        self.target_lane_pub = rospy.Publisher('target_lane_markers', MarkerArray, queue_size=1, latch=True)

        # Subscribers
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback, queue_size=1)
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1)
        rospy.Subscriber('cancel_route', Bool, self.cancel_route_callback, queue_size=1)

    def goal_callback(self, msg):
        rospy.loginfo("%s - goal position (%f, %f, %f) orientation (%f, %f, %f, %f) in %s frame", rospy.get_name(),
                    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                    msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,
                    msg.pose.orientation.w, msg.header.frame_id)

        if self.current_location == None:
            # TODO handle if current_pose gets lost at later stage - see current_pose_callback
            rospy.logwarn("%s - current_pose not available", rospy.get_name())
            return

        if self.lanelet2_map == None:
            rospy.logwarn("%s - lanelet2 map not available", rospy.get_name())
            return

        # if there is already a goal, use it as start point
        start_point = self.goal_point if self.goal_point else self.current_location
        new_goal = BasicPoint2d(msg.pose.position.x, msg.pose.position.y)

        # Get nearest lanelets
        goal_lanelet = findNearest(self.lanelet2_map.laneletLayer, new_goal, 1)[0][1]
        start_lanelet = findNearest(self.lanelet2_map.laneletLayer, start_point, 1)[0][1]
        self.publish_target_lanelets(start_lanelet, goal_lanelet)

        route = self.graph.getRoute(start_lanelet, goal_lanelet, 0, True)        # lanelet2.routing.Route
        if route == None:
            rospy.logwarn("%s - no route found, try new goal!", rospy.get_name())
            return

        path = route.shortestPath()
        path_no_lane_change = path.getRemainingLane(start_lanelet)

        # check if goal is in path
        if path_no_lane_change[len(path_no_lane_change)-1].id != goal_lanelet.id:
            rospy.logwarn("%s - last lanelet in path (%d) is not goal lanelet (%d)", rospy.get_name(), path_no_lane_change[len(path_no_lane_change)-1].id, goal_lanelet.id)
            return

        waypoints = self.convert_to_waypoints(path_no_lane_change)

        # build KDTree for nearest neighbour search
        waypoints_xy = np.array([(w.pose.pose.position.x, w.pose.pose.position.y) for w in waypoints])
        waypoint_tree = NearestNeighbors(n_neighbors=1, algorithm=self.nearest_neighbor_search).fit(waypoints_xy)

        # create new start and goal waypoints
        start_idx = waypoint_tree.kneighbors([(start_point.x, start_point.y)], 1, return_distance=False)
        start_wp = self.create_waypoint_on_path(waypoints, start_idx[0][0], start_point)
        d = get_distance_between_two_points_2d(start_wp.pose.pose.position, start_point)
        if d > self.distance_to_centerline_limit:
            rospy.logwarn("%s - start point too far (%f) from centerline", rospy.get_name(), d)
            return

        goal_idx = waypoint_tree.kneighbors([(new_goal.x, new_goal.y)], 1, return_distance=False)
        goal_wp = self.create_waypoint_on_path(waypoints, goal_idx[0][0], new_goal)
        d = get_distance_between_two_points_2d(goal_wp.pose.pose.position, new_goal)
        if d > self.distance_to_centerline_limit:
            rospy.logwarn("%s - goal point too far (%f) from centerline", rospy.get_name(), d)
            return

        if start_lanelet.id == goal_lanelet.id and start_idx[0][0] > goal_idx[0][0]:
            rospy.logwarn("%s - goal point can't be before the start point on the same lanelet", rospy.get_name())
            return

        # update goal point and add new waypoints to the existing ones
        self.goal_point = BasicPoint2d(goal_wp.pose.pose.position.x, goal_wp.pose.pose.position.y)
        # replace first waypoint with start_wp and last waypoint with goal_wp
        self.waypoints += [start_wp] + waypoints[start_idx[0][0] + 1 : goal_idx[0][0]] + [goal_wp]

        self.publish_waypoints(self.waypoints)
        rospy.loginfo("%s - path published", rospy.get_name())


    def current_pose_callback(self, msg):
        self.current_location = BasicPoint2d(msg.pose.position.x, msg.pose.position.y)

        if self.goal_point != None:
            d = get_distance_between_two_points_2d(self.current_location, self.goal_point)
            if d < self.distance_to_goal_limit:
                self.waypoints = []
                self.goal_point = None
                self.publish_waypoints(self.waypoints)
                rospy.logwarn("%s - goal reached, clearing path!", rospy.get_name())

    def cancel_route_callback(self, msg):
        if msg.data:
            self.waypoints = []
            self.goal_point = None
            self.publish_waypoints(self.waypoints)
            rospy.logwarn("%s - route cancelled!", rospy.get_name())

    def create_waypoint_on_path(self, waypoints, closest_idx, origin_point):
        wp = copy.deepcopy(waypoints[closest_idx])
        # interpolate point on path
        point = get_closest_point_on_path(waypoints, closest_idx, origin_point)
        wp.pose.pose.position = point
        return wp

    def convert_to_waypoints(self, lanelet_sequence):
        waypoints = []

        last_lanelet = False

        for i , lanelet in enumerate(lanelet_sequence):
            blinker = LANELET_TURN_DIRECTION_TO_WAYPOINT_STATE_MAP[lanelet.attributes['turn_direction']]

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
                waypoint = Waypoint()
                waypoint.pose.pose.position.x = point.x
                waypoint.pose.pose.position.y = point.y
                waypoint.pose.pose.position.z = point.z
                waypoint.wpstate.steering_state = blinker

                # calculate quaternion for orientation
                if last_lanelet and idx == len(lanelet.centerline)-1:
                    # use heading of previous point - last point of last lanelet has no following point
                    heading = get_heading_between_two_points(lanelet.centerline[idx-1], lanelet.centerline[idx])
                else:
                    heading = get_heading_between_two_points(lanelet.centerline[idx], lanelet.centerline[idx+1])
                waypoint.pose.pose.orientation = get_orientation_from_heading(heading)

                waypoint.twist.twist.linear.x = speed
                waypoint.dtlane.lw = self.wp_left_width
                waypoint.dtlane.rw = self.wp_right_width

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
        marker.header.frame_id = "map"
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
    rospy.init_node('lanelet2_global_planner')
    node = Lanelet2GlobalPlanner()
    node.run()