#!/usr/bin/env python3

import rospy
import lanelet2
import numpy as np
from lanelet2.io import Origin, load
from lanelet2.projection import UtmProjector
from lanelet2.core import BasicPoint2d
from lanelet2.geometry import distance, to2D

from sklearn.neighbors import KDTree

from geometry_msgs.msg import PoseStamped, Point
from autoware_msgs.msg import Lane, Waypoint, WaypointState
from std_msgs.msg import Bool, ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker
from tf.transformations import quaternion_from_euler
from helpers import get_heading_between_two_points

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
        self.lanelet2_map_name = rospy.get_param("~lanelet2_map_name")
        self.output_frame = rospy.get_param("~output_frame", map)
        self.distance_to_centerline_limit = rospy.get_param("~distance_to_centerline_limit", 5.0)
        self.speed_limit = rospy.get_param("~speed_limit", 40.0) / 3.6
        self.wp_left_width = rospy.get_param("~wp_left_width", 1.4)
        self.wp_right_width = rospy.get_param("~wp_right_width", 1.4)

        self.coordinate_transformer = rospy.get_param("/localization/coordinate_transformer")
        self.use_custom_origin = rospy.get_param("/localization/use_custom_origin")
        self.utm_origin_lat = rospy.get_param("/localization/utm_origin_lat")
        self.utm_origin_lon = rospy.get_param("/localization/utm_origin_lon")

        # Internal variables
        self.current_location = None
        self.goal_point = None
        self.waypoints = []
        self.Lanelet2_map = None

        # Load lanelet map
        if self.coordinate_transformer == "utm" and self.use_custom_origin:
                projector = UtmProjector(Origin(self.utm_origin_lat, self.utm_origin_lon))
        else:
            rospy.logfatal("lanelet2_global_planner - only utm and custom origin currently supported for lanelet2 map loading")
            exit(1)

        self.lanelet2_map = load(self.lanelet2_map_name, projector)

        # traffic rules
        self.traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                  lanelet2.traffic_rules.Participants.Vehicle)

        #Publishers
        self.waypoints_pub = rospy.Publisher('global_path', Lane, queue_size=1, latch=True)
        self.target_lane_pub = rospy.Publisher('target_lane_markers', MarkerArray, queue_size=1, latch=True)

        #Subscribers
        self.sub = rospy.Subscriber('goal', PoseStamped, self.goal_callback, queue_size=1)
        self.sub = rospy.Subscriber('current_pose', PoseStamped, self.current_pose_callback, queue_size=1)
        self.sub = rospy.Subscriber('cancel_global_path', Bool, self.cancel_global_path_callback, queue_size=1)

    def goal_callback(self, msg):
        rospy.loginfo("lanelet2_global_planner - goal position (%f, %f, %f) orientation (%f, %f, %f, %f) in %s frame",
                    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                    msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,
                    msg.pose.orientation.w, msg.header.frame_id)

        if self.current_location == None:
            # TODO handle if current_pose gets lost at later stage - see current_pose_callback
            rospy.logwarn("lanelet2_global_planner - current_pose not available")
            return

        if self.lanelet2_map == None:
            rospy.logwarn("lanelet2_global_planner - lanelet2 map not available")
            return

        # if there is already a goal, use it as start point
        start_point = self.goal_point if self.goal_point else self.current_location
        new_goal = BasicPoint2d(msg.pose.position.x, msg.pose.position.y)

        # Get nearest lanelets
        goal_lanelet = self.lanelet2_map.laneletLayer.nearest(new_goal, 1)[0]
        start_lanelet = self.lanelet2_map.laneletLayer.nearest(start_point, 1)[0]
        self.publish_target_lanelets(start_lanelet, goal_lanelet)

        graph = lanelet2.routing.RoutingGraph(self.lanelet2_map, self.traffic_rules)
        route = graph.getRoute(start_lanelet, goal_lanelet, 0, True)        # lanelet2.routing.Route
        if route == None:
            rospy.logwarn("lanelet2_global_planner - no route found, try new goal!")
            return

        path = route.shortestPath()
        path_no_lane_change = path.getRemainingLane(start_lanelet)

        # check if goal is in path
        if path_no_lane_change[len(path_no_lane_change)-1].id != goal_lanelet.id:
            rospy.logwarn("lanelet2_global_planner - last lanelet in path (%d) is not goal lanelet (%d)", path_no_lane_change[len(path_no_lane_change)-1].id, goal_lanelet.id)
            return

        waypoints, waypoint_tree = self.convert_to_waypoints(path_no_lane_change)

        # find closest point idx for start and goal
        d, start_idx = waypoint_tree.query([(start_point.x, start_point.y)], 1)
        if d[0][0] > self.distance_to_centerline_limit:
            rospy.logwarn("lanelet2_global_planner - start point too far (%f) from centerline", d[0][0])
            return

        d, goal_idx = waypoint_tree.query([(new_goal.x, new_goal.y)], 1)
        if d[0][0] > self.distance_to_centerline_limit:
            rospy.logwarn("lanelet2_global_planner - goal point too far (%f) from centerline", d[0][0])
            return

        # update goal point and add new waypoints to the existing ones
        self.goal_point = new_goal
        self.waypoints += waypoints[start_idx[0][0]:goal_idx[0][0]]

        self.publish_waypoints(self.waypoints)
        rospy.loginfo("lanelet2_global_planner - path published")


    def current_pose_callback(self, msg):
        self.current_location = BasicPoint2d(msg.pose.position.x, msg.pose.position.y)

    def cancel_global_path_callback(self, msg):
        if msg.data:
            self.waypoints = []
            self.goal_point = None
            self.publish_waypoints(self.waypoints)
            rospy.logwarn("lanelet2_global_planner - global path cancelled!")

    def convert_to_waypoints(self, lanelet_sequence):
        waypoints = []

        last_lanelet = False

        for i , lanelet in enumerate(lanelet_sequence):
            blinker = LANELET_TURN_DIRECTION_TO_WAYPOINT_STATE_MAP[lanelet.attributes['turn_direction']]

            if i == len(lanelet_sequence)-1:
                last_lanelet = True

            speed = self.speed_limit
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
                x, y, z, w = quaternion_from_euler(0, 0, heading)
                waypoint.pose.pose.orientation.x = x
                waypoint.pose.pose.orientation.y = y
                waypoint.pose.pose.orientation.z = z
                waypoint.pose.pose.orientation.w = w

                waypoint.twist.twist.linear.x = speed
                waypoint.dtlane.lw = self.wp_left_width
                waypoint.dtlane.rw = self.wp_right_width

                waypoints.append(waypoint)

        # build KDTree for nearest neighbour search
        waypoints_xy = np.array([(w.pose.pose.position.x, w.pose.pose.position.y) for w in waypoints])
        waypoint_tree = KDTree(waypoints_xy)

        return waypoints, waypoint_tree


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