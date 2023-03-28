#!/usr/bin/env python

import rospy
import lanelet2
import numpy as np
from lanelet2.io import Origin, load
from lanelet2.projection import UtmProjector
from lanelet2.core import BasicPoint2d
from lanelet2.geometry import distance, to2D

from sklearn.neighbors import KDTree

from geometry_msgs.msg import PoseStamped
from autoware_msgs.msg import Lane, Waypoint, WaypointState
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_euler
from helpers import get_heading_between_two_points

LANELET_TURN_DIRECTION_TO_WAYPOINT_STATE_MAP = {
    "straight": WaypointState.STR_STRAIGHT,
    "left": WaypointState.STR_LEFT,
    "right": WaypointState.STR_RIGHT
}

class Lanelet2GlobalPlanner:
    
    def __init__(self):

        # Parameters
        self.lanelet2_map_name = rospy.get_param("~lanelet2_map_name")
        self.output_frame = rospy.get_param("~output_frame", map)
        self.distance_to_centerline_limit = rospy.get_param("~distance_to_centerline_limit", 5.0)
        self.speed_limit = rospy.get_param("~speed_limit", 40.0) / 3.6

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
        self.waypoints_pub = rospy.Publisher('path', Lane, queue_size=1, latch=True)

        #Subscribers
        self.sub = rospy.Subscriber('goal', PoseStamped, self.goal_callback, queue_size=1)
        self.sub = rospy.Subscriber('current_pose', PoseStamped, self.current_pose_callback, queue_size=1)
        self.sub = rospy.Subscriber('cancel_global_path', Bool, self.cancel_global_path_callback, queue_size=1)

    def goal_callback(self, msg):

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

        if distance(start_point, to2D(start_lanelet.centerline)) > self.distance_to_centerline_limit:
            rospy.logwarn("lanelet2_global_planner - start point too far from centerline")
            return
        
        if distance(new_goal, to2D(goal_lanelet.centerline)) > self.distance_to_centerline_limit:
            rospy.logwarn("lanelet2_global_planner - goal point too far from centerline")
            return

        graph = lanelet2.routing.RoutingGraph(self.lanelet2_map, self.traffic_rules)
        route = graph.getRoute(start_lanelet, goal_lanelet, 0, True)        # lanelet2.routing.Route
        if route == None:
            rospy.logwarn("lanelet2_global_planner - no route found, try new goal!")
            return
        
        path = route.shortestPath()                                         # lanelet2.routing.LaneletPath - might be useful for lane change functionality?
        path_no_lane_change = path.getRemainingLane(start_lanelet)          # lanelet2.core.LaneletSequence
        new_waypoints, new_waypoint_tree = self.convert_to_waypoints(path_no_lane_change)

        # find closest point idx for start and goal
        _, start_idx = new_waypoint_tree.query([(start_point.x, start_point.y)], 1)
        _, goal_idx = new_waypoint_tree.query([(new_goal.x, new_goal.y)], 1)

        # update goal point and add new waypoints to the existing ones
        self.goal_point = new_goal
        self.waypoints += new_waypoints[start_idx[0][0]:goal_idx[0][0]]

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

                # TODO: will be replaced by speed from map
                waypoint.twist.twist.linear.x = self.speed_limit

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


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('lanelet2_global_planner')
    node = Lanelet2GlobalPlanner()
    node.run()