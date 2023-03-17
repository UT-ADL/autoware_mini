#!/usr/bin/env python

import rospy
import lanelet2
import numpy as np
from lanelet2.io import Origin, load
from lanelet2.projection import UtmProjector
from lanelet2.core import BasicPoint2d

from sklearn.neighbors import KDTree

from geometry_msgs.msg import PoseStamped
from autoware_msgs.msg import Lane, Waypoint, WaypointState
from std_msgs.msg import Bool

LANELET_TURN_DIRECTION_TO_WAYPOINT_STATE_MAP = {
    "straight": WaypointState.STR_STRAIGHT,
    "left": WaypointState.STR_LEFT,
    "right": WaypointState.STR_RIGHT
}

class Lanelet2GlobalPlanner:
    
    def __init__(self):

        # Parameters
        self.map_file = rospy.get_param("~lanelet2_map")
        self.distance_to_centerline_limit = rospy.get_param("~distance_to_centerline_limit", 5.0)

        # Internal variables
        self.current_location = None
        self.goal_point = None
        self.waypoints = []
        self.map = None
        self.output_frame = "map"

        # traffic rules
        self.traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                  lanelet2.traffic_rules.Participants.Vehicle)
        
        #Subscribers
        #self.sub = rospy.Subscriber('lanelet_map_bin', MapBin, self.map_callback, queue_size=1)
        self.sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback, queue_size=1)
        self.sub = rospy.Subscriber('current_pose', PoseStamped, self.current_pose_callback, queue_size=1)
        self.sub = rospy.Subscriber('cancel_global_path', Bool, self.cancel_global_path_callback, queue_size=1)

        #Publishers
        self.waypoints_pub = rospy.Publisher('path', Lane, queue_size=1, latch=True)

        # Load lanelet map - TODO: should be replaced by loading from the topic
        projector = UtmProjector(Origin(58.385345, 26.726272))
        self.map = load(self.map_file, projector)

    def goal_callback(self, msg):

        if self.current_location == None:
            # TODO handle if current_pose gets lost at later stage - see current_pose_callback
            rospy.logwarn("lanelet2_global_planner - current_pose not available")
            return

        if self.map == None:
            rospy.logwarn("lanelet2_global_planner - lanelet2 map not available")
            return

        # if there is already a goal, use it as start point
        start_point = self.goal_point if self.goal_point else self.current_location
        new_goal = BasicPoint2d(msg.pose.position.x, msg.pose.position.y)

        # Get nearest lanelets
        goal_lanelet = self.map.laneletLayer.nearest(new_goal, 1)[0]
        start_lanelet = self.map.laneletLayer.nearest(start_point, 1)[0]

        graph = lanelet2.routing.RoutingGraph(self.map, self.traffic_rules)
        route = graph.getRoute(start_lanelet, goal_lanelet, 0, True)        # lanelet2.routing.Route
        if route == None:
            rospy.logwarn("lanelet2_global_planner - no route found, try new goal!")
            return
        
        path = route.shortestPath()                                         # lanelet2.routing.LaneletPath - might be useful for lane change functionality?
        path_no_lane_change = path.getRemainingLane(start_lanelet)          # lanelet2.core.LaneletSequence
        new_waypoints, new_waypoint_tree = self.convert_to_waypoints(path_no_lane_change)

        # Check start and goal point distances from centerline and clip the path
        d, start_idx = new_waypoint_tree.query([(start_point.x, start_point.y)], 1)
        if d[0][0] > self.distance_to_centerline_limit:
            new_waypoints = None
            new_waypoint_tree = None
            rospy.logwarn("lanelet2_global_planner - start point too far from centerline")
            return
        
        d, goal_idx = new_waypoint_tree.query([(new_goal.x, new_goal.y)], 1)
        if d[0][0] > self.distance_to_centerline_limit:
            new_waypoints = None
            new_waypoint_tree = None
            rospy.logwarn("lanelet2_global_planner - goal point too far from centerline")
            return

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
        wp_id = 0

        for lanelet in lanelet_sequence:
            blinker = LANELET_TURN_DIRECTION_TO_WAYPOINT_STATE_MAP[lanelet.attributes['turn_direction']]

            # loop over centerline points use enumerate to get index
            for idx, point in enumerate(lanelet.centerline):
                if idx == 0:
                    # skip first point on every lanelet, because it is the same as the last point of the previous lanelet
                    continue
                waypoint = Waypoint()
                waypoint.pose.pose.position.x = point.x
                waypoint.pose.pose.position.y = point.y
                waypoint.pose.pose.position.z = point.z
                waypoint.wpstate.steering_state = blinker
                # TODO: add speeds to lanelets (speed_limit) or use traffic rules. For now, use constant speed
                # Later curvature based speed adjustment
                waypoint.twist.twist.linear.x = 4.0
                waypoints.append(waypoint)
                wp_id += 1

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