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
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA

LANELET_TURN_DIRECTION_TO_WAYPOINT_STATE_MAP = {
    "straight": WaypointState.STR_STRAIGHT,
    "left": WaypointState.STR_LEFT,
    "right": WaypointState.STR_RIGHT
}

class Lanelet2GlobalPlanner:
    
    def __init__(self):

        # Parameters
        self.map_file = rospy.get_param("~map_file", "/home/edgar/workspaces/autoware_ut/src/autoware_ut/maps/tartu_demo_route/lanelet2/tartu_demo_l_c_wp_dem.osm")
        self.distance_to_centerline_limit = rospy.get_param("~distance_to_centerline_limit", 3.0)

        # Internal variables
        self.start_point = None
        self.map = None
        self.output_frame = "map"

        # traffic rules
        self.traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                  lanelet2.traffic_rules.Participants.Vehicle)
        
        #Subscribers
        #self.sub = rospy.Subscriber('lanelet_map_bin', MapBin, self.map_callback, queue_size=1)
        self.sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback, queue_size=1)
        self.sub = rospy.Subscriber('current_pose', PoseStamped, self.current_pose_callback, queue_size=1)

        #Publishers
        self.waypoints_pub = rospy.Publisher('path', Lane, queue_size=1, latch=True)
        self.waypoints_markers_pub = rospy.Publisher('path_markers', MarkerArray, queue_size=1, latch=True)

        # Load lanelet map - TODO: should be replaced by loading from the topic
        projector = UtmProjector(Origin(58.385345, 26.726272))
        self.map = load(self.map_file, projector)

    def goal_callback(self, msg):

        rospy.loginfo("lanelet2_global_planner - goal point received")

        if self.start_point == None:
            # TODO handle if current_pose gets lost at later stage
            rospy.logwarn("lanelet2_global_planner - current_pose not available")
            return
        
        if self.map == None:
            rospy.logwarn("lanelet2_global_planner - lanelet2 map not available")
            return

        goal_point = BasicPoint2d(msg.pose.position.x, msg.pose.position.y)

        # Get nearest lanelets
        goal_lanelet = self.map.laneletLayer.nearest(goal_point, 1)[0]
        start_lanelet = self.map.laneletLayer.nearest(self.start_point, 1)[0]

        graph = lanelet2.routing.RoutingGraph(self.map, self.traffic_rules)
        # TODO this receives quite often NoneType object, although everything should be fine (sparse waypoints?)
        route = graph.getRoute(start_lanelet, goal_lanelet, 0, True)                 # lanelet2.routing.Route
        path = route.shortestPath()                                         # lanelet2.routing.LaneletPath - might be useful for lane change functionality?
        path_no_lane_change = path.getRemainingLane(start_lanelet)          # lanelet2.core.LaneletSequence

        waypoints, waypoint_tree = self.convert_to_waypoints(path_no_lane_change)
        
        # Check start and goal point distances from centerline and clip the path
        # TODO add also check for heading error - if too big warn and don't create the path
        d, start_idx = waypoint_tree.query([(self.start_point.x, self.start_point.y)], 1)
        if d[0][0] > self.distance_to_centerline_limit:
            rospy.logwarn("lanelet2_global_planner - start point too far from closest lanelet centerline")
            return
        
        # TODO this hardly happens, if placed too far laneletLayer.nearest won't return the nearest lanelet?
        d, goal_idx = waypoint_tree.query([(goal_point.x, goal_point.y)], 1)
        if d[0][0] > self.distance_to_centerline_limit:
            rospy.logwarn("lanelet2_global_planner - goal point too far from closest lanelet centerline")
            return

        self.publish_waypoints(waypoints[start_idx[0][0]:goal_idx[0][0]])
        self.publish_waypoints_markers(waypoints[start_idx[0][0]:goal_idx[0][0]])

        rospy.loginfo("lanelet2_global_planner - shortest path published")

    def current_pose_callback(self, msg):
        self.start_point = BasicPoint2d(msg.pose.position.x, msg.pose.position.y)

        # TODO add timing, if not available for too long, set current_pose_available to false
        # use it later in goal_callback to check if current_pose is available


    def convert_to_waypoints(self, lanelet_sequence):

        waypoints = []
        wp_id = 0

        for lanelet in lanelet_sequence:
            blinker = LANELET_TURN_DIRECTION_TO_WAYPOINT_STATE_MAP[lanelet.attributes['turn_direction']]

            for point in lanelet.centerline:
                waypoint = Waypoint()
                waypoint.pose.pose.position.x = point.x
                waypoint.pose.pose.position.y = point.y
                waypoint.pose.pose.position.z = point.z
                waypoint.wpstate.steering_state = blinker
                # TODO: add speeds to lanelets (speed_limit) or use traffic rules. For now, use constant speed
                # Later curvature based speed adjustment
                waypoint.twist.twist.linear.x = 10.0
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

    # TODO Exact copy from waypoint_loader - split to separate node waypoint_visualizer and remove from here
    def publish_waypoints_markers(self, waypoints):
        marker_array = MarkerArray()

        # Pose arrows
        for i, waypoint in enumerate(waypoints):

            # color the arrows based on the waypoint steering_flag (blinker)
            if waypoint.wpstate.steering_state == WaypointState.STR_LEFT:
                color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
            elif waypoint.wpstate.steering_state == WaypointState.STR_RIGHT:
                color = ColorRGBA(0.0, 0.0, 1.0, 1.0)
            else:
                color = ColorRGBA(0.0, 1.0, 0.0, 1.0)

            marker = Marker()
            marker.header.frame_id = self.output_frame
            marker.header.stamp = rospy.Time.now()
            marker.ns = "Waypoint pose"
            marker.id = i
            marker.type = marker.ARROW
            marker.action = marker.ADD
            marker.pose = waypoint.pose.pose
            marker.scale.x = 0.4
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color = color
            marker_array.markers.append(marker)

        # velocity labels
        for i, waypoint in enumerate(waypoints):
            marker = Marker()
            marker.header.frame_id = self.output_frame
            marker.header.stamp = rospy.Time.now()
            marker.ns = "Velocity label"
            marker.id = i
            marker.type = marker.TEXT_VIEW_FACING
            marker.action = marker.ADD
            marker.pose = waypoint.pose.pose
            marker.scale.z = 0.5
            marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
            marker.text = str(round(waypoint.twist.twist.linear.x * 3.6, 1))
            marker_array.markers.append(marker)

        # line strips
        marker = Marker()
        marker.header.frame_id = self.output_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "Path"
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.id = 0
        marker.scale.x = 0.5
        marker.color = ColorRGBA(0.4, 10, 1.0, 0.4)
        for waypoint in waypoints:
            marker.points.append(waypoint.pose.pose.position)
        marker_array.markers.append(marker)

        # publish markers
        self.waypoints_markers_pub.publish(marker_array)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('lanelet2_global_planner')
    node = Lanelet2GlobalPlanner()
    node.run()