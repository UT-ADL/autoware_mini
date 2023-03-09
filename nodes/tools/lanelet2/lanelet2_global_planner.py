#!/usr/bin/env python

import rospy
import lanelet2
from lanelet2.io import Origin, load
from lanelet2.projection import UtmProjector
from lanelet2.core import BasicPoint2d

from geometry_msgs.msg import PoseStamped
from autoware_msgs.msg import Lane, Waypoint, WaypointState
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA


class Lanelet2GlobalPlanner:
    
    def __init__(self):

        # Parameters
        self.map_file = rospy.get_param("~map_file", "/home/edgar/car/lanelet_maps/Valentina/test_lai_dem.osm")

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

        print("Goal entered!")
        if self.start_point == None:
            # TODO handle if current_pose gets lost at later stage
            rospy.logwarn("lanelet2_global_planner - current_pose not available")
            return
        
        if self.map == None:
            rospy.logwarn("lanelet2_global_planner - lanelet2 map not available")
            return

        goal_point = BasicPoint2d(msg.pose.position.x, msg.pose.position.y)

        # Get nearest lanelet
        goal_lanelet = self.map.laneletLayer.nearest(goal_point, 1)[0]

        # TESTing
        # start_point = BasicPoint2d(-529.905, -126.986)
        start_lanelet = self.map.laneletLayer.nearest(self.start_point, 1)[0]


        graph = lanelet2.routing.RoutingGraph(self.map, self.traffic_rules)
        route = graph.getRoute(start_lanelet, goal_lanelet)
        path = route.shortestPath()                                         # lanelet2.routing.LaneletPath
        path_no_lane_change = path.getRemainingLane(start_lanelet)          # lanelet2.core.LaneletSequence

        waypoints = self.convert_to_waypoints(path_no_lane_change)
        self.publish_waypoints(waypoints)
        self.publish_waypoints_markers(waypoints)


    def current_pose_callback(self, msg):
        self.start_point = BasicPoint2d(msg.pose.position.x, msg.pose.position.y)


    def convert_to_waypoints(self, lanelet_sequence):

        # extract centerlines - waypoint coordinates
        centerlines = [lanelet.centerline for lanelet in lanelet_sequence]

        waypoints = []
        wp_id = 0

        # iterate over centerline and create waypoints 
        for line in centerlines:    # lanelet2.core.ConstLineString3d
            for point in line:

                # TODO - need to make the path dense
                waypoint = Waypoint()

                waypoint.gid = wp_id
                waypoint.pose.pose.position.x = point.x
                waypoint.pose.pose.position.y = point.y
                waypoint.pose.pose.position.z = point.z

                # TODO - blinkers
                waypoint.wpstate.steering_state = 0
                # TODO - speed - curvature or traffic rules based?
                waypoint.twist.twist.linear.x = 5

                wp_id += 1

                waypoints.append(waypoint)
        
        return waypoints


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