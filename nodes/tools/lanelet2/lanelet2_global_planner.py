#!/usr/bin/env python

import rospy
import lanelet2
from lanelet2.io import Origin, load
from lanelet2.projection import UtmProjector
from lanelet2.core import BasicPoint2d

from geometry_msgs.msg import PoseStamped


class Lanelet2GlobalPlanner:
    
    def __init__(self):

        # traffic rules
        self.traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                  lanelet2.traffic_rules.Participants.Vehicle)
        
        #Subscribers
        #self.sub = rospy.Subscriber('lanelet_map_bin', MapBin, self.map_callback, queue_size=1)
        self.sub = rospy.Subscriber('move_base_simple/goal', PoseStamped, self.goal_callback, queue_size=1)
        self.sub = rospy.Subscriber('current_pose', PoseStamped, self.current_pose_callback, queue_size=1)

        #Publishers
        #TODO publish path

        # Load lanelet map - TODO: should be replaced by loading from the topic
        projector = UtmProjector(Origin(58.385345, 26.726272))
        self.map = load("/home/edgar/car/lanelet_maps/Valentina/test_lai_dem.osm", projector)

    def goal_callback(self, msg):

        goal_point = BasicPoint2d(msg.pose.position.x, msg.pose.position.y)

        # Get nearest lanelet
        goal_lanelet = self.map.laneletLayer.nearest(goal_point, 1)[0]

        # TESTing
        start_point = BasicPoint2d(-529.905, -126.986)
        start_lanelet = self.map.laneletLayer.nearest(start_point, 1)[0]


        graph = lanelet2.routing.RoutingGraph(self.map, self.traffic_rules)
        route = graph.getRoute(start_lanelet, goal_lanelet)
        path = route.shortestPath()                                         # lanelet2.routing.LaneletPath
        path_no_lane_chaneg = path.getRemainingLane(start_lanelet)          # lanelet2.core.LaneletSequence


    def current_pose_callback(self, msg):

        start_point = BasicPoint2d(msg.pose.position.x, msg.pose.position.y)




    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('lanelet2_global_planner')
    node = Lanelet2GlobalPlanner()
    node.run()