#!/usr/bin/env python

import rospy
from lanelet2_map_visualizer import draw_lanelets
import lanelet2
from lanelet2.io import Origin, loadRobust
from lanelet2.projection import UtmProjector

from visualization_msgs.msg import MarkerArray

#from autoware_lanelet2_msgs.msg import MapBin

class Lanelet2MapLoader:

    def __init__(self):
    
        # Parameters
        self.map_file = rospy.get_param("~map_file", "/home/edgar/workspaces/autoware_ut/src/autoware_ut/maps/tartu_demo_route/lanelet2/tartu_demo_l_c_wp_dem.osm")

        # create MarkerArray publisher
        self.markers_pub = rospy.Publisher('lanelet2_map_viz', MarkerArray, queue_size=1, latch=True)

        # Load the map using Lanelet2
        projector = UtmProjector(Origin(58.385345, 26.726272))
        map, errors = loadRobust(self.map_file, projector)

        if len(errors) > 0:
            rospy.logwarn("lanelet2_map_loader - Errors while loading map")
            # TODO: errors could be printed, parsed or summarized somehow (counted)
        
        
        rospy.loginfo("lanelet2_map_loader - map loaded with %i lanelets and %i regulatory elements" % (len(map.laneletLayer), len(map.regulatoryElementLayer)))

        # TODO: not supported in python? Convert the Lanelet2 map to a MapBin message
        # map_msg = MapBin()
        # map_msg.map = map.toBinMsg()
        # Publish the MapBin message as a latching message
        # map_pub = rospy.Publisher('lanelet_map_bin', MapBin, queue_size=1, latch=True)
        # map_pub.publish(map_msg)
        # rospy.loginfo("lanelet2_map_loader - Published MapBin message to the 'lanelet_map_bin' topic")

        # Test drawing the lanelets
        markers = draw_lanelets(map)
        self.markers_pub.publish(markers)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('lanelet2_map_loader')
    node = Lanelet2MapLoader()
    node.run()
