#!/usr/bin/env python

import rospy
from lanelet2_map_visualizer import visualize_lanelet2_map
import lanelet2
from lanelet2.io import Origin, loadRobust
from lanelet2.projection import UtmProjector

from visualization_msgs.msg import MarkerArray

#from autoware_lanelet2_msgs.msg import MapBin

class Lanelet2MapLoader:

    def __init__(self):
    
        # Parameters
        self.lanelet2_map = rospy.get_param("~lanelet2_map")

        # create MarkerArray publisher
        self.markers_pub = rospy.Publisher('lanelet2_map_markers', MarkerArray, queue_size=1, latch=True)

        # Load the map using Lanelet2
        projector = UtmProjector(Origin(58.385345, 26.726272))
        map, errors = loadRobust(self.lanelet2_map, projector)

        if len(errors) > 0:
            rospy.logwarn("lanelet2_map_loader - Errors while loading map")
            # TODO: errors could be printed, parsed or summarized somehow (counted)

        # Visualize the Lanelet2 map
        marker_array = visualize_lanelet2_map(map)
        self.markers_pub.publish(marker_array)

        rospy.loginfo("lanelet2_map_loader - map loaded with %i lanelets and %i regulatory elements from file: %s" % 
                      (len(map.laneletLayer), len(map.regulatoryElementLayer), str(self.lanelet2_map)))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('lanelet2_map_loader')
    node = Lanelet2MapLoader()
    node.run()
