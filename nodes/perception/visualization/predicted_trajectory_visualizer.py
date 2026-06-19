#!/usr/bin/env python3

import rospy
import shapely
from collections import defaultdict

from autoware_mini.msg import DetectedObjectArray
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA

from autoware_mini.visualization import triangulate_linestring

COLOR = ColorRGBA(1.0, 1.0, 0.0, 0.5) # Yellow

class PredictedTrajectoryVisualizer:
    def __init__(self):

        # Create pools of markers to be reused
        self.trajectory_markers = defaultdict(Marker)
        self.delete_trajectory_markers = defaultdict(Marker)

        self.published_ids = set()

        self.markers_pub = rospy.Publisher('predicted_objects_markers', MarkerArray, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('predicted_objects', DetectedObjectArray, self.objects_callback, queue_size=1, buff_size=2**20, tcp_nodelay=True)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def objects_callback(self, msg):
        new_published_ids = set()

        # Visualize future trajectories of all predicted objects
        markers = MarkerArray()
        for i, obj in enumerate(msg.objects):
            marker = self.trajectory_markers[i]
            if not marker.ns:
                # Create triangle list marker
                marker.ns = 'candidate_trajectories'
                marker.type = Marker.TRIANGLE_LIST
                marker.color = COLOR
                marker.scale.x = 1.0
                marker.scale.y = 1.0
                marker.scale.z = 1.0
                marker.pose.orientation.w = 1.0
                marker.lifetime = rospy.Duration(0.3)
            marker.header = msg.header
            marker.id = obj.id

            if not obj.candidate_trajectories.paths:
                marker.action = marker.DELETE
            else:
                marker.action = marker.ADD
                marker.points = []
                # visualize possible multiple trajectories
                for lane in obj.candidate_trajectories.paths:
                    linestring = shapely.linestrings([[p.position.x, p.position.y, p.position.z] for p in lane.waypoints])
                    triangle_points = triangulate_linestring(linestring, obj.dimensions.y)
                    marker.points.extend(triangle_points)
                marker.colors = [COLOR] * len(marker.points)

            markers.markers.append(marker)
            new_published_ids.add(obj.id)

        # Delete ids not published any more
        delete_ids = self.published_ids - new_published_ids
        for i, id in enumerate(delete_ids):
            marker = self.delete_trajectory_markers[i]
            if not marker.ns:
                marker.ns = 'candidate_trajectories'
                marker.action = marker.DELETE
            marker.header = msg.header
            marker.id = id
            markers.markers.append(marker)

        self.published_ids = new_published_ids

        # Publish markers
        self.markers_pub.publish(markers)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('predicted_trajectory_visualizer', log_level=rospy.INFO)
    node = PredictedTrajectoryVisualizer()
    node.run()
