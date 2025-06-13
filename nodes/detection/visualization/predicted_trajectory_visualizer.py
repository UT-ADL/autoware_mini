#!/usr/bin/env python3

import rospy
import shapely

from autoware_mini.msg import DetectedObjectArray
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header, ColorRGBA

from autoware_mini.visualization import triangulate_linestring

COLOR = ColorRGBA(1.0, 1.0, 0.0, 0.5) # Yellow

class PredictedTrajectoryVisualizer:
    def __init__(self):

        self.use_object_width = rospy.get_param('/planning/use_object_width')
        self.published_ids = set()

        self.markers_pub = rospy.Publisher('predicted_objects_markers', MarkerArray, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('predicted_objects', DetectedObjectArray, self.objects_callback, queue_size=1, buff_size=2**20, tcp_nodelay=True)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def objects_callback(self, msg):
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id

        new_published_ids = set()

        # Visualize future trajectories of all predicted objects
        markers = MarkerArray()
        for obj in msg.objects:
            marker = Marker(header=header)
            marker.ns = 'candidate_trajectories'
            marker.id = obj.id
            
            if len(obj.candidate_trajectories.paths) == 0:
                marker.action = marker.DELETE
            else:
                # visualize possible multiple trajectories
                for lane in obj.candidate_trajectories.paths:
                    linestring = shapely.linestrings([[p.position.x, p.position.y, p.position.z] for p in lane.waypoints])

                    triangle_points = triangulate_linestring(linestring, obj.dimensions.y if self.use_object_width else 0.2)
                    marker.points.extend(triangle_points)

                # Create triangle list marker
                marker.type = marker.TRIANGLE_LIST
                marker.action = marker.ADD
                marker.color = COLOR
                marker.scale.x = 1.0
                marker.scale.y = 1.0
                marker.scale.z = 1.0
                marker.pose.orientation.w = 1.0
                marker.colors = [COLOR] * len(marker.points)

            markers.markers.append(marker)
            new_published_ids.add(obj.id)

        # Delete ids not published any more
        delete_ids = self.published_ids - new_published_ids
        for id in delete_ids:
            marker = Marker(header=header)
            marker.ns = 'candidate_trajectories'
            marker.id = id
            marker.action = marker.DELETE
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
