#!/usr/bin/env python3

import math
import rospy
import numpy as np
from collections import defaultdict

from autoware_mini.msg import DetectedObjectArray
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from autoware_mini.geometry import get_orientation_from_heading

BBOX_COLOR = ColorRGBA(1.0, 0.0, 0.0, 0.8)
BBOX_3D_COLOR = ColorRGBA(1.0, 0.0, 0.0, 0.3)
CONVEX_HULL_COLOR = ColorRGBA(0.0, 1.0, 0.0, 0.8)
SPEED_COLOR = ColorRGBA(1.0, 1.0, 0.0, 1.0)
TEXT_COLOR = ColorRGBA(1.0, 1.0, 1.0, 1.0)

class DetectedObjectsVisualizer:
    def __init__(self):

        # Create pools of markers to be reused
        self.centroid_markers = defaultdict(Marker)
        self.bbox_markers = defaultdict(Marker)
        self.convex_hull_markers = defaultdict(Marker)
        self.speed_markers = defaultdict(Marker)
        self.text_markers = defaultdict(Marker)
        self.bbox_3d_markers = defaultdict(Marker)

        self.delete_centroid_markers = defaultdict(Marker)
        self.delete_bbox_markers = defaultdict(Marker)
        self.delete_convex_hull_markers = defaultdict(Marker)
        self.delete_speed_markers = defaultdict(Marker)
        self.delete_text_markers = defaultdict(Marker)
        self.delete_bbox_3d_markers = defaultdict(Marker)

        self.published_ids = set()

        self.markers_pub = rospy.Publisher('detected_objects_markers', MarkerArray, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('detected_objects', DetectedObjectArray, self.objects_callback, queue_size=1, buff_size=2**20, tcp_nodelay=True)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def objects_callback(self, msg):
        new_published_ids = set()
        markers = MarkerArray()
        for i, obj in enumerate(msg.objects):
            orientation = get_orientation_from_heading(obj.heading)

            # centroid
            marker = self.centroid_markers[i]
            if not marker.ns:
                marker.ns = 'centroid'
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.scale.x = 0.5
                marker.scale.y = 0.5
                marker.scale.z = 0.5
                marker.lifetime = rospy.Duration(0.3)
            marker.header = msg.header
            marker.id = obj.id
            marker.pose.position.x = obj.centroid.x
            marker.pose.position.y = obj.centroid.y
            marker.pose.position.z = obj.centroid.z
            marker.pose.orientation = orientation
            marker.color = obj.color
            markers.markers.append(marker)

            # bounding box
            marker = self.bbox_markers[i]
            if not marker.ns:
                marker.ns = 'bounding_box'
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.scale.x = 0.1
                marker.color = BBOX_COLOR
                marker.lifetime = rospy.Duration(0.3)
            marker.header = msg.header
            marker.id = obj.id
            marker.pose.position.x = obj.center.x
            marker.pose.position.y = obj.center.y
            marker.pose.position.z = obj.center.z
            marker.pose.orientation = orientation
            half_length = obj.dimensions.x / 2.0
            half_width = obj.dimensions.y / 2.0
            marker.points = [
               Point(x=0.0, y=0.0, z=0.0),
               Point(x=half_length, y=0.0, z=0.0),
               Point(x=half_length, y=half_width, z=0.0),
               Point(x=-half_length, y=half_width, z=0.0),
               Point(x=-half_length, y=-half_width, z=0.0),
               Point(x=half_length, y=-half_width, z=0.0),
               Point(x=half_length, y=0.0, z=0.0),
            ]
            markers.markers.append(marker)

            # convex hull
            if obj.convex_hull:
                marker = self.convex_hull_markers[i]
                if not marker.ns:
                    marker.ns = 'convex_hull'
                    marker.type = marker.LINE_STRIP
                    marker.action = marker.ADD
                    marker.pose.orientation.w = 1.0
                    marker.scale.x = 0.1
                    marker.color = CONVEX_HULL_COLOR
                    marker.lifetime = rospy.Duration(0.3)
                marker.header = msg.header
                marker.id = obj.id
                marker.points = [Point(x=x, y=y, z=z) for x, y, z in np.array(obj.convex_hull).reshape(-1, 3).tolist()]
                marker.points.append(marker.points[0])
                markers.markers.append(marker)

            # speed arrow
            marker = self.speed_markers[i]
            if not marker.ns:
                marker.ns = 'speed'
                marker.type = Marker.ARROW
                marker.action = Marker.ADD
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.color = SPEED_COLOR
                marker.lifetime = rospy.Duration(0.3)
            marker.header = msg.header
            marker.id = obj.id
            marker.pose.position.x = obj.centroid.x
            marker.pose.position.y = obj.centroid.y
            marker.pose.position.z = obj.centroid.z
            heading = math.atan2(obj.velocity.y, obj.velocity.x)
            marker.pose.orientation = get_orientation_from_heading(heading)
            marker.scale.x = max(math.sqrt(obj.velocity.x**2 + obj.velocity.y**2), 0.01)
            markers.markers.append(marker)

            # text
            marker = self.text_markers[i]
            if not marker.ns:
                marker.ns = 'text'
                marker.type = Marker.TEXT_VIEW_FACING
                marker.action = Marker.ADD
                marker.scale.z = 0.5
                marker.color = TEXT_COLOR
                marker.lifetime = rospy.Duration(0.3)
            marker.header = msg.header
            marker.id = obj.id
            marker.pose.position.x = obj.centroid.x
            marker.pose.position.y = obj.centroid.y
            marker.pose.position.z = obj.centroid.z + 1.0
            marker.text = "%s %d (%d km/h)" % (obj.label, obj.id, math.sqrt(obj.velocity.x**2 + obj.velocity.y**2 + obj.velocity.z**2) * 3.6)
            markers.markers.append(marker)

            new_published_ids.add(obj.id)

            # 3D bounding box
            marker = self.bbox_3d_markers[i]
            if not marker.ns:
                marker.ns = 'bbox_3d'
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.color = BBOX_3D_COLOR
                marker.lifetime = rospy.Duration(0.3)
            marker.header = msg.header
            marker.id = obj.id
            marker.pose.position.x = obj.center.x
            marker.pose.position.y = obj.center.y
            marker.pose.position.z = obj.center.z
            marker.pose.orientation = orientation
            marker.scale = obj.dimensions
            markers.markers.append(marker)

        # delete ids not published any more
        delete_ids = self.published_ids - new_published_ids
        for i, id in enumerate(delete_ids):
            marker = self.delete_centroid_markers[i]
            if not marker.ns:
                marker.ns = 'centroid'
                marker.action = marker.DELETE
            marker.header = msg.header
            marker.id = id
            markers.markers.append(marker)

            marker = self.delete_bbox_markers[i]
            if not marker.ns:
                marker.ns = 'bounding_box'
                marker.action = marker.DELETE
            marker.header = msg.header
            marker.id = id
            markers.markers.append(marker)

            marker = self.delete_convex_hull_markers[i]
            if not marker.ns:
                marker.ns = 'convex_hull'
                marker.action = marker.DELETE
            marker.header = msg.header
            marker.id = id
            markers.markers.append(marker)

            marker = self.delete_speed_markers[i]
            if not marker.ns:
                marker.ns = 'speed'
                marker.action = marker.DELETE
            marker.header = msg.header
            marker.id = id
            markers.markers.append(marker)

            marker = self.delete_text_markers[i]
            if not marker.ns:
                marker.ns = 'text'
                marker.action = marker.DELETE
            marker.header = msg.header
            marker.id = id
            markers.markers.append(marker)

            marker = self.delete_bbox_3d_markers[i]
            if not marker.ns:
                marker.ns = 'bbox_3d'
                marker.action = marker.DELETE
            marker.header = msg.header
            marker.id = id
            markers.markers.append(marker)
        self.published_ids = new_published_ids

        # publish markers
        self.markers_pub.publish(markers)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('detected_objects_visualizer', log_level=rospy.INFO)
    node = DetectedObjectsVisualizer()
    node.run()
