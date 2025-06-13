#!/usr/bin/env python3

import math
import rospy
import numpy as np

from autoware_mini.msg import DetectedObjectArray
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray 

from autoware_mini.geometry import get_orientation_from_heading

class DetectedObjectsVisualizer:
    def __init__(self):

        self.published_ids = set()

        self.markers_pub = rospy.Publisher('detected_objects_markers', MarkerArray, queue_size=1, tcp_nodelay=True)
        self.bboxes_pub = rospy.Publisher('detected_objects_bboxes', BoundingBoxArray, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('detected_objects', DetectedObjectArray, self.objects_callback, queue_size=1, buff_size=2**20, tcp_nodelay=True)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def objects_callback(self, msg):
        new_published_ids = set()
        markers = MarkerArray()
        bboxes = BoundingBoxArray(header=msg.header)
        for obj in msg.objects:
            # centroid
            marker = Marker(header=msg.header)
            marker.ns = 'centroid'
            marker.id = obj.id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = obj.centroid
            marker.pose.orientation = get_orientation_from_heading(obj.heading)
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color = obj.color
            markers.markers.append(marker)
            
            # bounding box
            marker = Marker(header=msg.header)
            marker.ns = 'bounding_box'
            marker.id = obj.id
            marker.type = marker.LINE_STRIP
            marker.action = marker.ADD
            marker.pose.position = obj.center
            marker.pose.orientation = get_orientation_from_heading(obj.heading)
            marker.scale.x = 0.1
            marker.color = ColorRGBA(1.0, 0.0, 0.0, 0.8)
            half_length = obj.dimensions.x / 2.0
            half_width = obj.dimensions.y / 2.0
            marker.points = [
               Point(-half_length, -half_width, 0.0),
               Point(-half_length, half_width, 0.0),
               Point(half_length, half_width, 0.0),
               Point(half_length, -half_width, 0.0),
               Point(-half_length, -half_width, 0.0),
            ]
            markers.markers.append(marker)

            # convex hull
            if len(obj.convex_hull) > 0:
                marker = Marker(header=msg.header)
                marker.ns = 'convex_hull'
                marker.id = obj.id
                marker.type = marker.LINE_STRIP
                marker.action = marker.ADD
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.1
                marker.color = ColorRGBA(0.0, 1.0, 0.0, 0.8)
                marker.points = [Point(x, y, z) for x, y, z in np.array(obj.convex_hull).reshape(-1, 3)]
                marker.points.append(marker.points[0])
                markers.markers.append(marker)

            # speed arrow
            marker = Marker(header=msg.header)
            marker.ns = 'speed'
            marker.id = obj.id
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position = obj.centroid
            heading = math.atan2(obj.velocity.y, obj.velocity.x)
            marker.pose.orientation = get_orientation_from_heading(heading)
            marker.scale.x = max(math.sqrt(obj.velocity.x**2 + obj.velocity.y**2), 0.01)
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color = ColorRGBA(1.0, 1.0, 0.0, 1.0)
            markers.markers.append(marker)

            # text
            marker = Marker(header=msg.header)
            marker.ns = 'text'
            marker.id = obj.id
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position = Point(obj.centroid.x, obj.centroid.y, obj.centroid.z + 1.0)
            marker.scale.z = 0.5
            marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
            marker.text = "%s %d (%d km/h)" % (obj.label, obj.id, math.sqrt(obj.velocity.x**2 + obj.velocity.y**2 + obj.velocity.z**2) * 3.6)
            markers.markers.append(marker)

            new_published_ids.add(obj.id)

            # 3D bounding box
            bbox = BoundingBox(header=msg.header)
            bbox.pose.position = obj.center
            bbox.pose.orientation = get_orientation_from_heading(obj.heading)
            bbox.dimensions = obj.dimensions
            bbox.label = obj.id
            bboxes.boxes.append(bbox)

        # delete ids not published any more
        delete_ids = self.published_ids - new_published_ids
        for id in delete_ids:
            marker = Marker(header=msg.header)
            marker.ns = 'centroid'
            marker.id = id
            marker.action = marker.DELETE
            markers.markers.append(marker)

            marker = Marker(header=msg.header)
            marker.ns = 'bounding_box'
            marker.id = id
            marker.action = marker.DELETE
            markers.markers.append(marker)

            marker = Marker(header=msg.header)
            marker.ns = 'convex_hull'
            marker.id = id
            marker.action = marker.DELETE
            markers.markers.append(marker)

            marker = Marker(header=msg.header)
            marker.ns = 'speed'
            marker.id = id
            marker.action = marker.DELETE
            markers.markers.append(marker)

            marker = Marker(header=msg.header)
            marker.ns = 'text'
            marker.id = id
            marker.action = marker.DELETE
            markers.markers.append(marker)
        self.published_ids = new_published_ids

        # publish markers
        self.markers_pub.publish(markers)
        self.bboxes_pub.publish(bboxes)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('detected_objects_visualizer', log_level=rospy.INFO)
    node = DetectedObjectsVisualizer()
    node.run()
