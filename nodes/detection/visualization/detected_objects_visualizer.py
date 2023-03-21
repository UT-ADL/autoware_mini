#!/usr/bin/env python

import math
import rospy

from autoware_msgs.msg import DetectedObjectArray
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import Header, ColorRGBA

class DetectedObjectsVisualizer:
    def __init__(self):
        self.objects_sub = rospy.Subscriber('detected_objects', DetectedObjectArray, self.objects_callback, queue_size=1, buff_size=1024*1024)
        self.markers_pub = rospy.Publisher('detected_objects_markers', MarkerArray, queue_size=1)
        self.published_ids = set()

    def objects_callback(self, msg):
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id

        new_published_ids = set()
        markers = MarkerArray()
        for object in msg.objects:
            # centroid
            marker = Marker(header=header)
            marker.ns = 'centroid'
            marker.id = object.id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = object.pose
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.color = object.color
            markers.markers.append(marker)
            
            # bounding box
            marker = Marker(header=header)
            marker.ns = 'bounding_box'
            marker.id = object.id
            marker.type = marker.LINE_STRIP
            marker.action = marker.ADD
            marker.pose = object.pose
            marker.scale.x = 0.1
            marker.color = ColorRGBA(1.0, 0.0, 0.0, 0.8)
            half_length = object.dimensions.x / 2.0
            half_width = object.dimensions.y / 2.0
            marker.points = [
                Point(-half_length, -half_width, 0.0),
                Point(-half_length, half_width, 0.0),
                Point(half_length, half_width, 0.0),
                Point(half_length, -half_width, 0.0),
                Point(-half_length, -half_width, 0.0),
            ]
            markers.markers.append(marker)

            # convex hull
            if len(object.convex_hull.polygon.points) > 0:
                marker = Marker(header=header)
                marker.ns = 'convex_hull'
                marker.id = object.id
                marker.type = marker.LINE_STRIP
                marker.action = marker.ADD
                marker.pose.position = object.pose.position
                marker.pose.orientation = Quaternion(0, 0, 0, 1)
                marker.scale.x = 0.1
                marker.color = ColorRGBA(0.0, 1.0, 0.0, 0.8)
                marker.points = [Point(p.x - object.pose.position.x, p.y - object.pose.position.y, p.z - object.pose.position.z) for p in object.convex_hull.polygon.points]
                marker.points.append(Point(object.convex_hull.polygon.points[0].x - object.pose.position.x, object.convex_hull.polygon.points[0].y - object.pose.position.y, object.convex_hull.polygon.points[0].z - object.pose.position.z))
                markers.markers.append(marker)

            marker = Marker(header=header)
            marker.ns = 'text'
            marker.id = object.id
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position = Point(object.pose.position.x, object.pose.position.y, object.pose.position.z + 1.0)
            marker.scale.z = 0.5
            marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
            marker.text = "%s %d (%d km/h)" % (object.label, object.id, math.sqrt(object.velocity.linear.x**2 + object.velocity.linear.y**2 + object.velocity.linear.z**2) * 3.6)
            markers.markers.append(marker)

            new_published_ids.add(object.id)

        # delete ids not published any more
        delete_ids = self.published_ids - new_published_ids
        for id in delete_ids:
            marker = Marker(header=header)
            marker.ns = 'centroid'
            marker.id = id
            marker.action = marker.DELETE
            markers.markers.append(marker)

            marker = Marker(header=header)
            marker.ns = 'bounding_box'
            marker.id = id
            marker.action = marker.DELETE
            markers.markers.append(marker)

            marker = Marker(header=header)
            marker.ns = 'convex_hull'
            marker.id = id
            marker.action = marker.DELETE
            markers.markers.append(marker)

            marker = Marker(header=header)
            marker.ns = 'text'
            marker.id = id
            marker.action = marker.DELETE
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
