#!/usr/bin/env python3

from geometry_msgs.msg import PointStamped, Point, Point32, Vector3Stamped

def tf_point32(p32, tf_listener, map_frame, radar_frame):
    point_stamped = PointStamped()
    point_stamped.header.frame_id = radar_frame
    point_stamped.point = Point(x=p32.x, y=p32.y, z=p32.z)
    tfed_point = tf_listener.transformPoint(map_frame, point_stamped).point
    return Point32(x=tfed_point.x, y=tfed_point.y, z=tfed_point.z)


def tf_point(p, tf_listener, map_frame, radar_frame):
    point_stamped = PointStamped()
    point_stamped.header.frame_id = radar_frame
    point_stamped.point = Point(x=p.x, y=p.y, z=p.z)
    tfed_point = tf_listener.transformPoint(map_frame, point_stamped).point
    return Point(x=tfed_point.x, y=tfed_point.y, z=tfed_point.z)


def tf_vector3(v3, tf_listener, map_frame, radar_frame):
    vector3_stamped = Vector3Stamped(vector=v3)
    vector3_stamped.header.frame_id = radar_frame
    tfed_vector3 = tf_listener.transformVector3(map_frame, vector3_stamped).vector
    return tfed_vector3
