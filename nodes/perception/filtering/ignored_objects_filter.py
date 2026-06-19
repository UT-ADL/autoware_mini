#!/usr/bin/env python3

import threading
import numpy as np
import shapely

import rospy
import tf2_ros

from std_msgs.msg import Bool
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import PointStamped
from autoware_mini.msg import DetectedObjectArray
from autoware_mini.transform import transform_point


class IgnoredObjectsFilter:

    def __init__(self):

        self._state_lock = threading.Lock()
        self.ignored_ids = set()
        self.last_objects = []
        self.ignore_pick_armed = False

        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)

        self.tracked_objects_pub = rospy.Publisher('tracked_objects', DetectedObjectArray, queue_size=1, tcp_nodelay=True)
        self.ignored_objects_pub = rospy.Publisher('ignored_objects', DetectedObjectArray, queue_size=1, tcp_nodelay=True)
        self.ignore_object_enabled_pub = rospy.Publisher('ignore_object_enabled', Bool, queue_size=1, latch=True, tcp_nodelay=True)

        rospy.Subscriber('tracked_objects_unfiltered', DetectedObjectArray, self.tracked_objects_callback, queue_size=1, buff_size=2**20, tcp_nodelay=True)
        rospy.Subscriber('/clicked_point', PointStamped, self.clicked_point_callback, queue_size=1, tcp_nodelay=True)

        rospy.Service('service_ignore_object', Empty, self.service_ignore_object)

        self.ignore_object_enabled_pub.publish(Bool(data=False))

        rospy.loginfo("%s - initialized", rospy.get_name())

    def service_ignore_object(self, req):
        with self._state_lock:
            self.ignore_pick_armed = not self.ignore_pick_armed
            armed = self.ignore_pick_armed
        self.ignore_object_enabled_pub.publish(Bool(data=armed))
        if armed:
            rospy.loginfo("%s - ignore-object mode armed: next click will toggle an object", rospy.get_name())
        else:
            rospy.loginfo("%s - ignore-object mode cancelled", rospy.get_name())
        return EmptyResponse()

    def clicked_point_callback(self, msg):
        with self._state_lock:
            if not self.ignore_pick_armed:
                return
            objects = self.last_objects

        point = msg.point
        if msg.header.frame_id != "map":
            try:
                transform = self.tf_buffer.lookup_transform("map", msg.header.frame_id, msg.header.stamp, rospy.Duration(0.1))
            except (tf2_ros.TransformException, rospy.ROSTimeMovedBackwardsException) as e:
                rospy.logwarn("%s - transform from %s to map failed (%s); try clicking again", rospy.get_name(), msg.header.frame_id, e)
                return
            point = transform_point(transform, point)

        click = shapely.points(point.x, point.y)
        hit_id = None
        for obj in objects:
            hull = shapely.polygons(np.array(obj.convex_hull).reshape(-1, 3))
            if shapely.contains(hull, click):
                hit_id = obj.id
                break

        with self._state_lock:
            self.ignore_pick_armed = False
            if hit_id is not None:
                if hit_id in self.ignored_ids:
                    self.ignored_ids.discard(hit_id)
                    action = "unignored"
                else:
                    self.ignored_ids.add(hit_id)
                    action = "ignored"
            else:
                action = None

        self.ignore_object_enabled_pub.publish(Bool(data=False))
        if action is not None:
            rospy.loginfo("%s - %s tracked object %d", rospy.get_name(), action, hit_id)
        else:
            rospy.logwarn("%s - clicked point did not hit any tracked object; ignore-object mode disarmed", rospy.get_name())

    def tracked_objects_callback(self, msg):
        filtered = []
        ignored = []
        current_ids = set()
        for obj in msg.objects:
            current_ids.add(obj.id)
            if obj.id in self.ignored_ids:
                obj.label = "IGNORED"
                ignored.append(obj)
            else:
                filtered.append(obj)

        with self._state_lock:
            self.last_objects = msg.objects
            # Prune ids no longer present so the set doesn't grow without bound.
            self.ignored_ids &= current_ids

        msg.objects = filtered
        self.tracked_objects_pub.publish(msg)
        msg.objects = ignored
        self.ignored_objects_pub.publish(msg)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('ignored_objects_filter', log_level=rospy.INFO)
    node = IgnoredObjectsFilter()
    node.run()
