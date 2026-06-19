#!/usr/bin/env python3

import rospy
import traceback
import tf2_ros
from std_msgs.msg import Bool

from autoware_mini.transform import transform_point

from geometry_msgs.msg import PointStamped
from autoware_mini.msg import DetectedObjectArray, DetectedObject
from std_msgs.msg import ColorRGBA

class ObstacleSimulation:
    def __init__(self):
        # get parameters
        self.publish_rate = rospy.get_param("~publish_rate")

        # list of objects
        self.objects = []
        self.id = 0

        # detected objects publisher
        self.objects_pub = rospy.Publisher('detected_objects', DetectedObjectArray, queue_size=1, tcp_nodelay=True)

        self.tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # initial position and vehicle command from outside
        rospy.Subscriber('/clicked_point', PointStamped, self.point_callback, queue_size=None, tcp_nodelay=True)

        # When assistance or ignore-object mode is active, ignore clicked points so the
        # click doesn't both spawn an obstacle and trigger the active feature.
        self.assistance_enabled = False
        self.ignore_object_enabled = False
        rospy.Subscriber('/planning/assistance_enabled', Bool, self.assistance_state_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/perception/ignore_object_enabled', Bool, self.ignore_object_state_callback, queue_size=1, tcp_nodelay=True)

        # start the publish loop
        rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.publish_detected_objects)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def point_callback(self, msg):
        if self.assistance_enabled or self.ignore_object_enabled:
            return

        if msg.header.frame_id == "base_link":
            transform = self.tf_buffer.lookup_transform("map", "base_link", msg.header.stamp, rospy.Duration(0.06))
            msg.point = transform_point(transform, msg.point)

        # check if clicked on an existing object
        for o in self.objects:
            if o.center.x - o.dimensions.x / 2.0 <= msg.point.x <= o.center.x + o.dimensions.x / 2.0 and \
                    o.center.y - o.dimensions.y / 2.0 <= msg.point.y <= o.center.y + o.dimensions.y / 2.0:
                self.objects.remove(o)
                rospy.loginfo("%s - removed obstacle %d", rospy.get_name(), o.id)
                return

        # if not, create a new 
        obj = DetectedObject()
        obj.id = self.id
        obj.label = 'unknown'
        obj.color = ColorRGBA(1.0, 1.0, 1.0, 0.8)
        obj.valid = True

        obj.center.x = obj.centroid.x = msg.point.x
        obj.center.y = obj.centroid.y = msg.point.y
        obj.center.z = obj.centroid.z = msg.point.z
        obj.heading = 0.0
        obj.dimensions.x = 1.0
        obj.dimensions.y = 1.0
        obj.dimensions.z = 1.0
        obj.position_reliable = True

        obj.convex_hull = [msg.point.x - 0.5, msg.point.y - 0.5, msg.point.z,
                           msg.point.x - 0.5, msg.point.y + 0.5, msg.point.z,
                           msg.point.x + 0.5, msg.point.y + 0.5, msg.point.z,
                           msg.point.x + 0.5, msg.point.y - 0.5, msg.point.z]

        self.objects.append(obj)
        rospy.loginfo("%s - added obstacle %d at (%f, %f, %f) in %s frame", rospy.get_name(), self.id, msg.point.x, msg.point.y, msg.point.z, msg.header.frame_id)
        self.id += 1

    def publish_detected_objects(self, timer_event):
        try:
            stamp = rospy.Time.now()

            # create message
            msg = DetectedObjectArray()
            msg.header.stamp = stamp
            msg.header.frame_id = 'map'
            msg.objects = self.objects
            self.objects_pub.publish(msg)
        except Exception as e:
            rospy.logerr_throttle(10, "%s - Exception in callback: %s", rospy.get_name(), traceback.format_exc())

    def assistance_state_callback(self, msg):
        self.assistance_enabled = bool(msg.data)

    def ignore_object_state_callback(self, msg):
        self.ignore_object_enabled = bool(msg.data)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('obstacle_simulation', log_level=rospy.INFO)
    node = ObstacleSimulation()
    node.run()
