#!/usr/bin/env python3

import rospy
import threading

from geometry_msgs.msg import PointStamped, Point32
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

        # initial position and vehicle command from outside
        rospy.Subscriber('/clicked_point', PointStamped, self.point_callback, queue_size=None, tcp_nodelay=True)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def point_callback(self, msg):
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

    def run(self):
        # start separate thread for spinning subcribers
        t = threading.Thread(target=rospy.spin)
        t.daemon = True # make sure Ctrl+C works
        t.start()

        # publish objects at fixed rate
        rate = rospy.Rate(self.publish_rate)

        while not rospy.is_shutdown():
            self.publish_detected_objects()
            try:
                rate.sleep()
            except (rospy.ROSTimeMovedBackwardsException, rospy.exceptions.ROSInterruptException):
                pass

    def publish_detected_objects(self):
        stamp = rospy.Time.now()

        # create message
        msg = DetectedObjectArray()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        msg.objects = self.objects
        self.objects_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('obstacle_simulation', log_level=rospy.INFO)
    node = ObstacleSimulation()
    node.run()
