#!/usr/bin/env python3

import rospy
import threading

from geometry_msgs.msg import PointStamped, Point32
from autoware_msgs.msg import DetectedObjectArray, DetectedObject
from std_msgs.msg import ColorRGBA

class ObstacleSimulation:
    def __init__(self):
        # get parameters
        self.publish_rate = rospy.get_param("~publish_rate", 10)

        # list of objects
        self.objects = []
        self.id = 0

        # detected objects publisher
        self.objects_pub = rospy.Publisher('detected_objects', DetectedObjectArray, queue_size=10)

        # initial position and vehicle command from outside
        rospy.Subscriber('/clicked_point', PointStamped, self.point_callback)

        rospy.loginfo("obstacle_simulation - initialized")

    def point_callback(self, msg):
        # check if clicked on an existing object
        for o in self.objects:
            if o.pose.position.x - o.dimensions.x / 2.0 <= msg.point.x <= o.pose.position.x + o.dimensions.x / 2.0 and \
                    o.pose.position.y - o.dimensions.y / 2.0 <= msg.point.y <= o.pose.position.y + o.dimensions.y / 2.0:
                self.objects.remove(o)
                rospy.loginfo("obstacle_simulation - removed obstacle %d", o.id)
                return

        # if not, create a new 
        obj = DetectedObject()
        obj.header.frame_id = msg.header.frame_id

        obj.id = self.id
        obj.label = 'unknown'
        obj.color = ColorRGBA(1.0, 1.0, 1.0, 0.8)
        obj.valid = True

        obj.space_frame = msg.header.frame_id
        obj.pose.position.x = msg.point.x
        obj.pose.position.y = msg.point.y
        obj.pose.position.z = msg.point.z
        obj.pose.orientation.x = 0.0
        obj.pose.orientation.y = 0.0
        obj.pose.orientation.z = 0.0
        obj.pose.orientation.w = 1.0
        obj.dimensions.x = 1.0
        obj.dimensions.y = 1.0
        obj.dimensions.z = 1.0
        obj.pose_reliable = True

        obj.convex_hull.polygon.points = [
            Point32(msg.point.x - 0.5, msg.point.y - 0.5, msg.point.z),
            Point32(msg.point.x - 0.5, msg.point.y + 0.5, msg.point.z),
            Point32(msg.point.x + 0.5, msg.point.y + 0.5, msg.point.z),
            Point32(msg.point.x + 0.5, msg.point.y - 0.5, msg.point.z)
        ]

        self.objects.append(obj)
        rospy.loginfo("obstacle_simulation - added obstacle %d at (%f, %f, %f) in %s frame", self.id, msg.point.x, msg.point.y, msg.point.z, msg.header.frame_id)
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
            rate.sleep()

    def publish_detected_objects(self):
        stamp = rospy.Time.now()

        # create message
        msg = DetectedObjectArray()
        msg.header.stamp = stamp
        msg.objects = self.objects

        # take message frame from the first object
        if len(msg.objects) > 0:
            msg.header.frame_id = msg.objects[0].header.frame_id

        # overwrite object timestamp
        for o in msg.objects:
            o.header.stamp = stamp
        

        self.objects_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('obstacle_simulation', log_level=rospy.INFO)
    node = ObstacleSimulation()
    node.run()
