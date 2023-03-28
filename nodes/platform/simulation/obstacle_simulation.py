<<<<<<< HEAD
#!/usr/bin/env python

import rospy
import threading
<<<<<<< HEAD

from geometry_msgs.msg import PointStamped, Point32
from autoware_msgs.msg import DetectedObjectArray, DetectedObject
from std_msgs.msg import ColorRGBA

=======
<<<<<<< HEAD
=======
#!/usr/bin/env python3

import rospy
import threading
>>>>>>> Added basic obstacle simulation node. Reorganize simulation and obstacle detection code a bit.
import math
import cv2

from geometry_msgs.msg import PointStamped, Point32
from autoware_msgs.msg import DetectedObjectArray, DetectedObject

from std_msgs.msg import ColorRGBA

from helpers import get_heading_from_pose_orientation

>>>>>>> Added basic obstacle simulation node. Reorganize simulation and obstacle detection code a bit.
class ObstacleSimulation:
    def __init__(self):
        # get parameters
        self.publish_rate = rospy.get_param("~publish_rate", 10)
<<<<<<< HEAD

        # list of objects
        self.objects = []
        self.id = 0
=======
        self.add_or_move = rospy.get_param("~add_or_move", 'move')

        # list of objects
        self.objects = []
>>>>>>> Added basic obstacle simulation node. Reorganize simulation and obstacle detection code a bit.

        # detected objects publisher
        self.objects_pub = rospy.Publisher('detected_objects', DetectedObjectArray, queue_size=10)

        # initial position and vehicle command from outside
        rospy.Subscriber('/clicked_point', PointStamped, self.point_callback)

        rospy.loginfo("obstacle_simulation - initialized")

    def point_callback(self, msg):
<<<<<<< HEAD
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
=======
        if self.add_or_move == 'add':
            self.objects.append(msg)
            rospy.loginfo("obstacle_simulation - appended obstacle at (%f, %f, %f) in %s frame", msg.point.x, msg.point.y, msg.point.z, msg.header.frame_id)
        elif self.add_or_move == 'move':
            self.objects = [msg]
            rospy.loginfo("obstacle_simulation - moved obstacle to (%f, %f, %f) in %s frame", msg.point.x, msg.point.y, msg.point.z, msg.header.frame_id)
        else:
            assert False, "Unknown add_or_move parameter value: " + self.add_or_move
>>>>>>> Added basic obstacle simulation node. Reorganize simulation and obstacle detection code a bit.

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
<<<<<<< HEAD
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
        

=======
        # create message
        msg = DetectedObjectArray()
        msg.header.stamp = rospy.Time.now()

        # create objects
        for i, o in enumerate(self.objects):
            obj = DetectedObject()
            obj.header.stamp = msg.header.stamp
            obj.header.frame_id = o.header.frame_id
            msg.header.frame_id = o.header.frame_id

            obj.id = i
            obj.label = 'unknown'
            obj.color = ColorRGBA(1.0, 1.0, 1.0, 0.8)
            obj.valid = True

            obj.space_frame = o.header.frame_id
            obj.pose.position.x = o.point.x
            obj.pose.position.y = o.point.y
            obj.pose.position.z = o.point.z
            obj.pose.orientation.x = 0.0
            obj.pose.orientation.y = 0.0
            obj.pose.orientation.z = 0.0
            obj.pose.orientation.w = 1.0
            obj.dimensions.x = 1.0
            obj.dimensions.y = 1.0
            obj.dimensions.z = 1.0
            obj.pose_reliable = True

            points = cv2.boxPoints((
                (obj.pose.position.x, obj.pose.position.y), 
                (obj.dimensions.x, obj.dimensions.y), 
                math.degrees(get_heading_from_pose_orientation(obj.pose))
            ))
            obj.convex_hull.polygon.points = [Point32(x, y, o.point.z) for x, y in points]

            msg.objects.append(obj)
        
>>>>>>> Added basic obstacle simulation node. Reorganize simulation and obstacle detection code a bit.
        self.objects_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('obstacle_simulation', log_level=rospy.INFO)
    node = ObstacleSimulation()
    node.run()
