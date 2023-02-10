#!/usr/bin/env python

import threading
import math

import rospy
import tf

from geometry_msgs.msg import PoseStamped, TwistStamped, PoseWithCovarianceStamped, Point
from autoware_msgs.msg import VehicleCmd

from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA

class BicycleSimulation:

    def __init__(self):
        # get parameters
        self.publish_rate = rospy.get_param("publish_rate", 50)
        self.wheel_base = rospy.get_param("wheel_base", 2.789)

        # internal state of bicycle model
        self.x = 0
        self.y = 0
        self.velocity = 0
        self.heading_angle = 0
        self.steering_angle = 0

        # localization publishers
        self.current_pose_pub = rospy.Publisher('current_pose', PoseStamped, queue_size=1)
        self.current_velocity_pub = rospy.Publisher('current_velocity', TwistStamped, queue_size=1)

        # initial position and vehicle command from outside
        self.initialpose_sub = rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.initialpose_callback)
        self.vehicle_cmd_sub = rospy.Subscriber('vehicle_cmd', VehicleCmd, self.vehicle_cmd_callback, queue_size=1)

        # visualization of the bicycle model
        self.bicycle_markers_pub = rospy.Publisher('bicycle_markers', MarkerArray, queue_size=10)

    def initialpose_callback(self, msg):
        # extract position
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # extract heading angle from orientation
        orientation = msg.pose.pose.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        _, _, self.heading_angle = tf.transformations.euler_from_quaternion(quaternion)

    def vehicle_cmd_callback(self, msg):
        # new velocity and steering angle take effect instantaneously
        self.velocity = msg.ctrl_cmd.linear_velocity
        self.steering_angle = msg.ctrl_cmd.steering_angle

    def update_model_state(self, delta_t):
        # compute change according to bicycle model equations
        x_dot = self.velocity * math.cos(self.heading_angle)
        y_dot = self.velocity * math.sin(self.heading_angle)
        heading_angle_dot = self.velocity * math.tan(self.steering_angle) / self.wheel_base

        # implment the change taking into account the update rate
        self.x += x_dot * delta_t
        self.y += y_dot * delta_t
        self.heading_angle += heading_angle_dot * delta_t

    def publisher(self):
        # publish localization at fixed rate
        rate = rospy.Rate(self.publish_rate)
        delta_t = 1. / self.publish_rate

        while not rospy.is_shutdown():
            # update model state
            self.update_model_state(delta_t)

            # publish localization messages and visualization markers
            stamp = rospy.Time.now()
            self.publish_current_pose(stamp)
            self.publish_current_velocity(stamp)
            self.publish_bicycle_markers(stamp)

            rate.sleep()

    def run(self):
        # start separate thread for publishing
        threading.Thread(target=self.publisher).start()
        # keep the main thread for subscribers
        rospy.spin()

    def publish_current_pose(self, stamp):

        pose_msg = PoseStamped()

        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = "map"

        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        pose_msg.pose.position.z = 0

        x, y, z, w = tf.transformations.quaternion_from_euler(0, 0, self.heading_angle)
        pose_msg.pose.orientation.x = x
        pose_msg.pose.orientation.y = y
        pose_msg.pose.orientation.z = z
        pose_msg.pose.orientation.w = w

        self.current_pose_pub.publish(pose_msg)

    def publish_current_velocity(self, stamp):
        
        vel_msg = TwistStamped()

        vel_msg.header.stamp = stamp
        vel_msg.header.frame_id = "base_link"

        vel_msg.twist.linear.x = self.velocity
        vel_msg.twist.linear.y = 0.0
        vel_msg.twist.linear.z = 0.0

        self.current_velocity_pub.publish(vel_msg)

    def publish_bicycle_markers(self, stamp):

        marker_array = MarkerArray()

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = stamp
        marker.type = marker.LINE_LIST
        marker.action = marker.ADD
        marker.id = 0
        marker.scale.x = 0.2
        marker.color = ColorRGBA(0.0, 1.0, 0.0, 0.8)

        # the location of the marker is current pose
        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        x, y, z, w = tf.transformations.quaternion_from_euler(0, 0, self.heading_angle)
        marker.pose.orientation.x = x
        marker.pose.orientation.y = y
        marker.pose.orientation.z = z
        marker.pose.orientation.w = w

        # draw wheel base
        marker.points.append(Point(0, 0, 0))
        marker.points.append(Point(self.wheel_base, 0, 0))

        marker_array.markers.append(marker)

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = stamp
        marker.type = marker.LINE_LIST
        marker.action = marker.ADD
        marker.id = 1
        marker.scale.x = 0.4
        marker.color = ColorRGBA(0.0, 1.0, 0.0, 0.8)

        # the location of the marker is current pose
        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.pose.orientation.x = x
        marker.pose.orientation.y = y
        marker.pose.orientation.z = z
        marker.pose.orientation.w = w

        wheel_length = 0.4

        # draw rear wheel
        marker.points.append(Point(-wheel_length, 0, 0))
        marker.points.append(Point(wheel_length, 0, 0))

        # draw front wheel
        marker.points.append(Point(self.wheel_base + wheel_length * math.cos(self.steering_angle), wheel_length * math.sin(self.steering_angle), 0))
        marker.points.append(Point(self.wheel_base - wheel_length * math.cos(self.steering_angle), -wheel_length * math.sin(self.steering_angle), 0))

        marker_array.markers.append(marker)

        self.bicycle_markers_pub.publish(marker_array)

if __name__ == '__main__':
    rospy.init_node('bicycle_simulation', log_level=rospy.INFO)
    node = BicycleSimulation()
    node.run()
