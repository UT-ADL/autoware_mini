#!/usr/bin/env python3

import rospy
import numpy as np
from ros_numpy import msgify, numpify
import message_filters
import traceback
from sensor_msgs.msg import PointCloud2

class CollisionPointsMerger:

    def __init__(self):

        # parameters
        synchronization_method = rospy.get_param("~synchronization_method")
        synchronization_queue_size = rospy.get_param("~synchronization_queue_size")
        synchronization_slop = rospy.get_param("~synchronization_slop")

        # Enable/disable topic subscriptions based on parameters
        enable_goal_checker = rospy.get_param("~enable_goal_checker")
        enable_object_checker = rospy.get_param("~enable_object_checker")
        enable_auto_stop_checker = rospy.get_param("~enable_auto_stop_checker")
        enable_traffic_light_checker = rospy.get_param("~enable_traffic_light_checker")
        enable_crosswalk_checker = rospy.get_param("~enable_crosswalk_checker")
        enable_yielding_checker = rospy.get_param("~enable_yielding_checker")
        enable_trajectory_checker = rospy.get_param("~enable_trajectory_checker")

        subscribers = []
        if enable_goal_checker:
            subscribers.append(message_filters.Subscriber("goal_collision_points", PointCloud2, tcp_nodelay=True))
        if enable_object_checker:
            subscribers.append(message_filters.Subscriber("object_collision_points", PointCloud2, tcp_nodelay=True))
        if enable_auto_stop_checker:
            subscribers.append(message_filters.Subscriber("stop_line_collision_points", PointCloud2, tcp_nodelay=True))
        if enable_traffic_light_checker:
            subscribers.append(message_filters.Subscriber("tfl_stopline_collision_points", PointCloud2, tcp_nodelay=True))
        if enable_crosswalk_checker:
            subscribers.append(message_filters.Subscriber("crosswalk_collision_points", PointCloud2, tcp_nodelay=True))
        if enable_yielding_checker:
            subscribers.append(message_filters.Subscriber("yielding_collision_points", PointCloud2, tcp_nodelay=True))
        if enable_trajectory_checker:
            subscribers.append(message_filters.Subscriber("trajectory_collision_points", PointCloud2, tcp_nodelay=True))

        if not subscribers:
            raise ValueError("No topics to subscribe to.")

        # publishers
        self.collision_points_pub = rospy.Publisher('collision_points', PointCloud2, queue_size=1, tcp_nodelay=True)

        # Synchronize messages
        if synchronization_method == "approximate":
            ts = message_filters.ApproximateTimeSynchronizer(subscribers, queue_size=synchronization_queue_size, slop=synchronization_slop)
        elif synchronization_method == "exact":
            ts = message_filters.TimeSynchronizer(subscribers, queue_size=2)
        else:
            raise ValueError(f"'{synchronization_method}' is not a known synchronization method")

        ts.registerCallback(self.collision_points_callback)

    def collision_points_callback(self, *msgs):
        try:
            # Convert all incoming messages to numpy arrays
            collision_points_np_list = [numpify(msg) for msg in msgs]
            collision_points_np = np.concatenate(collision_points_np_list)

            # Create a new PointCloud2 message
            collision_points_msg = msgify(PointCloud2, collision_points_np)
            collision_points_msg.header = msgs[0].header  # Use the header of the first message

            # Publish the merged collision points
            self.collision_points_pub.publish(collision_points_msg)

        except Exception as e:
            rospy.logerr_throttle(10, "%s - Exception in callback: %s", rospy.get_name(), traceback.format_exc())

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('collision_points_merger')
    node = CollisionPointsMerger()
    node.run()