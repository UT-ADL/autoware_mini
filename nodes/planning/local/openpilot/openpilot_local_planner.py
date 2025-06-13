#!/usr/bin/env python3

import rospy
import numpy as np
import shapely
import tf2_ros
import threading
import message_filters
from ros_numpy import numpify
from autoware_mini.msg import Path, Waypoint
from lexus_platform.msg import Float32MultiArrayStamped
from geometry_msgs.msg import PoseStamped
from autoware_mini.path import PathWrapper
from autoware_mini.geometry import get_heading_between_two_points

class OpenpilotLocalPlanner:

    def __init__(self):

        # parameters
        self.transform_timeout = rospy.get_param('~transform_timeout')
        self.default_left_width = rospy.get_param("default_left_width")
        self.default_right_width = rospy.get_param("default_right_width")

        # variables
        self.current_position = None
        self.global_path = None
        self.output_frame = None

        self.global_path_lock = threading.Lock()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # publishers
        self.openpilot_local_path_pub = rospy.Publisher('openpilot_local_path', Path, queue_size=1, tcp_nodelay=True)

        # subscribers
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('global_path', Path, self.global_path_callback, queue_size=None, tcp_nodelay=True)

        openpilot_position_sub = message_filters.Subscriber('/openpilot/position', Float32MultiArrayStamped, queue_size=1, tcp_nodelay=True)
        openpilot_velocity_sub = message_filters.Subscriber('/openpilot/velocity', Float32MultiArrayStamped, queue_size=1, tcp_nodelay=True)
        ts = message_filters.TimeSynchronizer([openpilot_position_sub, openpilot_velocity_sub], queue_size=2)

        ts.registerCallback(self.openpilot_prediction_callback)

    def current_pose_callback(self, msg):
        self.current_position = shapely.Point(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def global_path_callback(self, msg):
        output_frame = msg.header.frame_id

        if len(msg.waypoints) == 0:
            global_path = None
            rospy.loginfo("%s - Empty global path received", rospy.get_name())
        else:
            global_path = PathWrapper(msg.waypoints, velocities=True, blinkers=True)
            rospy.loginfo("%s - Global path received with %i waypoints", rospy.get_name(), len(global_path.waypoints))

        with self.global_path_lock:
            self.output_frame = output_frame
            self.global_path = global_path

    def openpilot_prediction_callback(self, position_msg, velocity_msg):
        openpilot_plan = float32_multiarray_to_numpy(position_msg).T
        openpilot_velocity = float32_multiarray_to_numpy(velocity_msg).T
        current_position = self.current_position

        with self.global_path_lock:
            global_path = self.global_path
            output_frame = self.output_frame

        openpilot_local_path = Path()
        openpilot_local_path.header.frame_id = output_frame
        openpilot_local_path.header.stamp = position_msg.header.stamp

        if current_position is None or global_path is None:
            self.openpilot_local_path_pub.publish(openpilot_local_path)
            return

        # fetch the transform from 'openpilot' frame to ouput frame
        try:
            transform = self.tf_buffer.lookup_transform(output_frame, position_msg.header.frame_id, position_msg.header.stamp, rospy.Duration(self.transform_timeout))
            tf_matrix = numpify(transform.transform)
        except (tf2_ros.TransformException, rospy.ROSTimeMovedBackwardsException) as e:
            rospy.logwarn("%s - %s", rospy.get_name(), e)
            return
        
        # transform openpilot plan to the given output frame
        openpilot_plan[:, 3] = 1 # replece the time dimension with ones to get homogeneous points
        openpilot_plan_homogeneous = openpilot_plan @ tf_matrix.T
        openpilot_plan = openpilot_plan_homogeneous[:, :3]

        waypoints = []
        for i in range(len(openpilot_plan)):
            if i == len(openpilot_plan) - 1:
                # use heading of previous point - last point of last lanelet has no following point 
                x, y, z = openpilot_plan[i]
                x_prev, y_prev, z_prev = openpilot_plan[i-1]
                waypoint = self.create_waypoint(shapely.Point(x, y, z), openpilot_velocity[i], global_path, previous_point=shapely.Point(x_prev, y_prev, z_prev))
                waypoints.append(waypoint)
            else:
                x, y, z = openpilot_plan[i]
                x_next, y_next, z_next = openpilot_plan[i+1]
                waypoint = self.create_waypoint(shapely.Point(x, y, z), openpilot_velocity[i], global_path, next_point=shapely.Point(x_next, y_next, z_next))
                waypoints.append(waypoint)

        openpilot_local_path.waypoints = waypoints
        self.openpilot_local_path_pub.publish(openpilot_local_path)

    def create_waypoint(self, current_point, openpilot_velocity, global_path, next_point=None, previous_point=None):
        if next_point is not None:
            heading = get_heading_between_two_points(current_point, next_point)
        else:
            heading = get_heading_between_two_points(previous_point, current_point)

        current_point_dist = global_path.linestring.project(current_point)

        waypoint = Waypoint()
        waypoint.position.x = current_point.x
        waypoint.position.y = current_point.y
        waypoint.position.z = current_point.z
        waypoint.lanechange_state = 0
        waypoint.blinker_state = global_path.get_blinker_at_distance(current_point_dist)
        waypoint.heading = heading
        waypoint.speed = np.linalg.norm(openpilot_velocity[:3]) #TODO: Speed is currently in openpilot frame, should be converted to base_link
        waypoint.left_width = self.default_left_width
        waypoint.right_width = self.default_right_width

        return waypoint

    def run(self):
        rospy.spin()

def float32_multiarray_to_numpy(multiarray):
    dims = tuple(map(lambda x: x.size, multiarray.layout.dim))
    data = multiarray.data[multiarray.layout.data_offset:]
    return np.array(data, dtype=np.float32).reshape(dims)

if __name__ == '__main__':
    rospy.init_node('openpilot_local_planner')
    node = OpenpilotLocalPlanner()
    node.run()