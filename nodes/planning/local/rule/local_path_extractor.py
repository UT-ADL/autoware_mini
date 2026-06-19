#!/usr/bin/env python3
import copy
import rospy
import threading
import traceback
import shapely
from autoware_mini.msg import Path
from geometry_msgs.msg import PoseStamped
from autoware_mini.path import PathWrapper
from autoware_mini.geometry import get_heading_between_two_points, get_point32_using_heading_and_distance

class LocalPathExtractor:

    def __init__(self):

        # parameters
        self.publish_rate = rospy.get_param("publish_rate")
        self.local_path_length = rospy.get_param("local_path_length")
        self.lookahead_distance = rospy.get_param("~lookahead_distance")
        self.distance_to_lookahead_path_limit = rospy.get_param("~distance_to_lookahead_path_limit")
        self.after_goal_obstacle_check_distance = rospy.get_param("~after_goal_obstacle_check_distance")

        # variables
        self.current_pose = None
        self.global_path = None
        self.last_ego_distance = 0

        self.lock = threading.Lock()

        # publishers
        self.local_path_pub = rospy.Publisher('extracted_local_path', Path, queue_size=1, tcp_nodelay=True)

        # subscribers
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('global_path', Path, self.global_path_callback, queue_size=None, tcp_nodelay=True)

        # publish local path at a fixed rate
        rospy.Timer(rospy.Duration(1 / self.publish_rate), self.extract_local_path)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def current_pose_callback(self, msg):
        self.current_pose = msg

    def global_path_callback(self, msg):
        if not msg.waypoints:
            with self.lock:
                self.global_path = None
                self.last_ego_distance = 0
            rospy.loginfo("%s - Empty global path received", rospy.get_name())
        else:
            with self.lock:
                self.global_path = PathWrapper(msg.waypoints)
                self.last_ego_distance = 0
            rospy.loginfo("%s - Global path received with %i waypoints", rospy.get_name(), len(self.global_path.waypoints))

    def extract_local_path(self, timer_event):
        try:
            current_pose = self.current_pose

            with self.lock:
                global_path = self.global_path
                last_ego_distance = self.last_ego_distance

            if current_pose is None:
                return

            local_path = Path()
            local_path.header = current_pose.header
            
            if global_path is None:
                self.local_path_pub.publish(local_path)
                return

            current_position = shapely.Point(current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z)

            lookahead_waypoints = global_path.extract_waypoints(last_ego_distance, last_ego_distance + self.lookahead_distance, trim=True)
            lookahead_path = PathWrapper(lookahead_waypoints)

            # if ego is still close to the lookahead path then use that to calculate the new ego distance
            # this avoids choosing an incorrect local path in places where the global path overlaps itself
            if lookahead_path.linestring.distance(current_position) <= self.distance_to_lookahead_path_limit:
                ego_distance_from_global_path_start = last_ego_distance + lookahead_path.linestring.project(current_position)
            else:
                # if ego is far from its lookahead path then just find the closest point on the global path
                ego_distance_from_global_path_start = global_path.linestring.project(current_position)

            # extract local path using distances
            local_path.waypoints = global_path.extract_waypoints(ego_distance_from_global_path_start, ego_distance_from_global_path_start + self.local_path_length)
            self.last_ego_distance = ego_distance_from_global_path_start

            # if local_path was extracted and approaches end of the global_path, add additonal point for object collision checking
            if len(local_path.waypoints) > 1 and local_path.waypoints[-1] == global_path.waypoints[-1]:
                heading = get_heading_between_two_points(local_path.waypoints[-2].position, local_path.waypoints[-1].position)

                # NB! Shallow copy is enough here because we overwrite entire position and boundary points. It does not work when overwriting individual fields.
                additional_point = copy.copy(local_path.waypoints[-1])
                additional_point.position = get_point32_using_heading_and_distance(additional_point.position, heading, self.after_goal_obstacle_check_distance)
                additional_point.left_boundary_point = get_point32_using_heading_and_distance(additional_point.left_boundary_point, heading, self.after_goal_obstacle_check_distance)
                additional_point.right_boundary_point = get_point32_using_heading_and_distance(additional_point.right_boundary_point, heading, self.after_goal_obstacle_check_distance)
                local_path.waypoints.append(additional_point)

            self.local_path_pub.publish(local_path)
        except Exception as e:
            rospy.logerr_throttle(10, "%s - Exception in callback: %s", rospy.get_name(), traceback.format_exc())

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('local_path_extractor')
    node = LocalPathExtractor()
    node.run()