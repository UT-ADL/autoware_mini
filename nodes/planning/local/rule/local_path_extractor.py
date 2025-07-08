#!/usr/bin/env python3
import rospy
import threading
import traceback
import shapely
from autoware_mini.msg import Path, Waypoint
from geometry_msgs.msg import PoseStamped
from autoware_mini.path import PathWrapper
from autoware_mini.geometry import get_heading_between_two_points, get_point_using_heading_and_distance

class LocalPathExtractor:

    def __init__(self):

        # parameters
        self.publish_rate = rospy.get_param("~publish_rate")
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

    def current_pose_callback(self, msg):
        self.current_pose = msg

    def global_path_callback(self, msg):
        if len(msg.waypoints) == 0:
            with self.lock:
                self.global_path = None
                self.last_ego_distance = 0
            rospy.loginfo("%s - Empty global path received", rospy.get_name())
        else:
            with self.lock:
                self.global_path = PathWrapper(msg.waypoints, distances=True)
                self.last_ego_distance = 0
            rospy.loginfo("%s - Global path received with %i waypoints", rospy.get_name(), len(self.global_path.waypoints))

    def extract_local_path(self):
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
            lookahead_waypoints = global_path.extract_waypoints(last_ego_distance, last_ego_distance + self.lookahead_distance)
            lookahead_path = PathWrapper(lookahead_waypoints)

            # if ego is still close to the lookahead path then use that to calculate the new ego distance
            # this avoids choosing an incorrect local path in places where the global path overlaps itself
            if lookahead_path.linestring.distance(current_position) <= self.distance_to_lookahead_path_limit:
                ego_distance_from_global_path_start = lookahead_path.linestring.project(current_position) + last_ego_distance
            else:
                # if ego is far from its lookahead path then just find the closest point on the global path
                ego_distance_from_global_path_start = global_path.linestring.project(current_position)

            # extract local path using distances
            local_path.waypoints = global_path.extract_waypoints(ego_distance_from_global_path_start, ego_distance_from_global_path_start + self.local_path_length)
            self.last_ego_distance = ego_distance_from_global_path_start

            # if local_path was extracted and approaches end of the global_path, add additonal point for object collision checking
            if len(local_path.waypoints) > 1 and local_path.waypoints[-1] == global_path.waypoints[-1]:
                heading = get_heading_between_two_points(local_path.waypoints[-2].position, local_path.waypoints[-1].position)
                point = get_point_using_heading_and_distance(local_path.waypoints[-1].position, heading, self.after_goal_obstacle_check_distance)
                local_path.waypoints.append(Waypoint(position=point))

            self.local_path_pub.publish(local_path)
        except Exception as e:
            rospy.logerr_throttle(10, "%s - Exception in callback: %s", rospy.get_name(), traceback.format_exc())

    def run(self):
        # start separate thread for spinning subcribers
        t = threading.Thread(target=rospy.spin)
        t.daemon = True # make sure Ctrl+C works
        t.start()

        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            self.extract_local_path()
            try:
                rate.sleep()
            except (rospy.ROSTimeMovedBackwardsException, rospy.exceptions.ROSInterruptException):
                pass

if __name__ == '__main__':
    rospy.init_node('local_path_extractor')
    node = LocalPathExtractor()
    node.run()