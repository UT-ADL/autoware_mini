#!/usr/bin/env python3

import rospy
import math
import shapely
import numpy as np

from autoware_mini.msg import Path, DetectedObjectArray
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import PointCloud2
from tf2_ros import TransformListener, Buffer

from autoware_mini.geometry import get_angle_between_two_headings, get_speed_from_velocity
from autoware_mini.collision import CollisionPoints, calculate_time_to_destination
from autoware_mini.path import PathWrapper
from autoware_mini.transform import get_car_front_point

class TrajectoryCollisionChecker:

    def __init__(self):

        # parameters
        self.safety_box_width = rospy.get_param("safety_box_width")
        self.safety_box_length = rospy.get_param("safety_box_length")
        self.braking_safety_distance_trajectory = rospy.get_param("~braking_safety_distance_trajectory")
        self.heading_alignment_limit = rospy.get_param("~heading_alignment_limit")
        self.use_object_width = rospy.get_param("use_object_width")
        self.safety_time_ego_front = rospy.get_param("~safety_time_ego_front")
        self.safety_time_ego_rear = rospy.get_param("~safety_time_ego_rear")

        # variables
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
        self.detected_objects = None
        self.current_speed = None


        # publishers
        self.local_path_collision_pub = rospy.Publisher('trajectory_collision_points', PointCloud2, queue_size=1, tcp_nodelay=True)

        # subscribers
        rospy.Subscriber('/detection/predicted_objects_map', DetectedObjectArray, self.predicted_objects_callback, queue_size=1, buff_size=2**20, tcp_nodelay=True)
        rospy.Subscriber('extracted_local_path', Path, self.local_path_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/localization/current_velocity', TwistStamped, self.current_velocity_callback, queue_size=1, tcp_nodelay=True)

    def predicted_objects_callback(self, msg):
        self.detected_objects = msg.objects

    def current_velocity_callback(self, msg):
        self.current_speed = msg.twist.linear.x

    def local_path_callback(self, msg):

        detected_objects = self.detected_objects
        current_speed = self.current_speed

        if detected_objects is None or current_speed is None:
            rospy.logwarn_throttle(3, "%s - detected objects or current velocity not received!", rospy.get_name())
            return

        collision_points = CollisionPoints()

        if len(msg.waypoints) > 0:
            local_path = PathWrapper(msg.waypoints, distances=True)
            local_path_buffer = local_path.linestring.buffer(self.safety_box_width / 2, cap_style="flat")
            shapely.prepare(local_path_buffer)

            car_front = get_car_front_point(self.tf_buffer, msg.header.frame_id)
            car_front_distance_from_local_path_start = local_path.linestring.project(car_front)

            for obj in detected_objects:
                for path in obj.candidate_trajectories.paths:

                    trajectory = PathWrapper(path.waypoints)
                    trajectory_to_check = trajectory.linestring

                    if self.use_object_width:
                        trajectory_to_check = trajectory_to_check.buffer(obj.dimensions.y / 2, cap_style="flat")

                    if local_path_buffer.intersects(trajectory_to_check):
                        trajectory_intersection_result = trajectory_to_check.intersection(local_path_buffer)
                        trajectory_intersection_coords = shapely.get_coordinates(trajectory_intersection_result)
                        trajectory_intersection_points = shapely.points(trajectory_intersection_coords)

                        # Calculate trajectory intersection distances for ego vehicle and object
                        distances = local_path.linestring.project(trajectory_intersection_points)
                        intersection_distance_from_local_path_start_min = min(distances)
                        intersection_distance_from_local_path_start_max = max(distances)
                        last_point_of_trajectory = shapely.Point(trajectory.linestring.coords[-1])

                        object_current_location = shapely.Point(obj.centroid.x, obj.centroid.y)
                        object_distance_from_local_path_start = local_path.linestring.project(object_current_location)

                        # Ignore objects behind local_path start OR trajectory passing through local_path start AND endpoint being also on local_path
                        if math.isclose(object_distance_from_local_path_start, 0.0, abs_tol=0.001) or \
                            (math.isclose(intersection_distance_from_local_path_start_min, 0.0, abs_tol=0.001) and \
                                local_path_buffer.intersects(last_point_of_trajectory)):
                            continue

                        object_local_path_heading = local_path.get_heading_at_distance(object_distance_from_local_path_start)
                        heading_difference = math.degrees(get_angle_between_two_headings(obj.heading, object_local_path_heading))
                        object_polygon = shapely.polygons(np.array(obj.convex_hull).reshape(-1, 3))

                        # Ignore object trajectories that are on our path and with similar heading - must be in front of us
                        if local_path_buffer.intersects(object_polygon) and heading_difference < self.heading_alignment_limit:
                            continue

                        # Extract INTERSECTION AREA: distances on local_path and extract points
                        collision_area_points, collision_area_distances = local_path.extract_points_and_distances(intersection_distance_from_local_path_start_min, intersection_distance_from_local_path_start_max)

                        # EGO distances, arrival and leaving times
                        collision_distance_from_ego_front = collision_area_distances - car_front_distance_from_local_path_start
                        ego_arrival_times = calculate_time_to_destination(current_speed, 0, collision_distance_from_ego_front)
                        ego_leaving_times = calculate_time_to_destination(current_speed, 0, collision_distance_from_ego_front + self.safety_box_length)
                        ego_arrival_times -= self.safety_time_ego_front
                        ego_leaving_times += self.safety_time_ego_rear

                        # OBJECT distances, arrival and leaving times
                        obj_velocity = get_speed_from_velocity(obj.velocity)
                        obj_acceleration = get_speed_from_velocity(obj.acceleration)
                        collision_distance_from_obj_front = np.array([trajectory.linestring.project(p) for p in collision_area_points])
                        obj_arrival_times = calculate_time_to_destination(obj_velocity, obj_acceleration, collision_distance_from_obj_front)
                        obj_leaving_times = calculate_time_to_destination(obj_velocity, obj_acceleration, collision_distance_from_obj_front + obj.dimensions.x)

                        # FIND COLLISION AREA
                        collision_mask = ((ego_arrival_times <= obj_leaving_times) & (ego_leaving_times >= obj_arrival_times))
                        collision_area_points = collision_area_points[collision_mask]

                        if heading_difference < self.heading_alignment_limit:
                            # objects with similar heading - add collision points with object's velocity
                            collision_points.add_points(points = collision_area_points,
                                vx = obj.velocity.x,
                                vy = obj.velocity.y,
                                vz = obj.velocity.z,
                                distance_to_stop = self.braking_safety_distance_trajectory,
                                deceleration_limit = np.inf,
                                category = CollisionPoints.MERGING_TRAJECTORY)
                        else:
                            # objects intersecting at angle, add with 0 velocity
                            collision_points.add_points(points = collision_area_points,
                                vx = 0.0,
                                vy = 0.0,
                                vz = 0.0,
                                distance_to_stop = self.braking_safety_distance_trajectory,
                                deceleration_limit = np.inf,
                                category = CollisionPoints.COLLIDING_TRAJECTORY)

        collision_points_msg = collision_points.create_message()
        collision_points_msg.header = msg.header
        self.local_path_collision_pub.publish(collision_points_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('trajectory_collision_checker')
    node = TrajectoryCollisionChecker()
    node.run()