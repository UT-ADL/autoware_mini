#!/usr/bin/env python3

import rospy
import math
import message_filters
import traceback
import shapely
import numpy as np
import warnings
import pyclothoids
from numpy.lib.recfunctions import structured_to_unstructured
from ros_numpy import numpify
from autoware_mini.msg import Path, LocalPath, Log
from autoware_mini.messages import path_to_local_path
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, TwistStamped
from autoware_mini.path import PathWrapper
from autoware_mini.collision import CollisionPoints
from autoware_mini.geometry import get_distance_between_two_points_2d, get_heading_from_orientation, calculate_headings, get_angle_between_two_headings
from autoware_mini.shapely import offset_curve, linesubstring
from autoware_mini.transform import get_distance_to_car_front

class SwervePlanner:

    def __init__(self):

        # parameters
        self.braking_reaction_time = rospy.get_param("braking_reaction_time")
        self.stopped_speed_limit = rospy.get_param("stopped_speed_limit")
        self.wide_safety_box_width = rospy.get_param("wide_safety_box_width")
        self.narrow_safety_box_width = rospy.get_param("narrow_safety_box_width")
        self.num_parallel_paths = rospy.get_param("parallel_paths")
        self.parallel_paths_interval = rospy.get_param("parallel_paths_interval")
        self.swerving_speed_tolerance = rospy.get_param("~swerving_speed_tolerance")
        self.swerving_counter_min_limit = rospy.get_param("~swerving_counter_min_limit")
        self.distance_to_centerline_limit = rospy.get_param("~distance_to_centerline_limit")
        swerving_alignment_angle_limit = rospy.get_param("~swerving_alignment_angle_limit")
        self.swerving_blocked_speed_limit = rospy.get_param("~swerving_blocked_speed_limit")
        synchronization_method = rospy.get_param("~synchronization_method")
        synchronization_queue_size = rospy.get_param("~synchronization_queue_size")
        synchronization_slop = rospy.get_param("~synchronization_slop")

        # variables
        self.current_pose = None
        self.current_speed = None
        self.distance_to_car_front = get_distance_to_car_front()
        self.parallel_path_counters = np.zeros(self.num_parallel_paths, dtype=int)
        self.last_parallel_path_id = 0
        self.last_target_path_id = 0
        self.swerving_alignment_angle_limit = math.radians(swerving_alignment_angle_limit)
        
        # publishers
        self.local_path_pub = rospy.Publisher('local_path', LocalPath, queue_size=1, tcp_nodelay=True)
        self.log_message_pub = rospy.Publisher('/dashboard/log_message', Log, queue_size=1, tcp_nodelay=True)

        # subscribers
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/localization/current_velocity', TwistStamped, self.current_velocity_callback, queue_size=1, tcp_nodelay=True)

        collision_points_sub = message_filters.Subscriber('collision_points', PointCloud2, tcp_nodelay=True)
        local_path_sub = message_filters.Subscriber('extracted_local_path', Path, tcp_nodelay=True)

        if synchronization_method == "approximate":
            ts = message_filters.ApproximateTimeSynchronizer([collision_points_sub, local_path_sub], queue_size=synchronization_queue_size, slop=synchronization_slop)
        elif synchronization_method == "exact":
            ts = message_filters.TimeSynchronizer([collision_points_sub, local_path_sub], queue_size=2)
        else:
            raise ValueError(f"'{synchronization_method}' is not a known synchronization method")

        ts.registerCallback(self.collision_points_and_path_callback)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def current_velocity_callback(self, msg):
        self.current_speed = msg.twist.linear.x

    def current_pose_callback(self, msg):
        self.current_pose = msg.pose

    def collision_points_and_path_callback(self, collision_points_msg, local_path_msg):
        try:
            collision_points = numpify(collision_points_msg)
            current_pose = self.current_pose
            current_speed = self.current_speed

            if current_speed is None or current_pose is None:
                rospy.logwarn_throttle(3, "%s - current speed or position not received!", rospy.get_name())
                self.parallel_path_counters.fill(0)
                return

            if not local_path_msg.waypoints:
                self.publish_local_path(path_to_local_path(local_path_msg))
                self.parallel_path_counters.fill(0)
                return
            
            current_position = shapely.Point(current_pose.position.x, current_pose.position.y, current_pose.position.z)
            current_heading = get_heading_from_orientation(current_pose.orientation)

            # create local path
            base_local_path = PathWrapper(local_path_msg.waypoints)

            # if ego is too far from local path, then publish an empty local path
            if base_local_path.linestring.distance(current_position) > self.distance_to_centerline_limit:
                local_path_msg.waypoints = []
                self.publish_local_path(path_to_local_path(local_path_msg))
                self.parallel_path_counters.fill(0)
                return

            # generate parallel paths for swerving
            parallel_paths = generate_parallel_paths(current_position, current_heading, base_local_path.linestring, self.num_parallel_paths, self.parallel_paths_interval)

            # choose the best path among parallel paths
            (target_object_distance,
             target_object_speed,
             stopping_point_distance,
             collision_point_category,
             collision_point_deceleration,
             target_distance_object,
             chosen_parallel_path_id) = self.choose_path(parallel_paths, current_position, current_speed, local_path_msg.waypoints[0].speed, collision_points)
            
            chosen_path_linestring = parallel_paths[chosen_parallel_path_id]
            assert shapely.get_num_points(chosen_path_linestring) == len(local_path_msg.waypoints), "Chosen path and local path waypoints length mismatch"

            for i, (x, y, z) in enumerate(chosen_path_linestring.coords):
                waypoint = local_path_msg.waypoints[i]
                waypoint.position.x = x
                waypoint.position.y = y
                waypoint.position.z = z

            if target_distance_object != np.inf:
                # Recalculate target speed for all the waypoints using the closest object
                zero_speeds_onwards = False
                approaching_speed = min(target_object_speed, 0.0)
                deceleration = collision_point_deceleration
                for i, wp in enumerate(local_path_msg.waypoints):

                    # once we get zero speed, keep it that way
                    if zero_speeds_onwards:
                        wp.speed = 0.0
                        continue

                    if i > 0:
                        target_distance_object -= get_distance_between_two_points_2d(local_path_msg.waypoints[i-1].position, local_path_msg.waypoints[i].position)

                    target_speed_object = max(0.0, approaching_speed + math.sqrt(max(0.0, target_object_speed**2 + 2 * deceleration * target_distance_object)))

                    # overwrite target speed of wp
                    wp.speed = min(target_speed_object, wp.speed)

                    # from stop point onwards all speeds are set to zero
                    if math.isclose(wp.speed, 0.0):
                        zero_speeds_onwards = True
            
            # Publish the local path message with the calculated values
            local_path_out = LocalPath()
            local_path_out.header = local_path_msg.header
            local_path_out.waypoints = local_path_msg.waypoints
            local_path_out.path_id = self._physical_offset(int(chosen_parallel_path_id))
            local_path_out.target_object_distance = float(target_object_distance)
            local_path_out.target_object_speed = float(target_object_speed)
            local_path_out.is_blocked = bool(target_distance_object != np.inf)
            local_path_out.stopping_point_distance = float(stopping_point_distance)
            local_path_out.collision_point_category = int(collision_point_category)
            self.publish_local_path(local_path_out)

        except Exception as e:
            rospy.logerr_throttle(10, "%s - Exception in callback: %s", rospy.get_name(), traceback.format_exc())

    def publish_local_path(self, local_path_out):
        self.local_path_pub.publish(local_path_out)
        if len(local_path_out.waypoints) > 1:
            category = local_path_out.collision_point_category
        else:
            category = CollisionPoints.NO_PATH
        self.log_message_pub.publish(Log(message=CollisionPoints.COLLISION_POINT_CATEGORY_CAPTION[category], color=CollisionPoints.COLLISION_POINT_CATEGORY_COLOR[category]))

    def _physical_offset(self, path_id):
        return -(path_id + 1) // 2 if path_id % 2 == 1 else path_id // 2

    def _path_from_offset(self, offset):
        return -offset * 2 - 1 if offset < 0 else offset * 2

    def choose_path(self, parallel_paths, current_position, current_speed, waypoint_speed, collision_points):
        """ Choose the best path among parallel paths based on target speed"""
        target_object_distances = np.zeros(self.num_parallel_paths)
        target_object_speeds = np.zeros(self.num_parallel_paths)
        stopping_point_distances = np.zeros(self.num_parallel_paths)
        collision_point_categories = np.full(self.num_parallel_paths, CollisionPoints.NO_OBSTACLES)
        collision_point_decelerations = np.zeros(self.num_parallel_paths)
        target_distances_objects = np.full(self.num_parallel_paths, np.inf)
        current_target_speeds = np.full(self.num_parallel_paths, -1.0)

        # extract collision points
        collision_points_shapely = shapely.points(structured_to_unstructured(collision_points[['x', 'y', 'z']]))

        for path_id, path in enumerate(parallel_paths):
            # if no collision points at all
            if collision_points.size == 0:
                current_target_speeds[path_id] = waypoint_speed
                continue

            ego_distance_from_path_start = path.project(current_position)
            trimmed_path = linesubstring(path, ego_distance_from_path_start + self.distance_to_car_front, path.length)

            wide_safety_buffer = trimmed_path.buffer(self.wide_safety_box_width / 2.0, cap_style="flat")
            narrow_safety_buffer = path.buffer(self.narrow_safety_box_width / 2.0, cap_style="flat")

            with warnings.catch_warnings(): # Ignore shapely runtime warnings during the union operation
                warnings.filterwarnings("ignore", category=RuntimeWarning, module="shapely")
                # Use smaller safety buffer until car front
                wide_safety_buffer = wide_safety_buffer.union(narrow_safety_buffer)

            shapely.prepare(wide_safety_buffer)
            shapely.prepare(narrow_safety_buffer)

            # use smaller safety buffer for lane boundary and trajectory collision points
            is_narrow_collision_point = (
                (collision_points['category'] == CollisionPoints.LANE_BOUNDARY) |
                (collision_points['category'] == CollisionPoints.COLLIDING_TRAJECTORY)
            )
            narrow_collision_points_shapely = collision_points_shapely[is_narrow_collision_point]
            wide_collision_points_shapely = collision_points_shapely[~is_narrow_collision_point]

            narrow_collision_points_mask = narrow_safety_buffer.intersects(narrow_collision_points_shapely)
            wide_collision_points_mask = wide_safety_buffer.intersects(wide_collision_points_shapely)

            collision_points_mask = np.zeros(len(collision_points_shapely), dtype=bool)
            collision_points_mask[is_narrow_collision_point] = narrow_collision_points_mask
            collision_points_mask[~is_narrow_collision_point] = wide_collision_points_mask

            # if no collision points in the safety box
            if np.all(~collision_points_mask):
                current_target_speeds[path_id] = waypoint_speed
                continue

            # filter out collision points that are outside the safety box
            collision_points_filtered = collision_points[collision_points_mask]
            collision_points_shapely_filtered = collision_points_shapely[collision_points_mask]

            collision_point_distances = path.project(collision_points_shapely_filtered)

            # calculate deceleration for every collision point
            deceleration_distances = collision_point_distances - self.distance_to_car_front - ego_distance_from_path_start
            required_decelerations = (current_speed**2) / np.maximum(2.0 * deceleration_distances, 0.001)
            deceleration_exceeded_mask = (required_decelerations > collision_points_filtered['deceleration_limit'])

            for i in np.where(deceleration_exceeded_mask)[0]:
                self.log_message_pub.publish(Log(message = CollisionPoints.COLLISION_POINT_CATEGORY_IGNORE_CAPTION[collision_points_filtered[i]['category']], color = "white", instant = True))
                rospy.logwarn_throttle(3, f"{rospy.get_name()} - {CollisionPoints.COLLISION_POINT_CATEGORY_IGNORE_CAPTION[collision_points_filtered[i]['category']]} ({deceleration_distances[i]:.1f} m) - deceleration of {required_decelerations[i]:.2f} m/s2 exceeds limit ({collision_points_filtered[i]['deceleration_limit']:.2f} m/s2), ignore!")

            # if all collision points exceed deceleration
            if np.all(deceleration_exceeded_mask):
                current_target_speeds[path_id] = waypoint_speed
                continue

            # filter out collision points that exceed deceleration limit
            collision_points_filtered = collision_points_filtered[~deceleration_exceeded_mask]
            collision_point_distances = collision_point_distances[~deceleration_exceeded_mask]

            # project speed to path for every collision point
            collision_point_path_headings = calculate_headings(shapely.get_coordinates(path), collision_point_distances)
            collision_point_speeds = collision_points_filtered['vx'] * np.cos(collision_point_path_headings) + collision_points_filtered['vy'] * np.sin(collision_point_path_headings)
            collision_point_braking_distances = collision_points_filtered['distance_to_stop']

            # calculate target speed for every collision point
            # for approaching objects (v < -stopped_speed_limit), approaching speed accounts for the object closing distance during ego braking
            target_distances = collision_point_distances - np.maximum(collision_point_braking_distances, self.braking_reaction_time * np.abs(collision_point_speeds)) - self.distance_to_car_front
            approaching_speeds = np.minimum(collision_point_speeds, 0.0)
            decelerations = collision_points_filtered['deceleration']
            target_speeds = np.maximum(0.0, approaching_speeds + np.sqrt(np.maximum(0.0, collision_point_speeds**2 + 2 * decelerations * target_distances)))

            # select min target speed (treat equally speeds below stopped_speed_limit) among the ones that do not exceed deceleration limit and is closest to the ego vehicle
            min_target_speed = max(self.stopped_speed_limit, np.min(target_speeds))
            mask = target_speeds <= min_target_speed

            # from all collision points whose speed is less-than-equal min_target_speed choose the closest collision point, this ensures the stability of the stopping wall visualization
            adjusted_distances = np.where(mask, collision_point_distances, np.inf)
            min_value_index = np.argmin(adjusted_distances)

            # keep track of the parameters of the parallel path
            target_object_distances[path_id] = collision_point_distances[min_value_index] - self.distance_to_car_front - ego_distance_from_path_start
            target_object_speeds[path_id] = collision_point_speeds[min_value_index]
            stopping_point_distances[path_id] = collision_point_distances[min_value_index] - collision_point_braking_distances[min_value_index]
            collision_point_categories[path_id] = collision_points_filtered[min_value_index]["category"]
            collision_point_decelerations[path_id] = collision_points_filtered[min_value_index]["deceleration"]
            target_distances_objects[path_id] = target_distances[min_value_index]
            current_target_speeds[path_id] = min(waypoint_speed, target_speeds[min_value_index])

            # block swerving if the current path is blocked by a fast-moving or misaligned object
            if path_id == self.last_parallel_path_id:
                collision_point_speed = math.hypot(collision_points_filtered[min_value_index]['vx'], collision_points_filtered[min_value_index]['vy'])
                if collision_point_speed > self.swerving_blocked_speed_limit:
                    break
                if collision_point_speed > self.stopped_speed_limit:
                    alignment_angle = math.acos(min(abs(target_object_speeds[path_id]) / collision_point_speed, 1.0))
                    if alignment_angle > self.swerving_alignment_angle_limit:
                        break

        # find paths that are within the swerving speed tolerance of the max speed
        max_speed = np.max(current_target_speeds)
        suitable_paths_mask = (max_speed - current_target_speeds <= self.swerving_speed_tolerance) & (current_target_speeds >= 0.0)
        suitable_paths_ids = np.where(suitable_paths_mask)[0]

        # from suitable paths choose the one that has a counter value above the minimum limit and is closest to the center path
        valid_mask = self.parallel_path_counters[suitable_paths_ids] >= self.swerving_counter_min_limit
        if np.any(valid_mask):
            target_path_id = suitable_paths_ids[np.argmax(valid_mask)]
        elif self.last_target_path_id in suitable_paths_ids:
            target_path_id = self.last_target_path_id
        else:
            # if last path is not among suitable paths, choose the first suitable path (closest to center)
            target_path_id = suitable_paths_ids[0]

        # clamp to one physical step at a time
        chosen_path_id = target_path_id
        current_offset = self._physical_offset(self.last_parallel_path_id)
        target_offset = self._physical_offset(target_path_id)
        if target_offset != current_offset:
            if target_offset > current_offset:
                chosen_path_id = self._path_from_offset(current_offset + 1)
            else:
                chosen_path_id = self._path_from_offset(current_offset - 1)

        if target_path_id == self.last_target_path_id:
            # if the target parallel path remains the same, then increase the counter of suitable paths and reset all others to zero
            self.parallel_path_counters = np.where(suitable_paths_mask, self.parallel_path_counters + 1, 0)
        else:
            # if the target parallel path changes, then reset all counters to zero and increase the suitable paths counters
            self.parallel_path_counters = np.where(suitable_paths_mask, 1, 0)
            self.last_target_path_id = target_path_id
        
        self.last_parallel_path_id = chosen_path_id

        return target_object_distances[chosen_path_id], target_object_speeds[chosen_path_id], stopping_point_distances[chosen_path_id], collision_point_categories[chosen_path_id], collision_point_decelerations[chosen_path_id], target_distances_objects[chosen_path_id], chosen_path_id
            
    def run(self):
        rospy.spin()


def find_best_connection_point(start_point, start_heading, path_arr, path_headings,
                               angles_difference_weight=8.0, align_angles_weight=4.0,
                               distance_weight=0.0002, cumulative_heading_weight=1.0):
    """
    Find best point on path to connect smoothly based on distance and orientation.
    :param start_point: numpy array of shape (2,) representing the starting point (x, y)
    :param start_heading: float representing the yaw angle in radians
    :param path_arr: numpy array of shape (N, 3) representing the path points (x, y, z)
    :param path_headings: numpy array of shape (N,) representing the headings of the path points in radians
    :param angles_difference_weight: weight for the angle difference cost
    :param align_angles_weight: weight for the alignment angle cost
    :param distance_weight: weight for the distance cost
    :param cumulative_heading_weight: weight for the cumulative heading change cost
    :return: index of the best connection point on the path
    """

    start_pt = np.array([start_point.x, start_point.y])
    target_pts = path_arr[1:-1, :2] # process all points except first and last and remove z coordinate

    vecs_to_target = target_pts - start_pt
    angles_to_target = np.arctan2(vecs_to_target[:, 1], vecs_to_target[:, 0])

    # Angle differences between start point heading and straight paths to target points
    diff_angles = get_angle_between_two_headings(start_heading,  angles_to_target)

    # Alignment angles between target point headings and paths to target points
    align_angles = get_angle_between_two_headings(angles_to_target, path_headings[1:-1])

    # Distances from start point to target points
    distances = np.linalg.norm(vecs_to_target, axis=1)

    # Cumulative heading change along the path from first to each candidate point
    heading_changes = get_angle_between_two_headings(path_headings[1:-2], path_headings[2:-1])
    cumulative_change = np.insert(np.cumsum(heading_changes), 0, 0.0)

    # Costs based on angle differences, alignment angles, distances, and cumulative heading change
    costs = (angles_difference_weight * diff_angles**2 +
                align_angles_weight * align_angles**2 +
                distance_weight * distances**2 +
                cumulative_heading_weight * cumulative_change**2)
    
    min_idx = np.argmin(costs)
    
    return int(min_idx + 1) # add 1 because the first point was skipped

def calculate_join_path(start_point, start_heading, path, path_headings):
    """
    Calculates ego path to global path using a clothoid
    :param start_point: Shapely point representing the starting point
    :param start_heading: float representing the yaw angle in radians
    :param path: shapely LineString representing the path to join to
    :param path_headings: list of float representing the headings of the path points in radians
    :return: list of waypoints representing the joined path,
    """
    # If the local path has less than 3 points, return it as is
    if shapely.get_num_points(path) < 3:
        return path

    path_arr = shapely.get_coordinates(path, include_z=True)

    # Find the best connection point on global path
    idx = find_best_connection_point(start_point, start_heading, path_arr, path_headings)
    assert idx > 0 and idx < len(path_arr) - 1, "Clothoid connection point index out of range"

    connect_point = path_arr[idx]
    connect_point_heading = path_headings[idx]

    # Calculate a clothoid cuve from the start point to the found connect point
    clothoid = pyclothoids.Clothoid.G1Hermite(start_point.x, start_point.y, start_heading, connect_point[0], connect_point[1], connect_point_heading)
    xs, ys = clothoid.SampleXY(idx + 1)
    path_arr[:idx + 1, 0] = xs
    path_arr[:idx + 1, 1] = ys

    return shapely.linestrings(path_arr)

def generate_parallel_paths(start_point, start_heading, local_path, num_paths, path_spacing):
    """
    Generate multiple parallel paths at a certain spacing from the local path
    :param start_point: Shapely point representing the starting point
    :param start_heading: float representing the start point yaw angle in radians
    :param local_path: LineString object representing the local path
    :param num_paths: number of paths to generate (should be odd)
    :param path_spacing: spacing between the paths (m)
    :return: list of parallel paths as shapely LineStrings
    """

    # Precompute headings for the local path
    path_headings = calculate_headings(shapely.get_coordinates(local_path))

    # Calculate join spline for the center path
    center_path = calculate_join_path(start_point, start_heading, local_path, path_headings)
    parallel_paths = [center_path]

    if num_paths < 2 or num_paths % 2 == 0:
        return parallel_paths

    # Build distances: -spacing, +spacing, -2*spacing, +2*spacing, ...
    num_offsets = (num_paths - 1) // 2
    distances = []
    for i in range(num_offsets):
        distances.append(-path_spacing * (i + 1))
        distances.append(path_spacing * (i + 1))

    # Batch offset single linestring at multiple distances
    coords = shapely.get_coordinates(local_path, include_z=True)
    offset_paths = shapely.linestrings(offset_curve(coords, distances))

    # Calculate join paths for each offset
    for offset_path in offset_paths:
        parallel_path = calculate_join_path(start_point, start_heading, offset_path, path_headings)
        parallel_paths.append(parallel_path)

    return parallel_paths

if __name__ == '__main__':
    rospy.init_node('swerve_planner')
    node = SwervePlanner()
    node.run()