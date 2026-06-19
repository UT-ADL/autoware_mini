#!/usr/bin/env python3

import rospy
import numpy as np
from autoware_mini.msg import Path, Waypoint
from autoware_mini.path import PathWrapper
from autoware_mini.geometry import calculate_radius

class PathSmoothing:

    def __init__(self):

        # Parameters
        self.waypoint_interval = rospy.get_param("waypoint_interval")
        self.adjust_speeds_in_curves = rospy.get_param("~adjust_speeds_in_curves")
        self.adjust_speeds_using_deceleration = rospy.get_param("~adjust_speeds_using_deceleration")
        self.adjust_endpoint_speed_to_zero = rospy.get_param("~adjust_endpoint_speed_to_zero")
        self.default_deceleration = rospy.get_param("default_deceleration")
        self.speed_averaging_window = rospy.get_param("~speed_averaging_window")
        self.radius_calc_neighbour_index = rospy.get_param("~radius_calc_neighbour_index")
        self.lateral_acceleration_limit = rospy.get_param("~lateral_acceleration_limit")
        self.output_debug_info = rospy.get_param("~output_debug_info")

        # Publishers
        self.smoothed_path_pub = rospy.Publisher('global_path', Path, queue_size=10, latch=True, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber('lane_change_global_path', Path, self.global_path_callback, queue_size=None, tcp_nodelay=True)


    def global_path_callback(self, msg):
        if len(msg.waypoints) < 2:
            # create empty path, nothing to smooth
            self.publish_smoothed_path([], msg.header.frame_id)
            return

        path = PathWrapper(msg.waypoints)

        # resample at fixed intervals and insert stop line distances to preserve their exact locations
        mask = path.stop_line_types > 0
        sl_distances, sl_ids, sl_types = path.distances[mask], path.stop_line_ids[mask], path.stop_line_types[mask]
        new_distances = np.arange(0, path.distances[-1], self.waypoint_interval)
        new_distances = np.append(new_distances, path.distances[-1])
        new_distances = np.union1d(new_distances, sl_distances)

        # interpolate new centerline points
        path_point_new = path.get_point_at_distance(new_distances)
        path_points_new_arr = np.array([(p.x, p.y, p.z) for p in path_point_new])

        # turn signals
        turn_signal_new = path.get_turn_signal_at_distance(new_distances)

        # priorities
        priority_new = path.get_priority_at_distance(new_distances)

        # left and right boundary points
        l_bound_points_new = path.get_left_boundary_point_at_distance(new_distances)
        r_bound_points_new = path.get_right_boundary_point_at_distance(new_distances)

        # nearest neighbour interpolation for boundary types
        l_type_new = path.get_left_boundary_type_at_distance(new_distances)
        r_type_new = path.get_right_boundary_type_at_distance(new_distances)

        # speed
        speed_new = path.get_speed_at_distance(new_distances)

        if self.adjust_speeds_in_curves and len(speed_new) >= 3:
            # Calculate speed limit based on lateral acceleration limit
            radius = calculate_radius(path_points_new_arr[:, 0], path_points_new_arr[:, 1], self.radius_calc_neighbour_index)
            speed_radius = np.sqrt(self.lateral_acceleration_limit * radius)
            speed_new = np.fmin(speed_new, speed_radius)

        # loop over array backwards and forwards to adjust speeds using the deceleration limit
        if self.adjust_speeds_using_deceleration:
            # backward loop
            for i in range(len(speed_new) - 2, 0, -1):
                accel_ds = 2 * self.default_deceleration * (new_distances[i + 1] - new_distances[i])
                speed_new[i] = min(speed_new[i], np.sqrt(speed_new[i + 1]**2 + accel_ds))
            # forward loop
            for i in range(1, len(speed_new)):
                accel_ds = 2 * self.default_deceleration * (new_distances[i] - new_distances[i - 1])
                speed_new[i] = min(speed_new[i], np.sqrt(speed_new[i - 1]**2 + accel_ds))

        if self.speed_averaging_window > 1 and len(speed_new) >= self.speed_averaging_window:
            # average array values using window size of n
            speed_new = np.convolve(speed_new, np.full(self.speed_averaging_window, 1.0/self.speed_averaging_window), mode='same')
            # replace n/2 values at the beginning and end of the array with the n+1 and n-1 values respectively
            speed_new[:int(self.speed_averaging_window/2)] = speed_new[int(self.speed_averaging_window/2)]
            speed_new[-int(self.speed_averaging_window/2):] = speed_new[-int(self.speed_averaging_window/2)-1]

        if self.adjust_endpoint_speed_to_zero:
            # set last point speed to zero
            speed_new[-1] = 0.0

            # adjust speed using deceleration limit and actual waypoint distances
            # backward loop - end point
            for i in range(len(speed_new) - 2, 0, -1):
                accel_ds = 2 * self.default_deceleration * (new_distances[i + 1] - new_distances[i])
                adjusted_speed = np.sqrt(speed_new[i + 1]**2 + accel_ds)
                if adjusted_speed > speed_new[i]:
                    break
                speed_new[i] = adjusted_speed

        # stop line fields - set at segment boundary positions
        stop_line_id_new = np.zeros(len(new_distances), dtype=int)
        stop_line_type_new = np.zeros(len(new_distances), dtype=int)
        sl_indices = np.searchsorted(new_distances, sl_distances)
        stop_line_id_new[sl_indices] = sl_ids
        stop_line_type_new[sl_indices] = sl_types

        if self.output_debug_info:
            debug_plots_path_smoothing(path.centerline_array[:, 0], path.centerline_array[:, 1], path.centerline_array[:, 2],
                                       path.turn_signals, path_points_new_arr[:, 0], path_points_new_arr[:, 1], path_points_new_arr[:, 2],
                                       turn_signal_new, path.distances, new_distances, path.speeds, speed_new)

        waypoints = [
            self.create_waypoint(*wp)
            for wp in zip(path_point_new,
                          turn_signal_new, speed_new,
                          l_bound_points_new,
                          r_bound_points_new,
                          l_type_new, r_type_new, priority_new,
                          stop_line_id_new, stop_line_type_new)
        ]

        self.publish_smoothed_path(waypoints, msg.header.frame_id)

    def publish_smoothed_path(self, waypoints, output_frame):
        # create path message
        path = Path()
        path.header.frame_id = output_frame
        path.header.stamp = rospy.Time.now()
        path.waypoints = waypoints

        self.smoothed_path_pub.publish(path)

    def create_waypoint(self, position, turn_signal, speed, l_point, r_point, l_type, r_type, priority, stop_line_id=0, stop_line_type=0):
        # create waypoint
        waypoint = Waypoint()
        waypoint.position = position
        waypoint.turn_signal = int(turn_signal)
        waypoint.speed = float(speed)
        waypoint.left_boundary_point = l_point
        waypoint.right_boundary_point = r_point
        waypoint.left_boundary_type = int(l_type)
        waypoint.right_boundary_type = int(r_type)
        waypoint.priority = bool(priority)
        waypoint.stop_line_id = int(stop_line_id)
        waypoint.stop_line_type = int(stop_line_type)

        return waypoint

    def run(self):
        rospy.spin()

def debug_plots_path_smoothing(x_path, y_path, z_path, turn_signal, x_new, y_new, z_new, turn_signal_new, distances, new_distances, speed, speed_new):

    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(10, 15))
    ax = fig.subplots()
    ax.scatter(x_path,y_path, color = 'blue')
    ax.scatter(x_new, y_new, color = 'red', marker = 'x', alpha = 0.5, label = 'interpolated')
    plt.legend()
    plt.show()

    # new plot for heights
    fig = plt.figure(figsize=(10, 15))
    ax = fig.subplots()
    ax.scatter(new_distances, z_new, color = 'red', marker = 'x', alpha = 0.5, label = 'height interpolated')
    ax.plot(new_distances, z_new, color = 'red', alpha = 0.5, label = 'height interpolated')
    ax.scatter(distances, z_path, color = 'blue', alpha = 0.5, label = 'height old')
    plt.legend()
    plt.show()

    # new plot for turn signals
    fig = plt.figure(figsize=(10, 15))
    ax = fig.subplots()
    ax.scatter(new_distances, turn_signal_new, color = 'red', marker = 'x', alpha = 0.5, label = 'turn signal interpolated')
    ax.plot(new_distances, turn_signal_new, color = 'red', alpha = 0.5, label = 'turn signal interpolated')
    ax.scatter(distances, turn_signal, color = 'blue', alpha = 0.5, label = 'turn signal old')
    plt.legend()
    plt.show()

    # new plot for speed
    fig = plt.figure(figsize=(10, 15))
    ax = fig.subplots()
    ax.scatter(new_distances, speed_new * 3.6, color = 'red', marker = 'x', alpha = 0.5, label = 'speed interpolated')
    ax.plot(new_distances, speed_new * 3.6, color = 'red', alpha = 0.5, label = 'speed interpolated')
    ax.scatter(distances, speed * 3.6, color = 'blue', alpha = 0.5, label = 'speed old')
    plt.legend()
    plt.show()


if __name__ == '__main__':
    rospy.init_node('path_smoothing')
    node = PathSmoothing()
    node.run()