#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
from autoware_msgs.msg import Lane, Waypoint
from helpers import get_orientation_from_yaw


class PathSmoothing:

    def __init__(self):

        # Parameters
        self.waypoint_interval = rospy.get_param("~waypoint_interval", 1.0)
        self.adjust_speeds_in_curves = rospy.get_param("~adjust_speeds_in_curves", True)
        self.radius_calc_neighbour_index = rospy.get_param("~radius_calc_neighbour_index", 4)
        self.lateral_acceleration_limit = rospy.get_param("~lateral_acceleration_limit", 3.0)

        # Publishers
        self.smoothed_path_pub = rospy.Publisher('smoothed_path', Lane, queue_size=1, latch=True)

        # Subscribers
        self.global_path_sub = rospy.Subscriber('global_path', Lane, self.global_path_callback, queue_size=1)


    def global_path_callback(self, msg):
        if len(msg.waypoints) == 0:
            # create marker_array to delete all visualization markers
            self.publish_smoothed_path(np.array([]), msg.header.frame_id)
            return

        # extract all waypoint attributes with one loop
        waypoints_array = np.array([(
                wp.pose.pose.position.x,
                wp.pose.pose.position.y,
                wp.pose.pose.position.z,
                wp.wpstate.steering_state,
                wp.twist.twist.linear.x
            ) for wp in msg.waypoints])

        smoothed_path_array = self.smooth_global_path(waypoints_array)

        self.publish_smoothed_path(smoothed_path_array, msg.header.frame_id)


    def smooth_global_path(self, waypoints_array):

        # extract into arrays
        xy_path = waypoints_array[:,:2]
        x_path = waypoints_array[:,0]
        y_path = waypoints_array[:,1]
        z_path = waypoints_array[:,2]
        blinker = waypoints_array[:,3]
        speed = waypoints_array[:,4]

        # distance differeneces between points
        distances = np.cumsum(np.sqrt(np.sum(np.diff(xy_path, axis=0)**2, axis=1)))
        # add 0 to the beginning of the array
        distances = np.insert(distances, 0, 0)
        # create new distances at fixed intervals
        new_distances = np.linspace(0, distances[-1], num=int(distances[-1] / self.waypoint_interval))

        #### INTERPOLATE  ####

        # interpolate x_new, y_new, z_new
        x_new = np.interp(new_distances, distances, x_path)
        y_new = np.interp(new_distances, distances, y_path)
        z_new = np.interp(new_distances, distances, z_path)

        # Blinkers
        blinker_left = np.rint(np.interp(new_distances, distances, (blinker == 1).astype(np.uint8)))
        blinker_right = np.rint(np.interp(new_distances, distances, (blinker == 2).astype(np.uint8)))
        # combine arrays into one so that left = 1, right = 2, straight = 3
        blinker_new = np.maximum(blinker_left, blinker_right * 2)
        blinker_new[blinker_new == 0] = 3

        # Yaw - calculate yaw angle for path and add last yaw angle to the end of the array
        yaw = np.arctan2(np.diff(y_new), np.diff(x_new))
        yaw = np.append(yaw, yaw[-1])

        # Speed
        # TODO: when map based speed is implemented check if gives reasonable results
        speed_interpolated = np.interp(new_distances, distances, speed)
        speed_new = speed_interpolated

        if self.adjust_speeds_in_curves:
            # Calculate speed limit based on lateral acceleration limit
            radius = calculate_radius_with_step_n(x_new, y_new, self.radius_calc_neighbour_index)
            speed_radius = np.sqrt(self.lateral_acceleration_limit * np.abs(radius))
            speed_new = np.fmin(speed_interpolated, speed_radius)

        # TODO: remove or add param for debug visualization
        # debug_visualize(x_path, y_path, z_path, blinker, x_new, y_new, z_new, blinker_new, distances, new_distances, speed, speed_new)

        # Stack
        smoothed_path_array = np.stack((x_new, y_new, z_new, blinker_new, speed_new, yaw), axis=1)

        return smoothed_path_array

    def publish_smoothed_path(self, smoothed_path, output_frame):
        # create lane message
        lane = Lane()
        lane.header.frame_id = output_frame
        lane.header.stamp = rospy.Time.now()
        lane.waypoints = [self.create_waypoint(*wp) for wp in smoothed_path]

        self.smoothed_path_pub.publish(lane)

    def create_waypoint(self, x, y, z, blinker, speed, yaw):
        # create waypoint
        waypoint = Waypoint()
        waypoint.pose.pose.position.x = x
        waypoint.pose.pose.position.y = y
        waypoint.pose.pose.position.z = z
        waypoint.wpstate.steering_state = int(blinker)
        waypoint.twist.twist.linear.x = speed
        waypoint.pose.pose.orientation = get_orientation_from_yaw(yaw)

        return waypoint

    def run(self):
        rospy.spin()

def calculate_radius_with_step_n(x, y, n):
    # calculate radius of the circle using 3 points
    # x, y: path coordinates in array
    # n: step sixe to select points used for calculating radius
    # https://en.wikipedia.org/wiki/Circumscribed_circle#Cartesian_coordinates

    radius = np.empty_like(x)
    # calculate radius for each point in the path
    radius[n:-n] = np.abs((x[:-2*n] - x[2*n:])**2 + (y[:-2*n] - y[2*n:])**2) / (2 * ((x[:-2*n] - x[n:-n]) * (y[2*n:] - y[n:-n]) - (x[2*n:] - x[n:-n]) * (y[:-2*n] - y[n:-n])))
    # Replace n values from beginning of the array with the first value
    radius[:n] = radius[n]
    # Replace n values from end of the array with the last value
    radius[-n:] = radius[-(n+1)]

    return radius

def debug_visualize(x_path, y_path, z_path, blinker, x_new, y_new, z_new, blinker_new, distances, new_distances, speed, speed_new):
    # plot 3 figures
    # 1. x-y path with old and new waypoints
    # 2. z path with old and new waypoints
    # 3. blinker path with old and new waypoints

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

    # new plot for blinkers
    fig = plt.figure(figsize=(10, 15))
    ax = fig.subplots()
    ax.scatter(new_distances, blinker_new, color = 'red', marker = 'x', alpha = 0.5, label = 'blinker interpolated')
    ax.plot(new_distances, blinker_new, color = 'red', alpha = 0.5, label = 'blinker interpolated')
    ax.scatter(distances, blinker, color = 'blue', alpha = 0.5, label = 'blinker old')
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