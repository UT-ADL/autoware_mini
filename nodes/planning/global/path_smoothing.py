#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plt
from autoware_msgs.msg import Lane, Waypoint
from helpers import get_orientation_from_yaw


class PathSmoothing:

    def __init__(self):

        # Parameters
        self.waypoint_interval = rospy.get_param("~waypoint_interval", 1.0)
        self.curvature_step = rospy.get_param("~curvature_step", 1)

        # Publishers
        self.smoothed_path_pub = rospy.Publisher('smoothed_path', Lane, queue_size=1)

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

        # Calculate yaw angle for path and add last yaw angle to the end of the array
        yaw = np.arctan2( np.diff(y_new), np.diff(x_new))
        yaw = np.append(yaw, yaw[-1])

        # TODO instead calc radius and then centrifugal force?
        # Curvature
        curvature = calculate_curvature(x_new, y_new, self.curvature_step)

        # TODO implement curvature based or centrifugal force based speed


        speed_interpolated = np.interp(new_distances, distances, speed)
        # TODO change when curvature based speed is implemented
        speed_curvature = np.full(new_distances.shape, 40.0)
        speed_new = np.minimum(speed_interpolated, speed_curvature)


        # TODO: remove or add param for debug visualization
        # debug_visualize(x_path, y_path, z_path, blinker, x_new, y_new, z_new, blinker_new, distances, new_distances)

        # Stack arrays for faster accessing
        smoothed_path_array = np.stack((x_new, y_new, z_new, blinker_new, speed_new, yaw), axis=1)
        return smoothed_path_array

    def publish_smoothed_path(self, smoothed_path, output_frame):
        # create lane message
        lane = Lane()
        lane.header.frame_id = output_frame
        lane.header.stamp = rospy.Time.now()
        lane.waypoints = [self.create_waypoint(x, y, z, blinker, speed, yaw) for x, y, z, blinker, speed, yaw in smoothed_path]

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

def calculate_curvature(x, y, step):
    # calculate curvature for a given path
    # x, y: path coordinates
    # step: step size for calculating curvature

    # calculate first and second derivative
    dx = np.gradient(x, step)
    dy = np.gradient(y, step)
    ddx = np.gradient(dx, step)
    ddy = np.gradient(dy, step)

    # calculate curvature
    curvature = (dx * ddy - dy * ddx) / (dx**2 + dy**2)**(3/2)

    return curvature



def debug_visualize(x_path, y_path, z_path, blinker, x_new, y_new, z_new, blinker_new, distances, new_distances):
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


if __name__ == '__main__':
    rospy.init_node('path_smoothing')
    node = PathSmoothing()
    node.run()