#!/usr/bin/env python3

import rospy
import numpy as np
import math

from geometry_msgs.msg import Point
from autoware_mini.msg import Path, Waypoint

from autoware_mini.geometry import get_heading_between_two_points, get_point_using_heading_and_distance, \
    get_distance_between_two_points_2d, get_angle_between_three_points, calculate_points_on_bezier_curve


class LaneChangePlanner:

    def __init__(self):

        # Parameters
        self.waypoint_interval = rospy.get_param("waypoint_interval")
        self.lane_change_base_length = rospy.get_param("lane_change_base_length")
        self.lane_change_perlane_length = rospy.get_param("lane_change_perlane_length")

        # Publishers
        self.lane_change_path_pub = rospy.Publisher('lane_change_global_path', Path, queue_size=10, latch=True, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber('lanelet2_global_path', Path, self.global_path_callback, queue_size=None, tcp_nodelay=True)


    def global_path_callback(self, msg):
        path = Path()
        path.header = msg.header

        waypoints = self.create_lane_change_paths(msg.waypoints)
        if waypoints is None:
            rospy.logerr("%s - path contained an impossible lane change!", rospy.get_name())
        else:
            path.waypoints = waypoints
        self.lane_change_path_pub.publish(path)

    def create_lane_change_paths(self, waypoints):
        idx = 0
        while idx < len(waypoints):
            # Check for lane change
            if waypoints[idx].lanechange_state > 0:
                start_idx = idx
                end_idx = None

                # Check that the lane change waypoint is not the last waypoint of the path
                if start_idx + 1 == len(waypoints):
                    return None
                start_point = waypoints[start_idx].position

                # Calculate a point 1 unit away in the driving direction for lane change angle calculation
                start_point_heading = waypoints[start_idx].heading
                other_point = get_point_using_heading_and_distance(waypoints[start_idx].position, start_point_heading, 1)

                blinker_state = waypoints[start_idx].blinker_state

                # Skip all lane change waypoints
                while idx < len(waypoints) and waypoints[idx].lanechange_state > 0:
                    lanechange_state = waypoints[idx].lanechange_state
                    idx += 1

                # Skip all non lane change waypoints until enough distance to perform lane change
                while idx < len(waypoints):
                    current_point = waypoints[idx].position
                    # Get the diagonal distance of the lane change
                    d = get_distance_between_two_points_2d(start_point, current_point)

                    # Calculate the lane change angle
                    a = get_angle_between_three_points(other_point, start_point, current_point)

                    # Calculate lane change length
                    given_lanechange_length = self.lane_change_base_length + lanechange_state * self.lane_change_perlane_length

                    # Use the angle to check that the lane change doesn't happen behind us
                    # Multiply the diagonal distance with cos(a) to get the parallel distance of the lane change
                    if abs(a) < math.pi/2 and d * math.cos(a) >= given_lanechange_length:
                        end_idx = idx
                        break

                    idx += 1

                # End of path before lane change is complete
                if end_idx is None:
                    return None

                # Replace section of waypoints with spline
                spline = self.calculate_lane_change_spline(waypoints[start_idx], waypoints[end_idx], 
                                                           given_lanechange_length, blinker_state)

                waypoints = waypoints[:start_idx] + spline + waypoints[end_idx+1:]

                # Advance the index beyond the lane change
                idx = start_idx + len(spline)

            else:
                idx += 1

        return waypoints

    def calculate_lane_change_spline(self, start_waypoint, end_waypoint, lanechange_length, blinker_state):

        ##################################################################
        # Calculate Bezier curve control points p0, p1, p2, p3
        ##################################################################

        control_point1 = get_point_using_heading_and_distance(start_waypoint.position, start_waypoint.heading, lanechange_length / 3)
        control_point2 = get_point_using_heading_and_distance(end_waypoint.position, end_waypoint.heading + math.pi, lanechange_length / 3)

        bezier_points = calculate_points_on_bezier_curve(
            start_waypoint.position,
            control_point1, control_point2, 
            end_waypoint.position,
            int(lanechange_length // self.waypoint_interval)
        )

        ##################################################################
        # Create lane change waypoints
        ##################################################################

        # Calculate the distance of each Bezier point from the benning of the spline
        lane_change_wp_distances = np.cumsum(np.sqrt(np.sum(np.diff(bezier_points, axis=0)**2, axis=1)))
        # Add 0 to the beginning of the array
        lane_change_wp_distances = np.insert(lane_change_wp_distances, 0, 0)
        # Use the first and last distance as datapoints
        distance_datapoints  = np.array([0, lane_change_wp_distances[-1]])
        
        # Speed interpolation
        speed_datapoints = np.array([start_waypoint.speed, end_waypoint.speed])
        speed = np.interp(lane_change_wp_distances, distance_datapoints, speed_datapoints)

        # Left lane width interpolation
        lw_datapoints = np.array([start_waypoint.left_width, end_waypoint.left_width])
        lw = np.interp(lane_change_wp_distances, distance_datapoints, lw_datapoints)

        # Right lane width interpolation
        lw_datapoints = np.array([start_waypoint.right_width, end_waypoint.right_width])
        rw = np.interp(lane_change_wp_distances, distance_datapoints, lw_datapoints)

        # z-coordinate interpolation
        z_datapoints = np.array([start_waypoint.position.z, end_waypoint.position.z])
        z_coords = np.interp(lane_change_wp_distances, distance_datapoints, z_datapoints)

        waypoints = []
        for i in range(len(bezier_points)):
            if i == len(bezier_points) - 1:
                heading = end_waypoint.heading
            else:
                point = Point(x=bezier_points[i, 0], y=bezier_points[i, 1])
                next_point = Point(x=bezier_points[i+1, 0], y=bezier_points[i+1, 1])
                heading = get_heading_between_two_points(point, next_point)

            waypoint = Waypoint()
            waypoint.position.x = bezier_points[i, 0]
            waypoint.position.y = bezier_points[i, 1]
            waypoint.position.z = z_coords[i]
            waypoint.heading = heading
            waypoint.speed = speed[i]
            waypoint.blinker_state = blinker_state
            waypoint.left_width = lw[i]
            waypoint.right_width = rw[i]
            
            waypoints.append(waypoint)

        return waypoints

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('lane_change_planner')
    node = LaneChangePlanner()
    node.run()