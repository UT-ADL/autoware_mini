#!/usr/bin/env python3

import rospy
import csv
import math

from autoware_mini.msg import Path, Waypoint

class WaypointLoader:
    def __init__(self):

        # Parameters
        self.waypoints_file = rospy.get_param("~waypoints_file")
        self.output_frame = rospy.get_param("~output_frame")
        self.default_left_width = rospy.get_param("default_left_width")
        self.default_right_width = rospy.get_param("default_right_width")

        # Publishers
        self.waypoints_pub = rospy.Publisher('global_path', Path, queue_size=10, latch=True, tcp_nodelay=True)
        
    def load_waypoints(self, waypoints_file):
        
        wp_id = 0

        # load waypoints from file
        with open(waypoints_file, 'r') as f:
            reader = csv.reader(f)
            # skip header
            next(reader)
            waypoints = []

            for row in reader:
                # skip empty rows, if no data at all - no waypoints are returned and empty path is published
                if not row:
                    continue
                # create waypoint
                waypoint = Waypoint()
                # 0  1  2  3    4         5            6              7           8          9
                # x, y, z, yaw, velocity, change_flag, steering_flag, accel_flag, stop_flag, event_flag
                # set waypoint values
                waypoint.position.x = float(row[0])
                waypoint.position.y = float(row[1])
                waypoint.position.z = float(row[2])

                # convert the heading in waypoints file to radians
                waypoint.heading = math.radians(float(row[3]))
                # set waypoint velocity
                waypoint.speed = float(row[4])

                # set waypoint flags
                waypoint.blinker_state = int(row[6])

                # set waypoint width
                waypoint.left_width = self.default_left_width
                waypoint.right_width = self.default_right_width

                waypoints.append(waypoint)

                wp_id += 1

        return waypoints

    def publish_waypoints(self, waypoints):
        path = Path()
        
        path.header.frame_id = self.output_frame
        path.header.stamp = rospy.Time.now()
        path.waypoints = waypoints
        
        self.waypoints_pub.publish(path)


    def run(self):
        waypoints = self.load_waypoints(self.waypoints_file)
        self.publish_waypoints(waypoints)

        if len(waypoints) == 0:
            rospy.logerr("%s - no waypoints found in file: %s ", rospy.get_name(), self.waypoints_file)
        else:
            rospy.loginfo("%s - %i waypoints published from file: %s", rospy.get_name(), len(waypoints), self.waypoints_file)

        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('waypoint_loader', log_level=rospy.INFO)
    node = WaypointLoader()
    node.run()