#!/usr/bin/env python

import rospy
import csv
import tf
import math

from autoware_msgs.msg import Lane, Waypoint

class WaypointLoader:
    def __init__(self):

        # Parameters
        self.waypoints_file = rospy.get_param("~waypoints_file")
        self.output_frame = rospy.get_param("~output_frame", "map")
        self.wp_left_width = rospy.get_param("~wp_left_width", 1.4)
        self.wp_right_width = rospy.get_param("~wp_right_width", 1.4)

        # Publishers
        self.waypoints_pub = rospy.Publisher('global_path', Lane, queue_size=1, latch=True)

        self.waypoints = self.load_waypoints(self.waypoints_file)
        self.publish_waypoints()  

        if len(self.waypoints) == 0:
            rospy.logerr("waypoint_loader - no waypoints found in file: %s ", self.waypoints_file)
        else:
            rospy.loginfo("waypoint_loader - %i waypoints published from file: %s" % (len(self.waypoints), self.waypoints_file))
        
    def load_waypoints(self, waypoints_file):
        
        wp_id = 0

        # load waypoints from file
        with open(waypoints_file, 'r') as f:
            reader = csv.reader(f)
            # skip header
            next(reader)
            waypoints = []

            for row in reader:
                # skip empty rows, if no data at all - no waypoints are returned and empty lane is published
                if not row:
                    continue
                # create waypoint
                waypoint = Waypoint()
                # 0  1  2  3    4         5            6              7           8          9
                # x, y, z, yaw, velocity, change_flag, steering_flag, accel_flag, stop_flag, event_flag
                # set waypoint values
                waypoint.gid = wp_id
                waypoint.pose.pose.position.x = float(row[0])
                waypoint.pose.pose.position.y = float(row[1])
                waypoint.pose.pose.position.z = float(row[2])

                # convert yaw (contains heading in waypoints file) to quaternion
                x, y, z, w = tf.transformations.quaternion_from_euler(0, 0, math.radians(float(row[3])))
                waypoint.pose.pose.orientation.x = x
                waypoint.pose.pose.orientation.y = y
                waypoint.pose.pose.orientation.z = z
                waypoint.pose.pose.orientation.w = w

                waypoint.twist.twist.linear.x = float(row[4])

                # set waypoint flags
                waypoint.change_flag = int(row[5])
                waypoint.wpstate.steering_state = int(row[6])
                waypoint.wpstate.accel_state = int(row[7])
                waypoint.wpstate.stop_state = int(row[8])
                waypoint.wpstate.event_state = int(row[9])

                # set waypoint width
                waypoint.dtlane.lw = self.wp_left_width
                waypoint.dtlane.rw = self.wp_right_width

                waypoints.append(waypoint)

                wp_id += 1

        return waypoints

    def publish_waypoints(self):
        lane = Lane()
        
        lane.header.frame_id = self.output_frame
        lane.header.stamp = rospy.Time.now()
        lane.waypoints = self.waypoints
        
        self.waypoints_pub.publish(lane)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('waypoint_loader', log_level=rospy.INFO)
    node = WaypointLoader()
    node.run()