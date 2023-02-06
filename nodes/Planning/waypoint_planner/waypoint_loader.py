#!/usr/bin/env python

import os
import rospy
import csv
import tf

from autoware_msgs.msg import LaneArray, Lane, Waypoint

class WaypointLoader:
    def __init__(self):

        # Parameters
        self.waypoints_file = rospy.get_param("~waypoints_file", None) # add check if none then error loading waypoint file

        # Publishers
        self.waypoints_pub = rospy.Publisher('/based/lane_waypoints_raw', LaneArray, queue_size=1, latch=True)

        # verify if file exists
        if not os.path.isfile(self.waypoints_file):
            rospy.logerr("WaypointLoader - file not found: %s ", self.waypoints_file)
            return
        else:
            rospy.loginfo("WaypointLoader - loading waypoints from file: %s ", self.waypoints_file)
            self.waypoints = self.load_waypoints(self.waypoints_file)
            self.publish_waypoints(self.waypoints)
            rospy.loginfo("WaypointLoader - waypoints are published ")


    def load_waypoints(self, waypoints_file):
        
        # load waypoints from file
        with open(waypoints_file, 'r') as f:
            reader = csv.reader(f)
            # skip header
            next(reader)
            waypoints = []

            for row in reader:
                # create waypoint
                waypoint = Waypoint()
                # 0      1  2  3  4    5         6            7              8           9          10   
                # wp_id, x, y, z, yaw, velocity, change_flag, steering_flag, accel_flag, stop_flag, event_flag
                # set waypoint values
                waypoint.gid = int(row[0])                          # not sure if should be used
                waypoint.pose.pose.position.x = float(row[1])
                waypoint.pose.pose.position.y = float(row[2])
                waypoint.pose.pose.position.z = float(row[3])
                waypoint.twist.twist.linear.x = float(row[5]) / 3.6 # convert to m/s

                # convert yaw to quaternion
                q = tf.transformations.quaternion_from_euler(0, 0, float(row[4]))
                waypoint.pose.pose.orientation.x = q[0]
                waypoint.pose.pose.orientation.y = q[1]
                waypoint.pose.pose.orientation.z = q[2]
                waypoint.pose.pose.orientation.w = q[3]

                # set waypoint flags
                waypoint.change_flag = int(row[6])

                # check if there are more columns - simple wayoint files have columns only up to change_flag
                if len(row)>7:
                    waypoint.wpstate.steering_flag = int(row[7])
                    waypoint.wpstate.accel_flag = int(row[8])
                    waypoint.wpstate.stop_flag = int(row[9])
                    waypoint.wpstate.event_flag = int(row[10])

                # add waypoint to array
                waypoints.append(waypoint)

        return waypoints

    def publish_waypoints(self, waypoints):
        # create lane
        lane = Lane()
        lane.header.frame_id = "map"
        lane.header.stamp = rospy.Time.now()
        lane.waypoints = waypoints
        # create lane array
        lane_array = LaneArray()
        lane_array.lanes.append(lane)
        # publish lane array
        self.waypoints_pub.publish(lane_array)


    def run(self):
        rospy.spin()
        

if __name__ == '__main__':
    rospy.init_node('waypoint_loader', anonymous=True, log_level=rospy.INFO)
    node = WaypointLoader()
    node.run()