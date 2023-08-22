#!/usr/bin/env python3

import rospy
import numpy as np

from autoware_msgs.msg import DetectedObjectArray, Lane, Waypoint
from geometry_msgs.msg import PoseStamped, TwistStamped

class NaivePredictor:
    def __init__(self):
        # Parameters
        self.prediction_horizon = rospy.get_param('~prediction_horizon')
        self.prediction_interval = rospy.get_param('~prediction_interval')

        # Publishers
        self.predicted_objects_pub = rospy.Publisher('predicted_objects', DetectedObjectArray, queue_size=1, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber('tracked_objects', DetectedObjectArray, self.tracked_objects_callback, queue_size=1, buff_size=2**20, tcp_nodelay=True)

    def tracked_objects_callback(self, msg):
        # Convert tracked objects to numpy array
        tracked_objects_array = np.empty((len(msg.objects)), dtype=[
            ('centroid', np.float32, (2,)),
            ('velocity', np.float32, (2,)),
            ('acceleration', np.float32, (2,)),
        ])
        for i, obj in enumerate(msg.objects):
            tracked_objects_array[i]['centroid'] = (obj.pose.position.x, obj.pose.position.y)
            tracked_objects_array[i]['velocity'] = (obj.velocity.linear.x, obj.velocity.linear.y) 
            tracked_objects_array[i]['acceleration'] = (obj.acceleration.linear.x, obj.acceleration.linear.y)

        # Predict future positions and velocities
        num_timesteps = int(self.prediction_horizon // self.prediction_interval)
        predicted_objects_array = np.empty((num_timesteps, len(msg.objects)), dtype=[
            ('centroid', np.float32, (2,)),
            ('velocity', np.float32, (2,)),
        ])
        predicted_objects_array[0] = tracked_objects_array[['centroid', 'velocity']]
        for i in range(1, num_timesteps):
            predicted_objects_array[i]['centroid'] = predicted_objects_array[i-1]['centroid'] + predicted_objects_array[i-1]['velocity'] * self.prediction_interval
            predicted_objects_array[i]['velocity'] = predicted_objects_array[i-1]['velocity'] + tracked_objects_array['acceleration'] * self.prediction_interval

        # Create candidate trajectories
        for i, obj in enumerate(msg.objects):
            lane = Lane()
            for j in range(num_timesteps):
                wp = Waypoint()
                wp.pose.pose.position.x, wp.pose.pose.position.y = predicted_objects_array[j][i]['centroid']
                wp.twist.twist.linear.x, wp.twist.twist.linear.y = predicted_objects_array[j][i]['velocity']
                lane.waypoints.append(wp)
            obj.candidate_trajectories.lanes.append(lane)

        # Publish predicted objects
        self.predicted_objects_pub.publish(msg)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('naive_predictor', log_level=rospy.INFO)
    node = NaivePredictor()
    node.run()