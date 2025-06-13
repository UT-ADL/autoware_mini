#!/usr/bin/env python3

import rospy
import numpy as np

from autoware_mini.msg import DetectedObjectArray, Path, Waypoint

from autoware_mini.geometry import get_speed_from_velocity, get_point_using_heading_and_distance

class NaivePredictor:
    def __init__(self):
        # Parameters
        self.prediction_horizon = rospy.get_param('~prediction_horizon')
        self.prediction_interval = rospy.get_param('~prediction_interval')
        self.prediction_min_speed = rospy.get_param('~prediction_min_speed')

        # Publishers
        self.predicted_objects_pub = rospy.Publisher('predicted_objects', DetectedObjectArray, queue_size=1, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber('tracked_objects', DetectedObjectArray, self.tracked_objects_callback, queue_size=1, buff_size=2**20, tcp_nodelay=True)

    def tracked_objects_callback(self, msg):
        num_objects = len(msg.objects)

        # Convert tracked objects to numpy array
        tracked_objects_array = np.zeros(num_objects, dtype=[
            ('position', np.float32, (2,)),
            ('velocity', np.float32, (2,)),
            ('acceleration', np.float32, (2,))
        ])

        valid_indices = []  # Keep track of indices for objects that need naive prediction

        for i, obj in enumerate(msg.objects):
            # Skip objects that do not need predictions
            # Don't create any trajectories if object doesn't have 2D speed
            if get_speed_from_velocity(obj.velocity) < self.prediction_min_speed or len(obj.candidate_trajectories.paths) > 0:
                continue

            # calculate prediction origin in front of the object
            car_front = get_point_using_heading_and_distance(obj.center, obj.heading, obj.dimensions.x / 2)
            tracked_objects_array[i]['position'] = (car_front.x, car_front.y)
            tracked_objects_array[i]['velocity'] = (obj.velocity.x, obj.velocity.y)
            tracked_objects_array[i]['acceleration'] = (obj.acceleration.x, obj.acceleration.y)

            valid_indices.append(i)

        # Predict future positions and velocities - includes also initial step, thus + 1
        num_timesteps = int(self.prediction_horizon // self.prediction_interval) + 1
        timesteps = np.arange(num_timesteps) * self.prediction_interval
        timesteps = timesteps[:, np.newaxis, np.newaxis]  # Reshape for broadcasting

        predicted_positions = (
            tracked_objects_array['position'][np.newaxis, :, :]  # Shape (1, num_objects, 2)
            + tracked_objects_array['velocity'][np.newaxis, :, :] * timesteps  # v0 * t
            + 0.5 * tracked_objects_array['acceleration'][np.newaxis, :, :] * timesteps**2  # (1/2) * a * t^2
        )
        predicted_velocities = (
            tracked_objects_array['velocity'][np.newaxis, :, :]  # Shape (1, num_objects, 2)
            + tracked_objects_array['acceleration'][np.newaxis, :, :] * timesteps  # v0 + a * t
        )

        # Create candidate trajectories
        for i in valid_indices:
            obj = msg.objects[i]
            obj_footprint_z = obj.center.z - obj.dimensions.z / 2
            path = Path()

            for t in range(num_timesteps):
                wp = Waypoint()
                wp.position.x, wp.position.y = predicted_positions[t, i]
                wp.position.z = obj_footprint_z
                wp.speed = np.linalg.norm(predicted_velocities[t, i])
                path.waypoints.append(wp)
            obj.candidate_trajectories.paths.append(path)

        # Publish predicted objects
        self.predicted_objects_pub.publish(msg)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('naive_predictor', log_level=rospy.INFO)
    node = NaivePredictor()
    node.run()