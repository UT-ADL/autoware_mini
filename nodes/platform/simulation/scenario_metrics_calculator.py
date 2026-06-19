#!/usr/bin/env python3

import os
import csv
import rospy
import numpy as np
import shapely
import scipy.ndimage
from tf2_ros import TransformListener, Buffer, TransformException
from geometry_msgs.msg import TwistStamped
from autoware_mini.msg import DetectedObjectArray
from ros_numpy import numpify

from autoware_mini.transform import get_distance_to_car_front

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

class ScenarioMetricsCalculator:
    def __init__(self):
        # Parameters
        self.metrics_frequency = rospy.get_param("~metrics_frequency")
        self.safety_box_width = rospy.get_param("/planning/safety_box_width")
        self.safety_box_length = rospy.get_param("/planning/safety_box_length")
        self.transform_timeout = rospy.get_param("~transform_timeout")

        self.current_velocity_msg = None
        self.detected_objects_msg = None
        self.last_detected_objects_stamp = None
        self.last_reference_car_location = None

        self.displacement_errors = []
        self.ego_speeds = []
        self.speed_timestamps = []
        self.reference_speeds = []
        self.collisions = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        # Subscribers (register before TF lookup so messages are received while constructor blocks)
        rospy.Subscriber('/localization/current_velocity', TwistStamped, self.current_velocity_callback, queue_size=1, buff_size=2**20, tcp_nodelay=True)
        rospy.Subscriber('/perception/detected_objects', DetectedObjectArray, self.detected_objects_callback, queue_size=1, tcp_nodelay=True)

        front_distance = get_distance_to_car_front()

        ego_box = [[front_distance, -self.safety_box_width / 2.0, 0.0, 1.0],
                   [front_distance, self.safety_box_width / 2.0, 0.0, 1.0],
                   [front_distance - self.safety_box_length, self.safety_box_width / 2.0, 0.0, 1.0],
                   [front_distance - self.safety_box_length, -self.safety_box_width / 2.0, 0.0, 1.0]]
        self.ego_box = np.array(ego_box)

        # Register the shutdown hook
        rospy.on_shutdown(self.shutdown_hook)

        # Calculate metrics at a fixed rate
        rospy.Timer(rospy.Duration(1 / self.metrics_frequency), self.calculate_metrics)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def current_velocity_callback(self, msg):
        self.current_velocity_msg = msg

    def detected_objects_callback(self, msg):
        self.detected_objects_msg = msg

    def calculate_metrics(self, event):
        current_velocity_msg = self.current_velocity_msg
        detected_objects_msg = self.detected_objects_msg

        if current_velocity_msg is None or detected_objects_msg is None:
            return

        # Calculate the displacement error
        try:
            transform_bl_ls = self.tf_buffer.lookup_transform("lexus_shadow", "base_link", event.current_real, rospy.Duration(self.transform_timeout))
            transform_ls_map = self.tf_buffer.lookup_transform("map", "lexus_shadow", event.current_real, rospy.Duration(self.transform_timeout))
        except (TransformException, rospy.ROSTimeMovedBackwardsException) as e:
            rospy.logwarn("%s - %s", rospy.get_name(), e)
            return

        translation_vector = numpify(transform_bl_ls.transform.translation)
        displacement_error = np.linalg.norm(translation_vector[:2]) # ignore height dimension
        if translation_vector[0] < 0: # make displacement error negative when ego is behind the ground-truth
            displacement_error = -displacement_error

        # Calculate the reference car velocity
        reference_speed = 0.0
        if self.last_reference_car_location is None or self.last_detected_objects_stamp is None:
            self.last_reference_car_location = numpify(transform_ls_map.transform.translation)
        else:
            current_reference_car_location = numpify(transform_ls_map.transform.translation)
            displacement = np.linalg.norm(current_reference_car_location[:2] - self.last_reference_car_location[:2])
            reference_speed = displacement / (event.current_real - event.last_real).to_sec() # m/s
            self.last_reference_car_location = current_reference_car_location

        # Check for collisions
        collision = False
        try:
            transform_bl_map = self.tf_buffer.lookup_transform("map", "base_link", detected_objects_msg.header.stamp, rospy.Duration(self.transform_timeout))
        except (TransformException, rospy.ROSTimeMovedBackwardsException) as e:
            rospy.logwarn("%s - %s", rospy.get_name(), e)
        else:
            # Transform the ego box to the actual location
            transform_bl_map_matrix = numpify(transform_bl_map.transform)
            ego_box_map = self.ego_box @ transform_bl_map_matrix.T
            ego_box_map = ego_box_map[:, :2]
            ego_box_poly = shapely.polygons(ego_box_map)

            # Turn detected objects into polygons
            detected_objects_poly = [shapely.polygons(np.array(obj.convex_hull).reshape(-1, 3)[:, :2]) for obj in detected_objects_msg.objects]

            # Check for collisions
            collision = any(ego_box_poly.intersects(detected_objects_poly))

        self.displacement_errors.append(displacement_error)
        self.ego_speeds.append(current_velocity_msg.twist.linear.x)
        self.speed_timestamps.append(current_velocity_msg.header.stamp.to_sec())
        self.reference_speeds.append(reference_speed)
        self.collisions.append(collision)

        self.last_detected_objects_stamp = detected_objects_msg.header.stamp

    def shutdown_hook(self):
        # Read save-related parameters fresh from parameter server
        scenario_name = rospy.get_param("~scenario_name")
        ade_csv_file = rospy.get_param("~ade_csv_file")
        ade_threshold = rospy.get_param("~ade_threshold")
        fde_threshold = rospy.get_param("~fde_threshold")
        ade_plot_file = rospy.get_param("~ade_plot_file")
        max_deceleration_threshold = rospy.get_param("~max_deceleration_threshold")
        collision_threshold = rospy.get_param("~collision_threshold")

        ade_score = np.mean(np.abs(self.displacement_errors))
        ade_score = round(ade_score, 2)

        fde_score = abs(self.displacement_errors[-1])
        fde_score = round(fde_score, 2)

        collision_score = np.mean(self.collisions) # count of collisions / total number of frames
        # use max 0.01 not to loose very tiny collision scores being rounded to 0.0 and classified as SUCCESS
        collision_score = round(max(0.01, collision_score), 2) if collision_score > 0 else 0.0

        # ignore acceleration/deceleration in the first and last 3 frames
        speed_diffs = np.diff(self.ego_speeds)[3:-3]
        time_diffs = np.diff(self.speed_timestamps)[3:-3]
        valid = time_diffs > 0
        max_deceleration = np.max(-speed_diffs[valid] / time_diffs[valid], initial=0)
        max_deceleration = round(max_deceleration, 2)

        if ade_csv_file.endswith(".csv"):

            new_csv_row = [scenario_name, ade_score, "SUCCESS" if ade_score <= ade_threshold else "FAILURE",
                           fde_score, "SUCCESS" if fde_score <= fde_threshold else "FAILURE",
                           collision_score, "SUCCESS" if collision_score <= collision_threshold else "FAILURE",
                           max_deceleration, "SUCCESS" if max_deceleration <= max_deceleration_threshold else "FAILURE"]

            csv_exists = os.path.isfile(ade_csv_file)
            csv_empty = os.stat(ade_csv_file).st_size == 0 if csv_exists else True

            # Write data to csv file
            with open(ade_csv_file, mode="a", newline="") as file:
                writer = csv.writer(file)

                # If the file is newly created or empty, write the header
                if csv_empty:
                    writer.writerow(["scenario_name", "ade_score", "ade_rating", "fde_score", "fde_rating", "collision_score", "collision_rating", "max_deceleration", "max_deceleration_rating"])

                writer.writerow(new_csv_row)
        else:
            print(f"ADE score: {ade_score}")
            print(f"FDE score: {fde_score}")
            print(f"Collision score: {collision_score}")
            print(f"Max deceleration: {max_deceleration}")

        # Plot the ADE metric over time
        if ade_plot_file.endswith(".png"):
            fig, ax1 = plt.subplots(figsize=(12, 6))
            ax2 = ax1.twinx()

            # Smooth the reference speeds
            reference_speeds_smooth = scipy.ndimage.gaussian_filter1d(self.reference_speeds, sigma=3)

            line1, = ax1.plot(self.displacement_errors, label='Displacement error', linewidth=3)
            line2, = ax2.plot(np.array(self.ego_speeds)*3.6, label='Ego speed', color='green', linewidth=1)
            line3, = ax2.plot(reference_speeds_smooth*3.6, label='Reference speed', color='black', linewidth=1)

            # Color the background based on collision state
            assert len(self.displacement_errors) == len(self.collisions), "Displacement errors and collision states must have the same length"

            in_region = False
            start = None

            for i in range(len(self.collisions)):
                if self.collisions[i] and not in_region:
                    start = i
                    in_region = True
                elif not self.collisions[i] and in_region:
                    ax1.axvspan(start, i, color='red', alpha=0.3)
                    in_region = False

            # Handle case where flag continues to the end
            if in_region:
                ax1.axvspan(start, len(self.collisions)-1, color='red', alpha=0.3)

            # Create a dummy patch for legend
            danger_patch = mpatches.Patch(color='red', alpha=0.3, label='Collision')
            ax1.legend(handles=[line1, line2, line3, danger_patch])

            ax1.set_xlabel("Timestep")
            ax1.set_ylabel("Displacement error (m)")
            ax2.set_ylabel("Speed (km/h)")
            ax1.set_ylim(-75, 75)
            ax2.set_ylim(-75, 75)

            plt.title(f"Displacement error in scenario: {scenario_name}")
            plt.axhline(0, color='gray', linewidth=1, linestyle='--')

            plt.savefig(ade_plot_file)
            plt.close()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('scenario_metrics_calculator', log_level=rospy.INFO)
    node = ScenarioMetricsCalculator()
    node.run()
