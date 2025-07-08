#!/usr/bin/env python3

import os
import csv
import rospy
import numpy as np
from tf2_ros import TransformListener, Buffer, TransformException
from autoware_mini.msg import VehicleCmd
from ros_numpy import numpify

import matplotlib.pyplot as plt

class ScenarioMetricsCalculator:
    def __init__(self):
        # Parameters
        self.scenario_name = rospy.get_param("~scenario_name")
        self.ade_csv_file = rospy.get_param("~ade_csv_file")
        self.ade_threshold = rospy.get_param("~ade_threshold")
        self.ade_plot_file = rospy.get_param("~ade_plot_file")
        self.speed_smoothness_threshold = rospy.get_param("~speed_smoothness_threshold")
        self.metrics_frequency = rospy.get_param("~metrics_frequency")
        self.transform_timeout = rospy.get_param("~transform_timeout")

        self.target_speed = None

        self.displacement_errors = []
        self.target_speeds = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        # Subscribers
        rospy.Subscriber('/control/vehicle_cmd', VehicleCmd, self.vehicle_cmd_callback, queue_size=1, buff_size=2**20, tcp_nodelay=True)

        # Register the shutdown hook
        rospy.on_shutdown(self.shutdown_hook)

    def vehicle_cmd_callback(self, msg):
        self.target_speed = msg.ctrl_cmd.linear_velocity #m/s

    def calculate_metrics(self):
        target_speed = self.target_speed
        # Fetch the transform
        try:
            transform = self.tf_buffer.lookup_transform("lexus_shadow", "base_link", rospy.Time.now(), rospy.Duration(self.transform_timeout))
        except (TransformException, rospy.ROSTimeMovedBackwardsException) as e:
            rospy.logwarn("%s - %s", rospy.get_name(), e)
            return
        
        translation_vector = numpify(transform.transform.translation)
        self.displacement_errors.append(np.linalg.norm(translation_vector[:2])) # ignore height dimension
        
        if target_speed is not None:
            self.target_speeds.append(target_speed)

    def shutdown_hook(self):
        ade_score = np.mean(self.displacement_errors)
        ade_score = round(ade_score, 2)

        # speed_smoothness = sqrt(1/N * sum(dv/dt)), where dv - difference between consecutive speed values, dt - sampling period, N - number of samples
        speed_smoothness = np.sqrt(max(np.sum(np.diff(self.target_speeds) * self.metrics_frequency) / len(self.target_speeds), 0))
        speed_smoothness = round(speed_smoothness, 2)

        if self.ade_csv_file.endswith(".csv"):

            new_csv_row = [self.scenario_name, ade_score, "SUCCESS" if ade_score <= self.ade_threshold else "FAILURE", 
                           speed_smoothness, "SUCCESS" if speed_smoothness <= self.speed_smoothness_threshold else "FAILURE"]
            
            csv_exists = os.path.isfile(self.ade_csv_file)
            csv_empty = os.stat(self.ade_csv_file).st_size == 0 if csv_exists else True

            # Write data to csv file
            with open(self.ade_csv_file, mode="a", newline="") as file:
                writer = csv.writer(file)

                # If the file is newly created or empty, write the header
                if csv_empty:
                    writer.writerow(["scenario_name", "ade_score", "ade_rating", "speed_smoothness", "speed_smoothness_rating"])

                writer.writerow(new_csv_row)
        else:
            print(f"ADE score: {ade_score}")
            print(f"Speed smoothness: {speed_smoothness}")

        # Plot the ADE metric over time
        if self.ade_plot_file.endswith(".png"):
            plt.figure(figsize=(10, 5))
            plt.plot(self.displacement_errors)

            plt.title(f"Displacement error in scenario: {self.scenario_name}")
            plt.xlabel("Timestep")
            plt.ylabel("Displacement error")
            plt.ylim(0, 100)

            plt.savefig(self.ade_plot_file)

    def run(self):
        rate = rospy.Rate(self.metrics_frequency)

        while not rospy.is_shutdown():
            self.calculate_metrics()
            try:
                rate.sleep()
            except (rospy.ROSTimeMovedBackwardsException, rospy.exceptions.ROSInterruptException):
                pass

if __name__ == '__main__':
    rospy.init_node('scenario_metrics_calculator', log_level=rospy.INFO)
    node = ScenarioMetricsCalculator()
    node.run()