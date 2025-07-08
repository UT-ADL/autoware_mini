#!/usr/bin/env python3
#
# Copyright (c) 2023 Autonomous Driving Lab (ADL), University of Tartu.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import rospy
import os
import glob
from carla_ros_scenario_runner_types.msg import CarlaScenarioList, CarlaScenario

class CarlaScenarioPublisher:

    def __init__(self):

        # Node parameters
        self.scenario_path = rospy.get_param("~scenario_path")

        # Publishers
        self.available_scenarios_pub = rospy.Publisher('/carla/available_scenarios', CarlaScenarioList, queue_size=10, latch=True)

    def run(self):
        # Read all scenario files from the given path
        scenario_files = glob.glob(os.path.join(self.scenario_path, '*.xosc'))

        # Publish scenarios
        scenario_list = CarlaScenarioList()
        for scenario_file in scenario_files:
            scenario = CarlaScenario(
                name=os.path.basename(scenario_file).split('.')[0],
                scenario_file=scenario_file
            )
            scenario_list.scenarios.append(scenario)
        self.available_scenarios_pub.publish(scenario_list)

        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('carla_scenario_publisher')
    node = CarlaScenarioPublisher()
    node.run()
