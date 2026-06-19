#!/usr/bin/env python3
import yaml
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from autoware_mini.msg import Path

from carla_ros_scenario_runner_types.msg import CarlaScenarioList, CarlaScenario, CarlaScenarioRunnerStatus
from carla_ros_scenario_runner_types.srv import ExecuteScenario

class GoalPublisher:

    def __init__(self):

        # Node parameters
        self.goals_file_name = rospy.get_param("~goals_file")

        # Internal variables
        self.goals = {}
        self.current_scenario_status = CarlaScenarioRunnerStatus.STOPPED
        self.previous_goal_failed = False

        # Publishers
        self.available_scenarios_pub = rospy.Publisher('/carla/available_scenarios', CarlaScenarioList, queue_size=10, latch=True)
        self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10, tcp_nodelay=True)
        self.scenario_status_publisher = rospy.Publisher('/scenario_runner/status', CarlaScenarioRunnerStatus, queue_size=10, tcp_nodelay=True, latch=True)
        
        # Subscribers
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback, queue_size=None, tcp_nodelay=True)
        rospy.Subscriber('global_path', Path, self.global_path_callback, queue_size=None, tcp_nodelay=True)

        # Services
        rospy.Service('/scenario_runner/execute_scenario', ExecuteScenario, self.publish_goal_handler)

    def publish_goal_handler(self, msg):
        if msg.scenario.name not in self.goals:
            rospy.logerr("Scenario not found in the goals file")
            return False

        # Reset the scenario runner status
        self.current_scenario_status = CarlaScenarioRunnerStatus.STOPPED
        self.scenario_status_publisher.publish(self.current_scenario_status)
        self.previous_goal_failed = False

        # Create a seperate thread for publishing goals, otherwise a delay causes the RViz to freeze
        t = threading.Thread(target = self.publish_goals, args=(self.goals[msg.scenario.name],))
        t.daemon = True
        t.start()

        return True
    
    def publish_goals(self, positions):
        for position in positions:
            goal_pose = PoseStamped()
            goal_pose.header.stamp = rospy.Time.now()
            goal_pose.header.frame_id = "map"

            goal_pose.pose.position.x = position["x"]
            goal_pose.pose.position.y = position["y"]
            goal_pose.pose.position.z = position["z"]

            goal_pose.pose.orientation.x = 0
            goal_pose.pose.orientation.y = 0
            goal_pose.pose.orientation.z = 0
            goal_pose.pose.orientation.w = 1

            self.goal_publisher.publish(goal_pose)

            if "delay" in position:
                rospy.sleep(position["delay"])
    
    def goal_callback(self, msg): 
        if self.current_scenario_status == CarlaScenarioRunnerStatus.STARTING:
            self.previous_goal_failed = True

        # If a goal is published, then set scenario runner status to STARTING (yellow)
        self.current_scenario_status = CarlaScenarioRunnerStatus.STARTING
        self.scenario_status_publisher.publish(self.current_scenario_status)
            
    def global_path_callback(self, msg):
        if msg.waypoints:
            # Set scenario runner status to RUNNING (green) only when a global path was found for the prevous goal point
            if not self.previous_goal_failed:
                self.current_scenario_status = CarlaScenarioRunnerStatus.RUNNING
                self.scenario_status_publisher.publish(self.current_scenario_status)
        else:
            # If the global path vanishes, then set scenario runner status to STOPPED (grey)
            self.current_scenario_status = CarlaScenarioRunnerStatus.STOPPED
            self.scenario_status_publisher.publish(self.current_scenario_status)
            self.previous_goal_failed = False

    def run(self):
        goals_list = CarlaScenarioList()

        with open(self.goals_file_name, 'r') as file:
            goals = yaml.safe_load_all(file)
        
            for goal in goals:
                self.goals[goal["name"]] = goal["positions"]

                scenario = CarlaScenario(
                    name=goal["name"],
                    scenario_file=self.goals_file_name
                )
                goals_list.scenarios.append(scenario)

        self.available_scenarios_pub.publish(goals_list)

        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('goal_planner')
    node = GoalPublisher()
    node.run()