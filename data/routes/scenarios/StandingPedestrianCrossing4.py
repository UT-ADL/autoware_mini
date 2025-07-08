#!/usr/bin/env python
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Simple pedestrian crossing scenario with no ego vehicle dependency.
"""

from __future__ import print_function

import py_trees
import carla
import math
import random

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorDestroy,
                                                                      KeepVelocity,
                                                                      Idle,
                                                                      ActorTransformSetter)
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import BasicScenario


class StandingPedestrianCrossing4(BasicScenario):
    """
    This class implements a pedestrian crossing scenario that starts automatically,
    with no dependency on ego vehicle.
    """

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True, timeout=60):
        """
        Setup all relevant parameters and create scenario
        """
        # Store the timeout
        self.timeout = timeout
        
        self._wmap = CarlaDataProvider.get_map()
        
        # Set the pedestrian spawn location and heading
        self._pedestrian_spawn_point = carla.Location(x=6, y=-90, z=36.0)
        self._pedestrian_heading = -6  # in degrees, as requested
        
        # Scenario parameters
        self._adversary_speed = 1.5     # Speed of the pedestrian [m/s]
        self._crossing_distance = 23.0  # Distance to cross [m]
        self._idle_time = 2.5          # Idle time before starting to walk [s]
        
        # Initialize randomness if needed
        if randomize:
            self._adversary_speed = random.uniform(1.0, 2.0)
            self._idle_time = random.uniform(1.0, 5.0)

        # Walker data
        self._walker_data = [
            {'transform': None, 'speed': self._adversary_speed, 'idle_time': self._idle_time, 
             'distance': self._crossing_distance, 'duration': self._crossing_distance / self._adversary_speed}
        ]
        
        # Important: For scenarios with no ego vehicle dependency, we need to pass an empty list
        # If ego_vehicles list contains None or invalid vehicles, we should clean it
        clean_ego_list = [vehicle for vehicle in ego_vehicles if vehicle is not None and vehicle.is_alive]
        
        # Initialize the parent class
        super(StandingPedestrianCrossing4, self).__init__(
            "StandingPedestrianCrossing4",
            clean_ego_list,  # Use the cleaned list
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable
        )
        
        # Force the scenario to be independent of any ego vehicle
        self.ego_vehicles = []
        
        if debug_mode:
            print("StandingPedestrianCrossing4 scenario initialized with NO ego vehicle dependency")
            print(f"Pedestrian spawn location: {self._pedestrian_spawn_point}")
            print(f"Pedestrian heading: {self._pedestrian_heading}")
            print(f"Pedestrian speed: {self._adversary_speed} m/s")
            print(f"Pedestrian idle time: {self._idle_time} s")
            print(f"Crossing distance: {self._crossing_distance} m")
            print(f"Timeout: {self.timeout} s")

    def _initialize_actors(self, config):
        """
        Initialize the pedestrian actor
        """
        # Create the pedestrian at the specified location
        spawn_rotation = carla.Rotation(yaw=self._pedestrian_heading)
        spawn_transform = carla.Transform(self._pedestrian_spawn_point, spawn_rotation)
        
        walker = CarlaDataProvider.request_new_actor('walker.pedestrian.0015', spawn_transform)
        if walker is None:
            raise ValueError("Failed to spawn the pedestrian")
            
        self.other_actors.append(walker)
        
        # Store the transform for later use
        self._walker_data[0]['transform'] = spawn_transform
        
        if self.debug_mode:
            print(f"Pedestrian actor initialized: {walker.id}")
            # Draw debug markers
            debug = self.world.debug
            debug.draw_point(self._pedestrian_spawn_point, size=0.2, color=carla.Color(255, 0, 0), life_time=self.timeout)
            
            # Draw walking path
            angle_rad = math.radians(self._pedestrian_heading)
            end_location = self._pedestrian_spawn_point + carla.Location(
                x=self._crossing_distance * math.cos(angle_rad),
                y=self._crossing_distance * math.sin(angle_rad)
            )
            debug.draw_arrow(
                self._pedestrian_spawn_point,
                end_location,
                thickness=0.2,
                arrow_size=0.2,
                color=carla.Color(0, 0, 255),
                life_time=self.timeout
            )

    def _create_behavior(self):
        """
        Create the behavior tree for the pedestrian crossing scenario.
        This behavior tree is completely independent of any ego vehicle.
        """
        # Create a sequence for the pedestrian behavior
        sequence = py_trees.composites.Sequence("PedestrianSequence")
        
        # Set up the walker
        walker_actor = self.other_actors[0]
        walker_data = self._walker_data[0]
        
        # Setup initial position
        sequence.add_child(ActorTransformSetter(walker_actor, walker_data['transform'], name="SetInitialPosition"))
        
        # Add an idle behavior to wait before walking
        if walker_data['idle_time'] > 0:
            idle_behavior = Idle(
                duration=walker_data['idle_time'],
                name=f"IdleBeforeWalking_{walker_data['idle_time']}s"
            )
            sequence.add_child(idle_behavior)
            
            if self.debug_mode:
                print(f"Pedestrian will wait for {walker_data['idle_time']} seconds before walking")
        
        # Walk across
        walk_behavior = KeepVelocity(
            walker_actor, 
            walker_data['speed'], 
            False,  # Not relative
            duration=walker_data['duration'],
            distance=walker_data['distance'],
            name="WalkAcrossRoad"
        )
        sequence.add_child(walk_behavior)
        
        # Add optional destroy behavior after walking is complete
        sequence.add_child(ActorDestroy(walker_actor, name="DestroyPedestrian"))
        
        # Add a timeout to end the scenario after the specified time
        timeout = TimeOut(self.timeout, name="ScenarioTimeout")
        sequence.add_child(timeout)
        
        if self.debug_mode:
            print("Autonomous pedestrian behavior tree created")
            
        return sequence

    def _create_test_criteria(self):
        """
        No test criteria needed for this scenario since it's purely autonomous.
        """
        return []

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        if hasattr(self, 'other_actors') and self.other_actors:
            if self.debug_mode:
                print("Cleaning up pedestrian actors...")
            self.remove_all_actors()


def get_available_scenarios():
    """
    Return the available scenarios for this module
    """
    return {"StandingPedestrianCrossing4": StandingPedestrianCrossing4}