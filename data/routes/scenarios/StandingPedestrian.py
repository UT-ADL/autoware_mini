#!/usr/bin/env python
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Pedestrian crossing scenario with a left turn and no ego vehicle dependency.
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


class StandingPedestrian(BasicScenario):
    """
    This class implements a pedestrian crossing scenario where the pedestrian
    walks straight, makes a left turn, and continues walking.
    The scenario starts automatically with no dependency on ego vehicle.
    """

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True, timeout=60):
        """
        Setup all relevant parameters and create scenario
        """
        # Store the timeout
        self.timeout = timeout
        
        self._wmap = CarlaDataProvider.get_map()
        
        # Set the pedestrian spawn location and heading
        self._pedestrian_spawn_point = carla.Location(x=-6, y=59, z=35.0)
        self._pedestrian_heading = -90  # in degrees, as requested
        
        # Scenario parameters
        self._adversary_speed = 1.5     # Speed of the pedestrian [m/s]
        self._first_segment_distance = 8.0  # First walking segment [m]
        self._second_segment_distance = 15.0  # Second walking segment after turn [m]
        self._idle_time = 0.0           # No idle time - start immediately
        self._turn_time = 4.0           # Time to complete the turn [s]
        
        # Calculate turn position based on initial heading and first segment distance
        angle_rad = math.radians(self._pedestrian_heading)
        self._turn_position = self._pedestrian_spawn_point + carla.Location(
            x=self._first_segment_distance * math.cos(angle_rad),
            y=self._first_segment_distance * math.sin(angle_rad)
        )
        
        # Calculate the heading after turn (subtract 90 degrees for left turn)
        self._second_heading = self._pedestrian_heading + 10
        if self._second_heading < -180:
            self._second_heading += 360
            
        # Initialize randomness if needed
        if randomize:
            self._adversary_speed = random.uniform(1.0, 2.0)

        # Walker data - now includes two segments and a turn
        self._walker_data = {
            'initial_transform': None,
            'speed': self._adversary_speed,
            'idle_time': self._idle_time,
            'first_distance': self._first_segment_distance,
            'first_duration': self._first_segment_distance / self._adversary_speed,
            'turn_position': self._turn_position,
            'turn_heading': self._second_heading,
            'turn_time': self._turn_time,
            'second_distance': self._second_segment_distance,
            'second_duration': self._second_segment_distance / self._adversary_speed
        }
        
        # Important: For scenarios with no ego vehicle dependency, we need to pass an empty list
        # If ego_vehicles list contains None or invalid vehicles, we should clean it
        clean_ego_list = [vehicle for vehicle in ego_vehicles if vehicle is not None and vehicle.is_alive]
        
        # Initialize the parent class
        super(StandingPedestrian, self).__init__(
            "StandingPedestrian",
            clean_ego_list,  # Use the cleaned list
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable
        )
        
        # Force the scenario to be independent of any ego vehicle
        self.ego_vehicles = []
        
        if debug_mode:
            print("StandingPedestrian scenario initialized with NO ego vehicle dependency")
            print(f"Pedestrian spawn location: {self._pedestrian_spawn_point}")
            print(f"Initial heading: {self._pedestrian_heading}")
            print(f"Turn position: {self._turn_position}")
            print(f"Heading after turn: {self._second_heading}")
            print(f"Pedestrian speed: {self._adversary_speed} m/s")
            print(f"First segment distance: {self._first_segment_distance} m")
            print(f"Second segment distance: {self._second_segment_distance} m")
            print(f"Timeout: {self.timeout} s")

    def _initialize_actors(self, config):
        """
        Initialize the pedestrian actor
        """
        # Create the pedestrian at the specified location
        spawn_rotation = carla.Rotation(yaw=self._pedestrian_heading)
        spawn_transform = carla.Transform(self._pedestrian_spawn_point, spawn_rotation)
        
        walker = CarlaDataProvider.request_new_actor('walker.pedestrian.0002', spawn_transform)
        if walker is None:
            raise ValueError("Failed to spawn the pedestrian")
            
        self.other_actors.append(walker)
        
        # Store the transform for later use
        self._walker_data['initial_transform'] = spawn_transform
        
        if self.debug_mode:
            print(f"Pedestrian actor initialized: {walker.id}")
            debug = self.world.debug
            
            # Draw debug markers
            debug.draw_point(self._pedestrian_spawn_point, size=0.2, color=carla.Color(255, 0, 0), life_time=self.timeout)
            debug.draw_point(self._turn_position, size=0.2, color=carla.Color(0, 255, 0), life_time=self.timeout)
            
            # Draw first segment path
            debug.draw_arrow(
                self._pedestrian_spawn_point,
                self._turn_position,
                thickness=0.2,
                arrow_size=0.2,
                color=carla.Color(0, 0, 255),
                life_time=self.timeout
            )
            
            # Draw second segment path
            angle_rad_2 = math.radians(self._second_heading)
            end_location = self._turn_position + carla.Location(
                x=self._second_segment_distance * math.cos(angle_rad_2),
                y=self._second_segment_distance * math.sin(angle_rad_2)
            )
            debug.draw_arrow(
                self._turn_position,
                end_location,
                thickness=0.2,
                arrow_size=0.2,
                color=carla.Color(255, 0, 255),
                life_time=self.timeout
            )

    def _create_behavior(self):
        """
        Create the behavior tree for the pedestrian crossing with a left turn.
        This behavior tree is completely independent of any ego vehicle.
        """
        # Create a sequence for the pedestrian behavior
        sequence = py_trees.composites.Sequence("StandingPedestrianSequence")
        
        # Set up the walker
        walker_actor = self.other_actors[0]
        walker_data = self._walker_data
        
        # Setup initial position
        sequence.add_child(ActorTransformSetter(walker_actor, walker_data['initial_transform'], name="SetInitialPosition"))
        
        # First walking segment - walk straight to the turning point
        first_walk = KeepVelocity(
            walker_actor, 
            walker_data['speed'], 
            False,  # Not relative
            duration=walker_data['first_duration'],
            distance=walker_data['first_distance'],
            name="WalkToTurnPoint"
        )
        sequence.add_child(first_walk)
        
        # Create the turn transform
        turn_rotation = carla.Rotation(yaw=walker_data['turn_heading'])
        turn_transform = carla.Transform(walker_data['turn_position'], turn_rotation)
        
        # Set the new direction at the turn point
        sequence.add_child(ActorTransformSetter(walker_actor, turn_transform, name="TurnLeft"))
        
        # Small pause for the turn to look natural
        sequence.add_child(Idle(duration=walker_data['turn_time'], name="CompleteTurn"))
        
        # Second walking segment - walk in the new direction
        second_walk = KeepVelocity(
            walker_actor, 
            walker_data['speed'], 
            False,  # Not relative
            duration=walker_data['second_duration'],
            distance=walker_data['second_distance'],
            name="WalkAfterTurn"
        )
        sequence.add_child(second_walk)
        
        # Add optional destroy behavior after walking is complete
        sequence.add_child(ActorDestroy(walker_actor, name="DestroyPedestrian"))
        
        # Add a timeout to end the scenario after the specified time
        timeout = TimeOut(self.timeout, name="ScenarioTimeout")
        sequence.add_child(timeout)
        
        if self.debug_mode:
            print("Autonomous pedestrian behavior tree with left turn created")
            
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
    return {"StandingPedestrian": StandingPedestrian}