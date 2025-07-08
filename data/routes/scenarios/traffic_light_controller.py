#!/usr/bin/env python

"""
Traffic Light Controller for Carla Simulator with ScenarioRunner
Forces traffic lights to turn red when ego vehicle approaches within detection range
Keeps them red for 10 seconds before allowing normal operation to resume
"""

import py_trees
import carla
import time
import math
import sys
import os

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenarios.basic_scenario import BasicScenario


class TrafficLightForcingController(py_trees.behaviour.Behaviour):
    """
    Behavior that forces traffic lights to red when ego vehicle approaches
    and keeps them red for a specified duration
    """
    def __init__(self, ego_actor, name="TrafficLightForcingController", detection_distance=50.0, red_duration=7.0,
                 trigger_point=None, trigger_radius=3.0):
        super(TrafficLightForcingController, self).__init__(name)
        self._ego_actor = ego_actor
        self._world = None
        self._debug = None
        self._tick_counter = 0
        self._started = False
        
        # Configuration parameters
        self._detection_distance = detection_distance  # meters - detection range for traffic lights
        self._red_duration = red_duration  # seconds - how long to keep the light red
        self._green_duration = 30.0  # seconds - how long to keep the light green
        
        # Traffic light management
        self._affected_traffic_lights = {}  # Dictionary of {traffic_light_id: time_to_reset}
        self._permanently_processed_ids = set()  # Lights we'll never touch again
        
        # Status tracking - to avoid reporting when no change
        self._last_active_count = 0
        self._last_processed_count = 0
        
        # Check for visualization flag - prevents all logs if disabled
        self._verbose = os.environ.get('SCENARIO_DRAW_WAYPOINTS', '0') == '1'
        
        # Trigger point logic
        self._trigger_point = trigger_point  # carla.Location or None
        self._trigger_radius = trigger_radius
        self._trigger_reached = False

        # Mode tracking
        self._mode = "RED"  # Start in RED mode
        self._green_mode_point = carla.Location(x=145.748108, y=443.184631, z=34.574802)
        self._green_mode_radius = 3.0
        self._green_mode_activated = False
        self._first_mode_switch = False
        self._red_mode_switch_point = carla.Location(x=-151, y=245, z=35)  # Old reactivation point
        self._red_mode_switch_radius = 30.0
        
    def initialise(self):
        """Initialize the controller"""
        if self._verbose:
            print(f"[{self.name}] Starting Traffic Light Controller in {self._mode} mode")
        
        self._started = False
        self._tick_counter = 0
        
        # Get world and debug
        self._world = CarlaDataProvider.get_world()
        try:
            self._debug = self._world.debug if self._verbose else None
        except:
            self._debug = None
            if self._verbose:
                print(f"[{self.name}] WARNING: Debug visualization unavailable")
        
        # Clear state
        self._affected_traffic_lights = {}
    
    def _draw_detection_circle(self, location, radius, color=carla.Color(255, 0, 0, 50)):
        """
        Draw a detection radius circle using line segments since draw_circle isn't available
        """
        try:
            if not self._debug:
                return
                
            # Draw a circle with line segments
            segments = 16  # Number of segments to use
            prev_point = None
            
            for i in range(segments + 1):
                angle = 2.0 * math.pi * float(i) / float(segments)
                x = location.x + radius * math.cos(angle)
                y = location.y + radius * math.sin(angle)
                point = carla.Location(x, y, location.z + 0.5)
                
                if prev_point:
                    self._debug.draw_line(
                        prev_point,
                        point,
                        thickness=0.1,
                        color=color,
                        life_time=0.5
                    )
                prev_point = point
                
        except Exception:
            # Silently ignore any errors in visualization
            pass
        
    def _find_nearby_traffic_lights(self):
        """Find traffic lights in detection range"""
        result = []
        
        # Exit if ego vehicle is invalid
        if not self._ego_actor or not self._ego_actor.is_alive:
            return result
        
        try:
            # Get ego vehicle location
            ego_location = self._ego_actor.get_location()
            
            # Get all traffic lights in the world
            all_traffic_lights = self._world.get_actors().filter('traffic.traffic_light*')
            if not all_traffic_lights:
                if self._verbose:
                    print(f"[{self.name}] WARNING: No traffic lights found in the world")
                return result
                
            # Check if ego vehicle is directly affected by a traffic light
            direct_light = self._ego_actor.get_traffic_light()
            if direct_light and direct_light.id not in self._permanently_processed_ids:
                distance = ego_location.distance(direct_light.get_location())
                if distance <= self._detection_distance:
                    result.append((direct_light, distance))
            
            # Process all traffic lights within detection range
            for light in all_traffic_lights:
                # Skip lights we've already permanently processed
                if light.id in self._permanently_processed_ids:
                    continue
                    
                # Check distance
                light_location = light.get_location()
                distance = ego_location.distance(light_location)
                
                if distance <= self._detection_distance:
                    # It's close enough, add to result
                    result.append((light, distance))
                
        except Exception as e:
            if self._verbose:
                print(f"[{self.name}] ERROR finding traffic lights: {e}")
            
        return result
    
    def _force_traffic_light_state(self, traffic_light, distance, target_state, duration):
        """Force a traffic light to specified state"""
        # Skip if already processed permanently
        if traffic_light.id in self._permanently_processed_ids:
            return False
            
        # Skip if already being controlled
        if traffic_light.id in self._affected_traffic_lights:
            return False
        
        try:
            # Get current state
            original_state = traffic_light.get_state()
            
            # Force to target state
            traffic_light.set_state(target_state)
            
            # Try to freeze if that API is available
            try:
                traffic_light.freeze(True)
            except:
                pass
                
            # Store in affected lights dictionary
            reset_time = time.time() + duration
            self._affected_traffic_lights[traffic_light.id] = {
                'light': traffic_light,
                'reset_time': reset_time,
                'original_state': original_state
            }
            
            # Visualize if enabled
            if self._debug:
                light_location = traffic_light.get_location()
                color = carla.Color(255, 0, 0) if target_state == carla.TrafficLightState.Red else carla.Color(0, 255, 0)
                state_name = "RED" if target_state == carla.TrafficLightState.Red else "GREEN"
                
                # Draw the light
                self._debug.draw_point(
                    light_location,
                    size=0.3,
                    color=color,
                    life_time=duration
                )
                
                # Draw text
                self._debug.draw_string(
                    light_location + carla.Location(z=1.0),
                    f"FORCED {state_name}: {duration}s",
                    color=carla.Color(255, 255, 255),
                    life_time=duration
                )
                
                # Draw line from ego to light
                if self._ego_actor and self._ego_actor.is_alive:
                    ego_location = self._ego_actor.get_location()
                    self._debug.draw_line(
                        ego_location,
                        light_location,
                        thickness=0.1,
                        color=color,
                        life_time=1.0
                    )
            
            # Log action only if verbose is enabled
            if self._verbose:
                state_name = "RED" if target_state == carla.TrafficLightState.Red else "GREEN"
                print(f"[{self.name}] FORCED traffic light ID={traffic_light.id} to {state_name} for {duration}s")
            
            return True
            
        except Exception as e:
            if self._verbose:
                print(f"[{self.name}] ERROR forcing traffic light {traffic_light.id}: {e}")
            return False
    
    def _release_traffic_lights(self):
        """Release traffic lights that have completed their duration"""
        current_time = time.time()
        lights_to_release = []
        
        for light_id, light_data in self._affected_traffic_lights.items():
            # Check if it's time to release
            if current_time >= light_data['reset_time']:
                lights_to_release.append(light_id)
                traffic_light = light_data['light']
                
                try:
                    # Unfreeze first
                    try:
                        traffic_light.freeze(False)
                    except:
                        pass
                        
                    # Restore original state
                    traffic_light.set_state(light_data['original_state'])
                    
                    # Add to permanently processed set
                    self._permanently_processed_ids.add(light_id)
                    
                    # Visualize if enabled
                    if self._debug:
                        light_location = traffic_light.get_location()
                        self._debug.draw_point(
                            light_location,
                            size=0.3,
                            color=carla.Color(0, 255, 0),
                            life_time=2.0
                        )
                        self._debug.draw_string(
                            light_location + carla.Location(z=1.0),
                            "RELEASED - NORMAL OPERATION",
                            color=carla.Color(255, 255, 255),
                            life_time=2.0
                        )
                    
                    # Log release only if verbose is enabled
                    if self._verbose:
                        duration = self._red_duration if self._mode == "RED" else self._green_duration
                        print(f"[{self.name}] RELEASED traffic light ID={light_id} after {duration}s")
                    
                except Exception as e:
                    if self._verbose:
                        print(f"[{self.name}] ERROR releasing traffic light {light_id}: {e}")
        
        # Remove released lights from affected dictionary
        for light_id in lights_to_release:
            if light_id in self._affected_traffic_lights:
                del self._affected_traffic_lights[light_id]

    def _release_all_traffic_lights(self):
        """Release all affected traffic lights and restore their original state"""
        for light_id, light_data in list(self._affected_traffic_lights.items()):
            traffic_light = light_data['light']
            try:
                try:
                    traffic_light.freeze(False)
                except:
                    pass
                traffic_light.set_state(light_data['original_state'])
                if self._debug:
                    light_location = traffic_light.get_location()
                    self._debug.draw_point(
                        light_location,
                        size=0.3,
                        color=carla.Color(0, 255, 0),
                        life_time=2.0
                    )
                    self._debug.draw_string(
                        light_location + carla.Location(z=1.0),
                        "RELEASED - NORMAL OPERATION",
                        color=carla.Color(255, 255, 255),
                        life_time=2.0
                    )
                if self._verbose:
                    print(f"[{self.name}] RELEASED traffic light ID={light_id} (final cleanup)")
            except Exception as e:
                if self._verbose:
                    print(f"[{self.name}] ERROR releasing traffic light {light_id} (final cleanup): {e}")
        self._affected_traffic_lights.clear()

    def update(self):
        """Main update method called by behavior tree"""
        self._tick_counter += 1

        # Check trigger points and handle mode transitions
        if self._ego_actor and self._ego_actor.is_alive:
            ego_location = self._ego_actor.get_location()
            
            # Check for green mode activation point
            if not self._green_mode_activated:
                dist = ego_location.distance(self._green_mode_point)
                if dist <= self._green_mode_radius:
                    if self._verbose:
                        print(f"[{self.name}] Green mode point reached at {ego_location} (distance {dist:.2f} m)")
                        print(f"[{self.name}] Switching to GREEN mode")
                    
                    self._release_all_traffic_lights()
                    self._mode = "GREEN"
                    self._green_mode_activated = True
                    # Reset processed lights to allow controlling them again
                    self._permanently_processed_ids.clear()
            
            # Check for red mode switch point (old reactivation point)
            if self._green_mode_activated and not self._first_mode_switch:
                dist = ego_location.distance(self._red_mode_switch_point)
                if dist <= self._red_mode_switch_radius:
                    if self._verbose:
                        print(f"[{self.name}] Red mode switch point reached at {ego_location} (distance {dist:.2f} m)")
                        print(f"[{self.name}] Switching back to RED mode")
                    
                    self._release_all_traffic_lights()
                    self._mode = "RED"
                    self._first_mode_switch = True
                    self._trigger_reached = True
                    # Reset processed lights to allow controlling them again
                    self._permanently_processed_ids.clear()

        # First tick initialization
        if not self._started:
            if self._verbose:
                print(f"[{self.name}] Traffic Light Controller is now ACTIVE in {self._mode} mode")
            self._started = True
        
        # Handle traffic light updates
        try:
            # 1. Release any traffic lights that have completed their duration
            self._release_traffic_lights()
            
            # 2. Find nearby traffic lights
            nearby_lights = self._find_nearby_traffic_lights()
            
            # 3. Force newly detected lights to appropriate state
            affected_count = 0
            for light, distance in nearby_lights:
                if self._mode == "RED":
                    if self._force_traffic_light_state(light, distance, carla.TrafficLightState.Red, self._red_duration):
                        affected_count += 1
                elif self._mode == "GREEN":
                    if self._force_traffic_light_state(light, distance, carla.TrafficLightState.Green, self._green_duration):
                        affected_count += 1
                    
            # 4. Report status ONLY if numbers have changed
            current_active = len(self._affected_traffic_lights)
            current_processed = len(self._permanently_processed_ids)
            
            # Only log if there's a change from last time
            if self._verbose and (current_active != self._last_active_count or 
                                current_processed != self._last_processed_count):
                print(f"[{self.name}] Mode: {self._mode}, Active: {current_active}, Processed: {current_processed} traffic lights")
                
                # Update the tracking values
                self._last_active_count = current_active
                self._last_processed_count = current_processed
            
            # 5. Report newly affected lights (if any)
            if self._verbose and affected_count > 0:
                print(f"[{self.name}] Forced {affected_count} new traffic lights to {self._mode}")
            
            # 6. Draw detection radius if enabled
            if self._debug and self._tick_counter % 10 == 0:
                if self._ego_actor and self._ego_actor.is_alive:
                    ego_location = self._ego_actor.get_location()
                    # Use our custom circle drawing method with color based on mode
                    color = carla.Color(255, 0, 0, 50) if self._mode == "RED" else carla.Color(0, 255, 0, 50)
                    self._draw_detection_circle(ego_location, self._detection_distance, color)
                
        except Exception as e:
            if self._verbose:
                print(f"[{self.name}] ERROR during update: {e}")
        
        # Always return RUNNING to keep the behavior alive throughout the route scenario
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """
        Terminate functionality - release any controlled traffic lights
        """
        if self._verbose:
            print(f"[{self.name}] Terminating with status {new_status}")
        self._release_all_traffic_lights()
        super(TrafficLightForcingController, self).terminate(new_status)


class TrafficLightForcingScenario(BasicScenario):
    """
    Scenario that forces traffic lights to red when ego vehicle approaches them
    """

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True, timeout=300):
        """
        Initialize parameters for traffic light forcing scenario
        """
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self._timeout = timeout
        self.config = config
        self._debug = debug_mode
        
        # Configuration
        self._detection_distance = 50.0  # meters - detection range
        self._red_duration = 7.0  # seconds - how long to keep lights red
        
        # Trigger point config
        self._trigger_point = carla.Location(77.602119, 724.942017, 35.844406)
        self._trigger_radius = 3.0
        
        # Get parameters from config if available
        if hasattr(config, 'parameters'):
            params = config.parameters
            if 'detection_distance' in params:
                self._detection_distance = float(params['detection_distance'])
            if 'red_duration' in params:
                self._red_duration = float(params['red_duration'])
            # Parse trigger point if provided
            if 'trigger_point' in params:
                # Expecting a string like "x,y,z"
                try:
                    x, y, z = map(float, params['trigger_point'].split(','))
                    self._trigger_point = carla.Location(x, y, z)
                except Exception:
                    print("WARNING: Invalid trigger_point format, expected 'x,y,z'")
            if 'trigger_radius' in params:
                self._trigger_radius = float(params['trigger_radius'])
        
        # Check if visualization is enabled via environment variable
        self._verbose = os.environ.get('SCENARIO_DRAW_WAYPOINTS', '0') == '1'
        
        # Initialize CarlaDataProvider
        CarlaDataProvider.set_world(self._world)
        
        # Call parent class constructor
        super(TrafficLightForcingScenario, self).__init__(
            "TrafficLightForcingScenario",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable
        )
        
        # Print configuration if visualization is enabled
        if self._verbose:
            print("\n=== TRAFFIC LIGHT FORCING SCENARIO ===")
            print(f"Detection distance: {self._detection_distance} meters")
            print(f"Red light duration: {self._red_duration} seconds")
            print(f"Green light duration: 20.0 seconds")
            print(f"Timeout: {self._timeout} seconds")
            print(f"Visualization: {'ENABLED' if self._verbose else 'DISABLED'}")
            
            if self.ego_vehicles:
                for i, ego in enumerate(self.ego_vehicles):
                    print(f"Ego vehicle {i}: ID={ego.id}, Type={ego.type_id}")
            else:
                print("WARNING: No ego vehicles found at initialization!")

    def _initialize_actors(self, config):
        """No additional actors needed"""
        pass

    def _create_behavior(self):
        """Create behavior tree for the scenario"""
        # Create a sequence behavior
        root = py_trees.composites.Sequence("TrafficLightForcing")
        
        # Check for ego vehicle
        if not self.ego_vehicles:
            if self._verbose:
                print("ERROR: No ego vehicle - creating empty behavior tree")
            return root
        
        # Create the traffic light controller behavior
        traffic_light_controller = TrafficLightForcingController(
            self.ego_vehicles[0],
            detection_distance=self._detection_distance,
            red_duration=self._red_duration,
            trigger_point=self._trigger_point,
            trigger_radius=self._trigger_radius
        )
        
        # Add the controller to the behavior tree
        root.add_child(traffic_light_controller)
        return root

    def _create_test_criteria(self):
        """No specific criteria for this scenario"""
        return []

    def __del__(self):
        """Clean up scenario"""
        if self._verbose:
            print("\n=== CLEANING UP TRAFFIC LIGHT FORCING SCENARIO ===")


# Register the scenario
def get_available_scenarios():
    """Return the available scenarios for this module"""
    return {
        "TrafficLightForcingScenario": TrafficLightForcingScenario
    }