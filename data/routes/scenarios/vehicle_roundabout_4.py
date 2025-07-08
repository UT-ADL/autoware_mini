#!/usr/bin/env python

import carla
import py_trees
import signal
import sys
import weakref
import time
import os
import matplotlib.colors as mcolors

from agents.navigation.basic_agent import BasicAgent
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import BasicScenario

# Helper function to convert color names to carla.Color objects
def name_to_carla_color(color_name):
    """Convert a color name to a carla.Color object."""
    try:
        rgb = mcolors.to_rgb(color_name)
        # matplotlib colors are in range [0,1], convert to [0,255]
        r, g, b = [int(c * 255) for c in rgb]
        return carla.Color(r, g, b)
    except ValueError:
        print(f"Warning: Color '{color_name}' not found, using white instead")
        return carla.Color(255, 255, 255)  # Default to white

class VehicleRoundabout4(BasicScenario):
    """Scenario with a vehicle approaching from the left following a specific route."""

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True, timeout=600):
        # Set basic properties
        self._world = world
        self._map = world.get_map()
        self._timeout = timeout
        self._debug_mode = debug_mode
        self._cleanup_performed = False
        self._keep_running = True
        
        # Agent and navigation properties
        self.agent = None
        self._agent_vehicle = None
        self._destination_coords = carla.Location(x=45, y=-243, z=37)
        self._spawn_coords = carla.Location(x=40.5, y=-242, z=36)
        self._initial_speed = 50  # km/h
        self._waypoint_list = []
        self._active_waypoints = []
        
        # Visualization colors - now using named colors
        self._route_color = name_to_carla_color('magenta')
        self._start_color = name_to_carla_color('magenta')
        self._destination_color = name_to_carla_color('magenta')
        self._wp_color = name_to_carla_color('red')
        self._wpreached_color = name_to_carla_color('grey')
        
        # Debug helper
        self._debug_helper = self._world.debug
        self._ego_vehicle_refs = []
        
        # Initialize world provider
        CarlaDataProvider.set_world(self._world)
        
        # Call parent constructor
        super(VehicleRoundabout4, self).__init__(
            "VehicleRoundabout4",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable
        )
        
        # Set up signal handling and store ego vehicle refs
        self._initialize_scenario(ego_vehicles)
        
    def _initialize_scenario(self, ego_vehicles):
        """Initialize scenario with signal handling and vehicle tracking"""
        # Set up signal handlers
        self.original_sigint = signal.getsignal(signal.SIGINT)
        signal.signal(signal.SIGINT, lambda sig, frame: self._terminate_scenario(f"Signal {sig} received"))
        signal.signal(signal.SIGTERM, lambda sig, frame: self._terminate_scenario(f"Signal {sig} received"))
        if sys.platform != 'win32':
            signal.signal(signal.SIGHUP, lambda sig, frame: self._terminate_scenario(f"Signal {sig} received"))
        
        # Store weak references to ego vehicles with callback when destroyed
        for ego_vehicle in ego_vehicles:
            if ego_vehicle:
                self._ego_vehicle_refs.append(
                    weakref.ref(ego_vehicle, lambda ref: self._terminate_scenario("Ego vehicle destroyed"))
                )
    
    def _terminate_scenario(self, reason=""):
        """Handle all scenario termination and cleanup"""
        if not self._keep_running or self._cleanup_performed:
            return
            
        # Log termination reason
        if reason:
            print(f"Terminating scenario: {reason}")
            
        # Mark scenario as not running
        self._keep_running = False
        
        # Clean up all actors
        self._cleanup_performed = True
        
        # Clean up agent vehicle
        if self._agent_vehicle and self._agent_vehicle.is_alive:
            self._agent_vehicle.destroy()
            self._agent_vehicle = None
        
        # Clean up other actors
        for actor in self.other_actors[:]:
            if actor and actor.is_alive:
                actor.destroy()
                self.other_actors.remove(actor)
        self.other_actors.clear()
        
        # Restore original signal handler if interrupted
        if reason.startswith("Signal") and self.original_sigint:
            signal.signal(signal.SIGINT, self.original_sigint)
    
    def _visualize_scenario(self, route=None):
        """Visualize the scenario if visualization is enabled"""
        # Check if visualization is enabled
        if not (os.environ.get('SCENARIO_DRAW_WAYPOINTS', '0') == '1' or self._debug_mode):
            return
        
        # Visualize route if provided
        if route:
            for i in range(len(route) - 1):
                current_wp = route[i][0] if isinstance(route[i], tuple) else route[i]
                next_wp = route[i+1][0] if isinstance(route[i+1], tuple) else route[i+1]
                
                current_loc = current_wp.transform.location if hasattr(current_wp, 'transform') else current_wp
                next_loc = next_wp.transform.location if hasattr(next_wp, 'transform') else next_wp
                
                self._debug_helper.draw_line(
                    current_loc + carla.Location(z=0.5), 
                    next_loc + carla.Location(z=0.5),
                    thickness=0.2, 
                    color=self._route_color, 
                    life_time=600.0
                )
        
        # Visualize start and destination
        if hasattr(self, '_actual_spawn_transform'):
            # Start point
            start_loc = self._actual_spawn_transform.location
            self._debug_helper.draw_point(start_loc + carla.Location(z=2.0), size=0.5, 
                color=self._start_color, life_time=600.0)
            self._debug_helper.draw_string(start_loc + carla.Location(z=3.0), "START", 
                draw_shadow=True, color=self._start_color, life_time=600.0)
        
        # Destination point
        dest_loc = self._destination_coords
        self._debug_helper.draw_point(dest_loc + carla.Location(z=2.0), size=0.5, 
            color=self._destination_color, life_time=600.0)
        self._debug_helper.draw_string(dest_loc + carla.Location(z=3.0), "DESTINATION", 
            draw_shadow=True, color=self._destination_color, life_time=600.0)
        
        # Visualize waypoints
        for i, wp in enumerate(self._active_waypoints):
            loc = wp['location']
            speed = wp['speed']
            color = self._wpreached_color if wp['reached'] else self._wp_color
            
            self._debug_helper.draw_point(loc + carla.Location(z=1.0), size=0.5, 
                color=color, life_time=600.0)
            self._debug_helper.draw_string(loc + carla.Location(z=2.0), f"WP{i+1}: {speed}km/h", 
                draw_shadow=True, color=color, life_time=600.0)
            
        # Vehicle status if available
        if self._agent_vehicle and self._agent_vehicle.is_alive:
            current_loc = self._agent_vehicle.get_location()
            speed = self._agent_vehicle.get_velocity().length() * 3.6  # km/h
            distance = current_loc.distance(self._destination_coords)
            print(f"Agent 3 status - Distance: {distance:.2f}m, Speed: {speed:.2f}km/h")

    def _initialize_actors(self, config):
        # Select vehicle blueprint
        blueprint = self._world.get_blueprint_library().find('vehicle.mini.cooper_s_2021')
        
        # Find closest spawn point
        spawn_points = self._map.get_spawn_points()
        closest_spawn = None
        closest_distance = float('inf')
        
        for spawn_point in spawn_points:
            distance = spawn_point.location.distance(self._spawn_coords)
            if distance < closest_distance:
                closest_distance = distance
                closest_spawn = spawn_point
        
        if not closest_spawn:
            print("No valid spawn points found")
            return None
            
        # Save spawn transform and spawn vehicle
        self._actual_spawn_transform = closest_spawn
        agent_vehicle = self._world.try_spawn_actor(blueprint, self._actual_spawn_transform)
        
        if not agent_vehicle:
            print("Failed to spawn vehicle")
            return None
            
        # Setup agent
        agent = BasicAgent(agent_vehicle, target_speed=self._initial_speed)
        agent.ignore_traffic_lights(False)
        agent.ignore_stop_signs(False)
        agent.ignore_vehicles(False)
        
        # Get destination waypoint
        destination_waypoint = self._map.get_waypoint(
            self._destination_coords,
            project_to_road=True,
            lane_type=carla.LaneType.Driving
        )
        
        # Small delay and get current waypoint
        time.sleep(0.1)
        current_waypoint = self._map.get_waypoint(
            agent_vehicle.get_location(),
            project_to_road=True,
            lane_type=carla.LaneType.Driving
        )
        
        # Create route
        self._create_route(agent, current_waypoint, destination_waypoint)
        
        # Store agent and vehicle
        self.agent = agent
        self._agent_vehicle = agent_vehicle
        self.other_actors.append(agent_vehicle)
        CarlaDataProvider.register_actor(agent_vehicle)
        
        # Initial visualization
        self._visualize_scenario()

        return agent_vehicle

    def _create_route(self, agent, current_waypoint, destination_waypoint):
        """Create a route for the agent with optional waypoints between start and destination."""
        # Get starting position
        vehicle_loc = self._actual_spawn_transform.location
        start_waypoint = self._map.get_waypoint(
            vehicle_loc,
            project_to_road=True,
            lane_type=carla.LaneType.Driving
        )
        
        # Case 1: Direct routing if no custom waypoints provided
        if not self._waypoint_list:
            self._set_direct_route(agent, start_waypoint, destination_waypoint)
            return
        
        # Case 2: Routing through custom waypoints
        waypoint_objects = self._prepare_custom_waypoints(start_waypoint, vehicle_loc)
        
        # If no valid waypoints found, fall back to direct routing
        if not waypoint_objects:
            self._set_direct_route(agent, start_waypoint, destination_waypoint)
            return
        
        # Add destination and create route through all waypoints
        waypoint_objects.append(destination_waypoint)
        complete_route = self._create_route_through_waypoints(agent, start_waypoint, waypoint_objects)
        
        # Set final route
        if complete_route and len(complete_route) > 2:
            agent.set_global_plan(complete_route)
            self._visualize_scenario(complete_route)
        else:
            agent.set_destination(destination_waypoint.transform.location)
        
        # Log scenario information
        self._log_scenario_info(complete_route)

    def _set_direct_route(self, agent, start_waypoint, destination_waypoint):
        """Create a direct route from start to destination without custom waypoints."""
        global_route = agent._global_planner.trace_route(
            start_waypoint.transform.location,
            destination_waypoint.transform.location
        )
        
        if global_route and len(global_route) > 2:
            agent.set_global_plan(global_route)
            self._visualize_scenario(global_route)
        else:
            agent.set_destination(destination_waypoint.transform.location)

    def _prepare_custom_waypoints(self, start_waypoint, vehicle_loc):
        """Convert custom waypoints to CARLA waypoints and filter them."""
        waypoint_objects = []
        self._active_waypoints = []
        forward_vector = start_waypoint.transform.get_forward_vector()
        
        for wp_coords in self._waypoint_list:
            # Convert coordinates to CARLA location
            approx_location = carla.Location(
                x=wp_coords[0],
                y=wp_coords[1],
                z=wp_coords[2] if len(wp_coords) > 2 else 0
            )
            
            # Get closest road waypoint
            road_waypoint = self._map.get_waypoint(
                approx_location,
                project_to_road=True,
                lane_type=carla.LaneType.Driving
            )
            
            # Check if waypoint is ahead of vehicle (using dot product)
            to_waypoint = road_waypoint.transform.location - vehicle_loc
            if forward_vector.x * to_waypoint.x + forward_vector.y * to_waypoint.y > -5.0:
                waypoint_objects.append(road_waypoint)
                
                # Store waypoint data for speed control
                speed = wp_coords[3] if len(wp_coords) > 3 else self._initial_speed
                self._active_waypoints.append({
                    'location': road_waypoint.transform.location,
                    'speed': speed,
                    'reached': False
                })
        
        return waypoint_objects

    def _create_route_through_waypoints(self, agent, start_waypoint, waypoint_objects):
        """Create a route passing through all waypoints in sequence."""
        complete_route = []
        current_point = start_waypoint
        
        for next_point in waypoint_objects:
            # Create route segment to next waypoint
            route_segment = agent._global_planner.trace_route(
                current_point.transform.location,
                next_point.transform.location
            )
            
            # Filter out waypoints too close to origin (likely invalid)
            filtered_segment = [
                (wp, option) for wp, option in route_segment 
                if wp.transform.location.distance(carla.Location(0, 0, 0)) >= 10
            ]
            
            # Remove duplicate point when connecting segments
            if complete_route and filtered_segment:
                filtered_segment = filtered_segment[1:]
            
            complete_route.extend(filtered_segment)
            current_point = next_point
        
        return complete_route

    def _log_scenario_info(self, complete_route):
        """Log scenario information and waypoints in XML format."""
        print(f"Scenario: VehicleRoundabout4")
        print(f"  Start: {self._spawn_coords}")
        print(f"  Destination: {self._destination_coords}")
        print(f"  Initial speed: {self._initial_speed}km/h")
        print(f"  Waypoints: {len(complete_route)}")

        #print(f"\nAgent 3 Waypoints in XML format:")
        #for wp_tuple in complete_route:
            #wp = wp_tuple[0]  # Extract waypoint from tuple
            #loc = wp.transform.location
            #print(f'<position x="{loc.x:.6f}" y="{loc.y:.6f}" z="{loc.z:.6f}"/>')
        #print("End of waypoints\n")

    def _create_behavior(self):
        """Create behavior tree for the scenario."""
        # Create root parallel behavior with timeout
        root = py_trees.composites.Parallel(
            name="VehicleRouteParallel",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE
        )
        root.add_child(TimeOut(self._timeout))
        
        # Create driving sequence
        sequence = py_trees.composites.Sequence("DestinationSequence")
        
        # Define driving behavior
        class DriveToDestination(py_trees.behaviour.Behaviour):
            def __init__(self, scenario):
                super().__init__("DriveToDestination")
                self.scenario = scenario
                self.running_time = 0
                self.agent_done = False
                self.last_ego_check = 0
                self.last_viz = 0
                self._success_returned = False
            
            def initialise(self):
                """Reset behavior state."""
                self.running_time = 0
                self.last_ego_check = 0
                self.last_viz = 0
                
                if self.scenario._keep_running:
                    self._success_returned = False
                    self.agent_done = False
                else:
                    self._success_returned = True
                    self.agent_done = True
            
            def update(self):
                """Process vehicle behavior."""
                # Early exit checks
                if not self.scenario._keep_running:
                    return py_trees.common.Status.SUCCESS
                
                if self.agent_done:
                    if not self._success_returned:
                        self.scenario._terminate_scenario("Destination reached")
                        self._success_returned = True
                        return py_trees.common.Status.SUCCESS
                    return py_trees.common.Status.FAILURE
                
                # Periodic ego vehicle check
                if self._should_check_ego() and not self._ego_vehicles_exist():
                    self.scenario._terminate_scenario("No ego vehicles remain")
                    return py_trees.common.Status.SUCCESS
                
                # Get agent and vehicle
                agent = self.scenario.agent
                vehicle = self.scenario._agent_vehicle
                
                # Validate vehicle
                if not vehicle or not vehicle.is_alive:
                    self.scenario._terminate_scenario("Agent vehicle no longer exists")
                    return py_trees.common.Status.SUCCESS
                
                # Check destination
                if agent.done():
                    self.agent_done = True
                    return py_trees.common.Status.RUNNING
                
                # Update speed at waypoints
                self._update_waypoint_speeds(vehicle, agent)
                
                # Control vehicle
                vehicle.apply_control(agent.run_step())
                
                # Periodic visualization
                self._update_visualization()
                
                # Update time and check timeout
                self.running_time += 1
                if self.running_time > self.scenario._timeout * 10:
                    self.scenario._terminate_scenario("Scenario timeout reached")
                    return py_trees.common.Status.SUCCESS
                
                return py_trees.common.Status.RUNNING
            
            def _should_check_ego(self):
                """Determine if ego vehicles should be checked."""
                if self.running_time - self.last_ego_check > 20:
                    self.last_ego_check = self.running_time
                    return True
                return False
            
            def _ego_vehicles_exist(self):
                """Check if any ego vehicles still exist."""
                return any(
                    ego_ref() and ego_ref().is_alive 
                    for ego_ref in self.scenario._ego_vehicle_refs
                )
            
            def _update_waypoint_speeds(self, vehicle, agent):
                """Update speed when approaching waypoints."""
                for wp in self.scenario._active_waypoints:
                    if not wp['reached'] and vehicle.get_location().distance(wp['location']) < 20.0:
                        new_speed = wp['speed']
                        current_speed = agent._target_speed * 3.6
                        
                        if abs(current_speed - new_speed) > 1.0:
                            agent.set_target_speed(new_speed)
                        
                        wp['reached'] = True
                        self.scenario._visualize_scenario()
                        break
            
            def _update_visualization(self):
                """Update visualization periodically."""
                if self.running_time - self.last_viz > 100:
                    self.last_viz = self.running_time
                    self.scenario._visualize_scenario()
        
        # Add behavior to tree
        sequence.add_child(DriveToDestination(self))
        root.add_child(sequence)
    
        return root

    def _create_test_criteria(self):
        return []
        
    def terminate(self):
        """Override terminate method from the parent class"""
        self._terminate_scenario("Scenario terminated by framework")
        super(VehicleRoundabout4, self).terminate()

def get_available_scenarios():
    return {"VehicleRoundabout4": VehicleRoundabout4}