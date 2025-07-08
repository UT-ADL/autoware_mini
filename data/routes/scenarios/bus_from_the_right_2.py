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

class BusFromTheRight2(BasicScenario):
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
        self._destination_coords = carla.Location(x=-105, y=416, z=35)
        
        # Use the specific scene coordinates provided
        self._spawn_coords = carla.Location(x=59.5, y=700, z=36.0)
        self._spawn_yaw = 230.0  # Default spawn yaw in degrees - you can change this value
        
        self._initial_speed = 10  # km/h
        
        # Define waypoints with direct/road flag
        # Format: [x, y, z, speed_km_h, is_direct]
        # is_direct=1: Go directly to this waypoint
        # is_direct=0: Follow road to this waypoint
        self._waypoint_list = [
            [51, 691, 36.0, 10, 1],    # Direct waypoint
            [42, 686, 36, 25, 0],        # Road waypoint
            [13, 653, 36, 40, 0], # Road waypoint
            [-77.5, 508, 36, 20, 0], # Road waypoint
            [-89, 462, 35, 0, 1], # Road waypoint - Will stop here for 10 seconds
            [-95, 453, 35, 10, 1], # Road waypoint
            #[172, 798, 36, 50, 0], # Road waypoint
            #[114, 851, 36, 50, 0], # Road waypoint
        ]
        self._active_waypoints = []
        
        # Waiting properties
        self._wait_waypoint_index = len(self._waypoint_list) - 2  # Index of the waypoint to wait at (last waypoint)
        self._wait_duration = 30.0  # Wait duration in seconds
        self._is_waiting = False
        self._wait_start_time = 0.0
        
        # Visualization colors
        self._route_color = name_to_carla_color('red')
        self._direct_route_color = name_to_carla_color('red')
        self._start_color = name_to_carla_color('red')
        self._destination_color = name_to_carla_color('red')
        self._wpreached_color = name_to_carla_color('grey')
        self._direct_wp_color = name_to_carla_color('red')
        self._road_wp_color = name_to_carla_color('red')
        
        # Debug helper
        self._debug_helper = self._world.debug
        self._ego_vehicle_refs = []
        
        # Initialize world provider
        CarlaDataProvider.set_world(self._world)
        
        # Call parent constructor
        super(BusFromTheRight2, self).__init__(
            "BusFromTheRight2",
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
    
    def _visualize_scenario(self, routes=None):
        """Visualize the scenario if visualization is enabled"""
        # Check if visualization is enabled
        if not (os.environ.get('SCENARIO_DRAW_WAYPOINTS', '0') == '1' or self._debug_mode):
            return

        # Visualize route if provided
        if routes and isinstance(routes, list):
            for route_info in routes:
                route = route_info['route']
                is_direct = route_info['is_direct']
                color = self._direct_route_color if is_direct else self._route_color
                
                for i in range(len(route) - 1):
                    current_wp = route[i][0] if isinstance(route[i], tuple) else route[i]
                    next_wp = route[i+1][0] if isinstance(route[i+1], tuple) else route[i+1]
                    
                    current_loc = current_wp.transform.location if hasattr(current_wp, 'transform') else current_wp
                    next_loc = next_wp.transform.location if hasattr(next_wp, 'transform') else next_wp
                    
                    self._debug_helper.draw_line(
                        current_loc + carla.Location(z=0.5), 
                        next_loc + carla.Location(z=0.5),
                        thickness=0.2, 
                        color=color, 
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
            
            # Also visualize the forward vector to show yaw direction
            forward_vec = self._actual_spawn_transform.get_forward_vector()
            forward_point = start_loc + forward_vec * 5  # 5 meters forward
            
            self._debug_helper.draw_arrow(
                start_loc + carla.Location(z=1.0),
                forward_point + carla.Location(z=1.0),
                thickness=0.2,
                arrow_size=0.5,
                color=carla.Color(0, 255, 255),  # Cyan
                life_time=600.0
            )
        
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
            is_direct = wp['is_direct']
            color = self._wpreached_color if wp['reached'] else (self._direct_wp_color if is_direct else self._road_wp_color)
            
            self._debug_helper.draw_point(loc + carla.Location(z=1.0), size=0.5, 
                color=color, life_time=600.0)
            
            wp_type = "Direct" if is_direct else "Road"
            self._debug_helper.draw_string(loc + carla.Location(z=2.0), f"WP{i+1} ({wp_type}): {speed}km/h", 
                draw_shadow=True, color=color, life_time=600.0)
            
        # Vehicle status if available
        if self._agent_vehicle and self._agent_vehicle.is_alive:
            current_loc = self._agent_vehicle.get_location()
            speed = self._agent_vehicle.get_velocity().length() * 3.6  # km/h
            distance = current_loc.distance(self._destination_coords)
            print(f"Agent 4 status - Distance: {distance:.2f}m, Speed: {speed:.2f}km/h")

    def _initialize_actors(self, config):
        # Select vehicle blueprint
        blueprint = self._world.get_blueprint_library().find('vehicle.mitsubishi.fusorosa')
        
        # Set up a transform at our desired spawn coordinates with specified yaw
        spawn_transform = carla.Transform(
            location=self._spawn_coords,
            rotation=carla.Rotation(pitch=0, yaw=self._spawn_yaw, roll=0)  # Use specified yaw
        )
        
        # Try spawning at the exact location
        agent_vehicle = self._world.try_spawn_actor(blueprint, spawn_transform)
        
        if not agent_vehicle:
            print("Failed to spawn vehicle at exact coordinates")
            return None
            
        # Save spawn transform and spawn vehicle
        self._actual_spawn_transform = spawn_transform
        
        # Setup agent
        agent = BasicAgent(agent_vehicle, target_speed=self._initial_speed)
        agent.ignore_traffic_lights(True)
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
        """Create a route with mix of direct and road following based on waypoint flags."""
        # Get starting position
        vehicle_loc = self._actual_spawn_transform.location
        
        # Process waypoints
        processed_waypoints = []
        self._active_waypoints = []
        
        # Store waypoints for visualization and speed control
        for wp_coords in self._waypoint_list:
            # Convert coordinates to CARLA location
            wp_loc = carla.Location(
                x=wp_coords[0],
                y=wp_coords[1],
                z=wp_coords[2] if len(wp_coords) > 2 else 0
            )
            
            # Get speed and direct flag
            speed = wp_coords[3] if len(wp_coords) > 3 else self._initial_speed
            is_direct = bool(wp_coords[4]) if len(wp_coords) > 4 else False
            
            # Create a custom waypoint object
            class CustomWaypoint:
                def __init__(self, location):
                    self.transform = carla.Transform(location=location)
            
            custom_wp = CustomWaypoint(wp_loc)
            
            # Get the closest road waypoint for road following
            road_wp = self._map.get_waypoint(
                wp_loc,
                project_to_road=True,
                lane_type=carla.LaneType.Driving
            )
            
            # Store processed waypoint info
            processed_waypoints.append({
                'direct_wp': custom_wp,
                'road_wp': road_wp,
                'is_direct': is_direct,
                'location': wp_loc
            })
            
            # Store waypoint data for speed control and visualization
            self._active_waypoints.append({
                'location': wp_loc,
                'speed': speed,
                'is_direct': is_direct,
                'reached': False
            })
        
        # Now create the complete route
        complete_route = []
        all_routes = []
        
        # Create a starting waypoint using the actual spawn transform
        # This preserves the yaw information for the first segment
        start_wp = type('obj', (), {'transform': self._actual_spawn_transform})
        current_point = start_wp
        
        # Process each waypoint in sequence
        for i, wp_info in enumerate(processed_waypoints):
            is_direct = wp_info['is_direct']
            
            if is_direct:
                # Create direct path to waypoint
                direct_wp = wp_info['direct_wp']
                
                # Add direct segment
                direct_segment = [(current_point, carla.LaneChange.NONE), (direct_wp, carla.LaneChange.NONE)]
                
                # Add to visualization routes
                all_routes.append({
                    'route': direct_segment,
                    'is_direct': True
                })
                
                # Add to complete route
                if i == 0:
                    complete_route.extend(direct_segment)
                else:
                    # Skip the first point to avoid duplicates
                    complete_route.append((direct_wp, carla.LaneChange.NONE))
                
                # Update current point
                current_point = direct_wp
            else:
                # Create road-following path to waypoint
                road_wp = wp_info['road_wp']
                
                # Get current location
                current_loc = current_point.transform.location
                
                # Create road route from current point to road waypoint
                road_route = agent._global_planner.trace_route(
                    current_loc,
                    road_wp.transform.location
                )
                
                # Add to visualization routes
                all_routes.append({
                    'route': road_route,
                    'is_direct': False
                })
                
                # Add to complete route, skipping first point if not the first segment
                if i == 0:
                    complete_route.extend(road_route)
                else:
                    # Skip the first point to avoid duplicates
                    complete_route.extend(road_route[1:] if road_route else [])
                
                # Update current point
                current_point = road_wp
        
        # Add final segment to destination
        final_route = agent._global_planner.trace_route(
            current_point.transform.location,
            destination_waypoint.transform.location
        )
        
        # Add to visualization routes
        all_routes.append({
            'route': final_route,
            'is_direct': False
        })
        
        # Add to complete route, skipping first point
        complete_route.extend(final_route[1:] if final_route else [])
        
        # Set the route
        if complete_route and len(complete_route) > 1:
            agent.set_global_plan(complete_route)
            self._visualize_scenario(all_routes)
        else:
            print("Warning: Failed to create complete route, using direct destination")
            agent.set_destination(destination_waypoint.transform.location)
        
        # Log scenario information
        print(f"Scenario: BusFromTheRight2")
        print(f"  Start: {self._spawn_coords}")
        print(f"  Initial orientation: {self._spawn_yaw} degrees")
        print(f"  Waypoints: {len(self._waypoint_list)}")
        print(f"  Destination: {self._destination_coords}")
        print(f"  Initial speed: {self._initial_speed}km/h")
        print(f"  Total waypoints in route: {len(complete_route)}")

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
                
                # Handle waiting at specific waypoint
                if self.scenario._is_waiting:
                    # Check if wait duration has passed
                    current_time = time.time()
                    elapsed_wait_time = current_time - self.scenario._wait_start_time
                    
                    if elapsed_wait_time >= self.scenario._wait_duration:
                        # Resume movement
                        self.scenario._is_waiting = False
                        agent.set_target_speed(self.scenario._initial_speed)
                        print(f"Resumed movement after waiting for {elapsed_wait_time:.1f} seconds")
                        self.scenario._visualize_scenario()
                    else:
                        # Keep vehicle stopped while waiting
                        vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
                        
                        # Periodically update status
                        if int(elapsed_wait_time) != int(elapsed_wait_time - 0.1):
                            print(f"Waiting at waypoint: {elapsed_wait_time:.1f}/{self.scenario._wait_duration} seconds")
                        
                        return py_trees.common.Status.RUNNING
                
                # Check destination
                if agent.done():
                    self.agent_done = True
                    return py_trees.common.Status.RUNNING
                
                # Update speed at waypoints
                self._update_waypoint_speeds(vehicle, agent)
                
                # Control vehicle if not waiting
                if not self.scenario._is_waiting:
                    try:
                        control = agent.run_step()
                        vehicle.apply_control(control)
                    except Exception as e:
                        print(f"Error controlling vehicle: {e}")
                        return py_trees.common.Status.FAILURE
                
                # Periodic visualization
                self._update_visualization()
                
                # Update time and check timeout
                self.running_time += 1
                if self.running_time > self.scenario._timeout * 10:
                    self.scenario._terminate_scenario("Scenario timeout reached for Bus")
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
                for i, wp in enumerate(self.scenario._active_waypoints):
                    # Use 2.0 meters threshold for the stopping waypoint to ensure precision
                    distance_threshold = 4 if i == self.scenario._wait_waypoint_index else 20.0
                    distance_to_wp = vehicle.get_location().distance(wp['location'])
                    
                    if not wp['reached'] and distance_to_wp < distance_threshold:
                        new_speed = wp['speed']
                        current_speed = agent._target_speed * 3.6
                        
                        # Check if this is the waypoint where we should stop
                        if i == self.scenario._wait_waypoint_index:
                            # Start waiting period
                            self.scenario._is_waiting = True
                            self.scenario._wait_start_time = time.time()
                            print(f"Reached waypoint {i+1}, stopping for {self.scenario._wait_duration} seconds")
                            print(f"Distance to waypoint: {distance_to_wp:.2f} meters")
                            # Force stop
                            agent.set_target_speed(0)
                            vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
                        elif abs(current_speed - new_speed) > 1.0:
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
        super(BusFromTheRight2, self).terminate()

def get_available_scenarios():
    return {"BusFromTheRight2": BusFromTheRight2}