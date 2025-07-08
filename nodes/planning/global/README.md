# Global Planning Module

## Overview
The Global Planning module in Autoware Mini is responsible for generating high-level paths from the vehicle's current position to a specified goal point. It serves as the foundation for the vehicle's navigation system by providing feasible routes that consider the road network, traffic rules, and mission objectives.

![Global planner Pipeline](/images/nodes/global_planning.png)

## Architecture
The Global Planning module takes inputs from:

- **Localization module**: Provides vehicle position (`/localization/current_pose`) and velocity (`/localization/current_velocity`) data
- **Map data**: Uses static map data (Lanelet2 format) for route planning
- **Goal Publisher Interface**: Provides goal points (`/move_base_simple/goal`) for navigation, either from manual input or automated scenarios

It produces outputs to:
- **Local Planning module**: Supplies the final smoothed global path (`global_path`) for local trajectory generation

## Components
The module consists of several specialized components:

### Lanelet2 Planning
- **Lanelet2 Global Planner**: Creates optimal global routes using the Lanelet2 map framework. It finds the shortest feasible path between current position and goal point while respecting road rules and lane connectivity.
- **Lane Change Planner**: Generates smooth lane change trajectories when the global path requires changing lanes, using Bezier curves to create natural transitions.
- **Lanelet2 Map Visualizer**: Provides visualization of map elements including lanelets, traffic lights, stop lines, and regulatory elements.

### Waypoint Processing Utilities
- **Waypoint Loader**: Loads pre-defined waypoints from CSV files to create a global path, useful for fixed routes or pre-recorded paths.
- **Waypoint Saver**: Records vehicle trajectory as waypoints, allowing for manual route creation by driving the desired path.

### Common Utilities
- **Goal Publisher**: Manages goal points and publishes them to the planning system. It supports both manual goal selection and automated scenario execution.
- **Path Smoothing**: Applies various smoothing techniques to the global path to ensure driving comfort and feasibility:
  - Interpolates waypoints at fixed intervals
  - Adjusts speeds for curves based on lateral acceleration limits
  - Ensures speed profiles respect acceleration and deceleration limits
  - Manages endpoint velocities

## Data Flow
1. Global planner module receives current vehicle position from localizer module and goal destination from goal publisher
2. Lanelet2-based planning generates a coarse global path
3. (Optional) The path undergoes lane change planning and smoothing
4. A detailed global path with speed profiles and lane information is published to local planning module for further processing