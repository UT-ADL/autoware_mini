# Local Planning Module

## Overview
The Local Planning module in Autoware Mini is responsible for generating safe, comfortable, and dynamically feasible trajectories for the vehicle to follow in real-time. It translates high-level global paths into precise motion plans while accounting for the vehicle's dynamics, traffic conditions, and obstacles in the immediate environment.

![Local planner Pipeline](/images/nodes/local_planning.png)

## Architecture
The Local Planning module takes inputs from:

- **Global Planning module**: Receives reference path (`global_path`) to follow
- **Detection module**: Obtains final detected obstacles (`predicted objects`) for collision avoidance
- **Localization module**: Uses vehicle current pose and velocity data
- **Map data**: Uses static map data (Lanelet2 format) for traffic rules and road geometry

It produces outputs to:
- **Control module**: Provides optimized, collision-free trajectory (`local_path`) with velocity profiles for each waypoint

## Components

The module consists of several specialized components:

### Local Path Extractor 

The local path extractor extracts a portion of the global path around the vehicle's current position. It uses the vehicle's localization data to determine the relevant segment of the global path that needs to be followed. This segment is then used for further processing in the local planner.

### Rule-Based Planning
The local planner uses a rule-based approach that identifies potential collision points through specialized checker nodes. Each checker examines specific scenarios:

- **Goal Stop Checker**: Ensures the vehicle stops at the goal point of the global path.
- **Automatic Stop Checker**: Detects stop lines and manages stops until manual override.
- **Object Collision Checker**: Identifies potential collisions with obstacles on the path.
- **Pedestrian Crosswalk Checker**: Handles crosswalks and checks for crossing pedestrians.
- **Traffic Light Stopline Checker**: Monitors traffic light states and creates collision points at red lights.
- **Trajectory Collision Checker**: Checks for potential collisions with other vehicles' predicted trajectories.
- **Yielding Checker**: Handles yielding situations at yield signs or intersections.
- **Collision Points Merger**: Merges all collision points from various checkers into a single point cloud.
- **Speed Planner**: Generates the final trajectory by adjusting speeds based on all collision points.

### Collision Points Merger

Collision Points Merger merges all collision points from various checkers into a single point cloud. The merger ensures that the speed planner has a comprehensive view of all obstacles and traffic rules that need to be considered when generating the trajectory.

### Speed Planner

The speed planner generates the final trajectory by adjusting speeds based on all merged collision points. It creates safe deceleration profiles for the vehicle, ensuring that the trajectory is both comfortable and safe to follow. The speed planner takes into account the required stopping distance, maximum deceleration allowed, and other metadata associated with each collision point.

### Openpilot-Based Planning

As an alternative to the rule-based approach, Autoware Mini also supports a local planner powered by Openpilot's end-to-end trajectory prediction:

- Component receives position and velocity predictions from Openpilot end-to-end neural network
- Transforms predictions into the appropriate coordinate frame
- Constructs waypoints with proper position, heading, velocity, and blinker states
- Outputs a final collision-free trajectory that preserves Openpilot's motion planning characteristics

## Data Flow
1. Local planner receives the global path from Global Planning module and current vehicle state from Localization module
2. The local path extractor extracts a portion of the global path around the vehicle's current position
3. Various rule-based checkers identify potential collision points for specific scenarios (stop lines, traffic lights, other vehicles and their candidate trajectories, pedestrian on crosswalks, etc.)
4. Collision points from all checkers are merged into a single point cloud
5. The speed planner adjusts the trajectory waypoint velocities 
6. The final optimized trajectory is published to the Control module for execution

Alternatively, when using the Openpilot-based planner:
1. Local planner receives the global path from Global Planning module and current vehicle state from Localization module
2. Openpilot generates trajectory and velocity predictions from comma.ai end-to-end neural network
3. Local planner transforms these predictions and generates a local path with appropriate waypoint attributes
4. The final trajectory is published to the Control module for execution
