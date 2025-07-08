# Control Module Overview

The control module in Autoware Mini is responsible for executing trajectory tracking by converting planned path into vehicle control commands. 

![Control System Architecture](/images/nodes/control.png)

## Architecture

The control module takes inputs from:
- **Planning module**: Provides local path with waypoints and planned target velocities
- **Localization module**: Provides current vehicle position, orientation, and velocity

It produces outputs:
- **Vehicle commands**: Steering angle, velocity, acceleration, and blinker states forwarded to the vehicle control interface

## Available Controllers

### Classic Controllers

Two main geometric control algorithm implementations are available:

#### Pure Pursuit Controller

The Pure Pursuit algorithm calculates steering commands by:
1. Finding a target point on the path ahead of the vehicle (lookahead point)
2. Computing the curvature required to reach that point
3. Converting the curvature to a steering angle vehicle should follow

**Characteristics:**
- Simple and robust implementation
- Performance depends on appropriate lookahead distance tuning
- May cut corners at high speeds if not properly tuned

#### Stanley Controller

The Stanley algorithm, originally developed for Stanford's DARPA Grand Challenge entry, combines two correction terms to compute the vehicle steering angle:

1. **Heading Error Correction**: Directly corrects the difference between the vehicle's heading and the path's heading at the front wheel's projection point on the path.

2. **Cross-Track Error Correction**: Applies steering proportional to the lateral deviation from the path.

**Characteristics:**
- Better precision in path tracking
- More responsive to path deviations by first correcting heading error and then cross-track error 
- May have oscillations at low speeds if not properly tuned

## Safety Features

Both controllers implement:
- **Error checking**: Stops vehicle if lateral error or heading angle difference exceeds limits
- **Emergency braking**: Applies maximum deceleration in critical situations
- **Smooth deceleration**: Calculates optimal deceleration when approaching obstacles
