# Vehicle

Nodes related to running the autonomy stack in the real vehicle.

## ssc_interface

A Python ROS node for interfacing with Autoware's SSC. 

#### Parameters
| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `use_adaptive_gear_ratio` | bool | `True` | Whether to use an adaptive gear ratio. |
| `enable_reverse_motion` | bool | `False` | Whether to enable reverse motion. |
| `command_timeout` | int | `200` | Timeout (in ms) to send dummy command message to keep SSC alive. |
| `wheel_base` | float | `2.789` | Distance between front and rear axle (in meters). |
| `ssc_gear_ratio` | float | `16.135` | Default gear ratio used in SSC. |
| `acceleration_limit` | float | `3.0` | Maximum allowed acceleration (in m/s^2). |
| `deceleration_limit` | float | `3.0` | Maximum allowed deceleration (in m/s^2). |
| `max_curvature_rate` | float | `0.15` | Maximum allowed curvature rate (in rad/s), this affects the steering angle aggressiveness. |
| `agr_coef_a` | float | `15.713` | Adaptive gear ratio coefficient a (base steering ratio). |
| `agr_coef_b` | float | `0.053` | Adaptive gear ratio coefficient b (velocity component). |
| `agr_coef_c` | float | `0.042` | Adaptive gear ratio coefficient c (steering angle component). |

#### Subscribed Topics
| Name | Type | Description |
| --- | --- | --- |
| `engage` | `std_msgs/Bool` | Command to engage/disengage. |
| `vehicle_cmd` | `autoware_msgs/VehicleCmd` | Command for vehicle motion. |
| `/ssc/module_states` | `automotive_navigation_msgs/ModuleState` | Module states feedback. Used to check the active state of SSC. |
| `/ssc/curvature_feedback` | `automotive_platform_msgs/CurvatureFeedback` | Curvature feedback. |
| `/ssc/throttle_feedback` | `automotive_platform_msgs/ThrottleFeedback` | Throttle feedback. |
| `/ssc/brake_feedback` | `automotive_platform_msgs/BrakeFeedback` | Brake feedback. |
| `/ssc/gear_feedback` | `automotive_platform_msgs/GearFeedback` | Gear feedback. |
| `/ssc/steering_feedback` | `automotive_platform_msgs/SteeringFeedback` | Steering feedback. |
| `/ssc/velocity_accel_cov` | `automotive_platform_msgs/VelocityAccelCov` | Velocity, acceleration, covariance feedback. |
| `/pacmod/parsed_tx/turn_rpt` | `pacmod_msgs/SystemRptInt` | Turn signal feedback from Pacmod (SSC does not provide turn signal info). |

#### Published Topics
| Name | Type | Description |
| --- | --- | --- |
| `/ssc/arbitrated_speed_commands` | `automotive_platform_msgs/SpeedMode` | Speed command (including acceleration/deceleration limits) to SSC. |
| `/ssc/arbitrated_steering_commands` | `automotive_platform_msgs/SteerMode` | Steering command to SSC. |
| `/ssc/turn_signal_command` | `automotive_platform_msgs/TurnSignalCommand` | Turn signal command to SSC. |
| `/ssc/gear_select` | `automotive_platform_msgs/GearCommand` | Gear commands to SSC. |
| `vehicle_status` | `autoware_msgs/VehicleStatus` | Status information from SSC. |

## button_panel

Reacts to engage button in the car. Also logs marker button presses.


## Parameters

| Name              | Type  | Default | Description |
|-------------------|-------|---------|-------------|
| cooldown          | float |   `2.0` | Cooldown period (in seconds) after pressing the engage button. Prevents infinitely delaying the engage by pressing the button repeatedly. |

#### Subscribed Topics

| Name           | Type                      | Description |
|----------------|---------------------------|-------------|
| current_pose   | geometry_msgs/PoseStamped | The current pose of the vehicle. Used to set the pose of markers. |
| joy            | sensor_msgs/Joy            | The joystick input for button presses. |

#### Published Topics

| Name              | Type                | Description |
|-------------------|---------------------|-------------|
| engage            | std_msgs/Bool       | Sends a signal to engage the autonomy. |
| log/markers       | visualization_msgs/Marker | Marker messages to log a button press. |
