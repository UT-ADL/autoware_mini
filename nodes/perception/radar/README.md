# Perception - Radar

## radar_detector

ROS node that processes radar tracks and publishes detected objects. It applies consistency filtering to ensure radar detections are stable over multiple frames.

#### Parameters

| Name | Type | Default Value | Description |
| ----- | ----- | ------------- | ------------ |
| `output_frame` | string | `"map"` | Target frame for the detected objects. |
| `consistency_check` | int | `5` | Number of consecutive detections over which a radar object is detected before it is published/used further. |
| `radar_time_offset` | float | `-0.07` | Time offset for radar objects to align with other sensors (s). |
| `transform_timeout` | float | `0.06` | Timeout in seconds for waiting for a transform to become available. |

#### Subscribed Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `/radar_fc/radar_tracks` | `radar_msgs/RadarTracks` | Radar track data containing position, velocity, and classification information. |
| `/localization/current_velocity` | `geometry_msgs/TwistStamped` | Current ego vehicle velocity used for velocity transformations. |

#### Published Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `detected_objects` | `autoware_mini/DetectedObjectArray` | Detected objects from radar. Each object includes its id, position, velocity, acceleration, and dimensions. |

## lidar_radar_fusion

ROS node that fuses lidar and radar detection results. Radar provides velocity and acceleration information for objects detected by lidar, and also contributes additional moving objects that lidar might have missed.

#### Parameters

| Name | Type | Default Value | Description |
| ----- | ----- | ------------- | ------------ |
| `max_euclidean_distance` | float | `2.0` | Maximum distance in meters for associating objects. |
| `angular_velocity_threshold` | float | `0.1` | Threshold for filtering out radar objects based on ego vehicle's angular velocity (rad/s). |
| `radar_speed_threshold` | float | `1.0` | Threshold for filtering out stationary unmatched radar objects based on their speed (m/s). |

#### Subscribed Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `radar/detected_objects` | `autoware_mini/DetectedObjectArray` | Objects detected by the radar detector. |
| `lidar/detected_objects` | `autoware_mini/DetectedObjectArray` | Objects detected by lidar processing nodes. |

#### Published Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `detected_objects` | `autoware_mini/DetectedObjectArray` | Fused detected objects. Includes lidar objects with added velocity and acceleration from radar when available, plus additional moving objects detected only by radar. |
