# Perception - Prediction

This directory contains ROS nodes for predicting the future movement of detected and tracked objects.

## naive_predictor

ROS node that predicts future object trajectories using a constant velocity or constant acceleration model.

#### Parameters

| Name                    | Type | Default Value | Description |
|-------------------------| ----- | ------------- | ------------ |
| `~prediction_horizon`   | float | `3.0` | Prediction time horizon in seconds. |
| `~prediction_interval`  | float | `3.0` | Time interval between prediction points in seconds. |
| `~prediction_min_speed` | float | `1.0` | Minimum speed threshold in m/s. Objects moving slower than this will not get predictions. |

#### Subscribed Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `tracked_objects` | `autoware_mini/DetectedObjectArray` | Tracked objects to generate predictions for. |

#### Published Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `predicted_objects` | `autoware_mini/DetectedObjectArray` | Objects with added candidate trajectories based on constant velocity/acceleration model. |

## map_based_predictor

ROS node that predicts future object trajectories using map information from the lanelet2 map. Objects are matched to candidate lanelets using a weighted cost function that considers cross-track error, heading difference, and speed difference.

#### Parameters

| Name                           | Type | Default Value | Description |
|--------------------------------| ----- | ------------- | ------------ |
| `~lanelet2_map_path`           | string | `-` | Path of the lanelet2 map file to load. |
| `~prediction_horizon`          | float | `5.0` | Prediction time horizon in seconds. |
| `~prediction_interval`         | float | `0.5` | Time interval between prediction points in seconds. |
| `~trajectories_to_predict`     | int | `1` | Number of trajectories to predict for each object. |
| `~prediction_min_speed`        | float | `1.0` | Minimum speed threshold in m/s. Objects moving slower than this will not get predictions. |
| `~distance_from_lanelet`       | float | `0.0` | Search radius in meters beyond lanelet borders for finding candidate lanelets. |
| `~heading_difference_threshold` | float | `30.0` | Maximum allowed heading difference in degrees between object and lanelet. Lanelets exceeding this threshold are excluded from matching. |
| `~prediction_clipping_deceleration_limit` | float | `2.8` | Maximum deceleration in m/s² used when clipping predictions at stop lines. |
| `~max_height_difference`       | float | `3.0` | Maximum height difference in meters for lanelet matching. Set to 0 to disable height filtering. Useful for multi-level road scenarios. |
| `~matching_cross_track_weight` | float | `1.0` | Weight for cross-track error (lateral offset) in the lanelet matching cost function. |
| `~matching_heading_weight`     | float | `2.0` | Weight for heading difference in the lanelet matching cost function. |
| `~matching_speed_weight`       | float | `0.5` | Weight for speed difference in the lanelet matching cost function. |

#### Subscribed Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `tracked_objects` | `autoware_mini/DetectedObjectArray` | Tracked objects to generate predictions for. |

#### Published Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `predicted_objects` | `autoware_mini/DetectedObjectArray` | Objects with added map-based candidate trajectories. |
