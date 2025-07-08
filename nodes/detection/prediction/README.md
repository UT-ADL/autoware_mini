# Detection - Prediction

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

ROS node that predicts future object trajectories using map information from the lanelet2 map.

#### Parameters

| Name                           | Type | Default Value | Description |
|--------------------------------| ----- | ------------- | ------------ |
| `~lanelet2_map_path`           | string | `-` | Path of the lanelet2 map file to load. |
| `~prediction_horizon`          | float | `5.0` | Prediction time horizon in seconds. |
| `~prediction_interval`         | float | `0.5` | Time interval between prediction points in seconds. |
| `~trajectories_to_predict`     | int | `1` | Number of trajectories to predict for each object. |
| `~prediction_min_speed`        | float | `1.0` | Minimum speed threshold in m/s. Objects moving slower than this will not get predictions. |
| `~distance_from_lanelet`       | float | `0.0` | Maximum allowed distance from lanelet borders in meters for an object to be associated with it. |
| `~heading_difference_threshold` | float | `30.0` | Maximum allowed angle difference in degrees between object heading and lanelet heading. |
| `~use_offset_for_prediction`   | bool | `True` | Whether to use lateral offset from lanelet centerline for prediction. |

#### Subscribed Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `tracked_objects` | `autoware_mini/DetectedObjectArray` | Tracked objects to generate predictions for. |

#### Published Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `predicted_objects` | `autoware_mini/DetectedObjectArray` | Objects with added map-based candidate trajectories. |
