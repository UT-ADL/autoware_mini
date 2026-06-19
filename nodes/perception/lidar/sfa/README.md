# Perception - SFA Detector

## sfa_detector

ROS node that performs LiDAR-based object detection using a Super Fast and Accurate [SFA](https://github.com/maudzung/SFA3D/) model loaded via ONNX. It processes point cloud data, generates BEV (Bird's Eye View) maps, performs object detection, and publishes detected 3D objects.

#### Parameters

| Name | Type | Default Value | Description |
| ----- | ----- | ------------- | ------------ |
| `~onnx_path` | string | - | Path to the trained ONNX model file |
| `~min_z` | float | `-2.73` | Minimum z-axis value for processing points |
| `~max_z` | float | `1.27` | Maximum z-axis value for processing points |
| `~score_thresh` | float | `0.2` | Score threshold for filtering detections |
| `~top_k` | int | `50` | Number of top scoring detections to process |
| `/perception/output_frame` | string | `map` | Target frame for the detected objects |
| `~transform_timeout` | float | `0.06` | Timeout in seconds for waiting for a transform to become available |

#### Subscribed Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `points_raw` | `sensor_msgs/PointCloud2` | Raw LiDAR point cloud data |

#### Published Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `detected_objects` | `autoware_mini/DetectedObjectArray` | Detected objects with classification (car, pedestrian, etc.), position, dimensions, orientation, and confidence score |

#### Key Features

- Supports both short-distance (608x608) and long-distance (1216x1216) models
- Generates BEV maps from 3D point clouds
- Performs object detection with 12 different classes
- Transforms detection results to the desired output frame
- Provides configurable score thresholding and detection limits
