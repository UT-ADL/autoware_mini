# Detection Module

The Detection module is responsible for identifying and tracking objects around the vehicle, as well as predicting their future motion.

![Detection Architecture](/images/nodes/detection_architecture.png)

<details>
    <summary><b>Click here for details</b></summary>

## Architecture

The detection module takes inputs from:
- **LiDAR detections**: Raw LiDAR point cloud data
- **Radar data**: Raw radar tracks from the vehicle radar sensor
- **Localization module**: Provides vehicle position (`/localization/current_pose`) and velocity (`/localization/current_velocity`) data
- **Map data**: Uses static map data (Lanelet2 format) for context-aware filtering and prediction

It produces outputs to:
- **Local Planning module**: Supplies the final detected objects array (`predicted_objects`) and their predicted candidate trajectories for local planner consideration

## Components

![Detection Pipeline](/images/nodes/detection.png)

### Detection
The detection subsystem processes raw sensor data to identify objects in the environment. The system operates on either raw lidar or raw radar data, or a combination of both.

#### LiDAR point clouds Processing Methods
The module provides three different methods for LiDAR point cloud processing:

##### Cluster-Based Detection (lidar_cluster)
A traditional approach that segments and clusters point clouds into separate objects:
- **Ground Removal**: Filters out ground points using either [JCP](https://www.mdpi.com/2072-4292/13/16/3239) (Jump-Convolution-Process) or naive method
- **Point Clustering**: Groups remaining points into clusters using DBSCAN algorithm
- **Cluster detector**: Creates object detections and corresponding bounding boxes from points' clusters

##### Neural Network-Based Detection (lidar_sfa)
Uses a deep learning approach for object detection:
- **BEV Conversion**: Transforms 3D point clouds into 2D Bird's Eye View representation
- **SFA Detection**: Applies [SFA3D](https://github.com/maudzung/SFA3D) neural network for object detections
- **Post-processing**: Filters detections by certain thresholds (e.g., class confidence, size) and refines final predictions

##### Vella LiDAR Integration (lidar_vella)
Integrates [the Vella LiDAR processing system](https://visimind.com/products/lidar-software/) for object detection:
- **Track Conversion**: Converts Vella track objects to the detected objects format
- **Confidence Filtering**: Filters detections based on confidence score and track length

#### Radar Processing (radar_detector)
Processes raw radar tracks to identify and track objects:
- **Consistency Filtering**: Ensures radar detections persist across multiple frames before considering them valid
- **Velocity Computation**: Combines ego-vehicle velocity with relative radar measurements to obtain map-frame velocities
- **Object Generation**: Creates object detections and corresponding bounding boxes from persistent radar tracks

#### LiDAR-Radar Fusion
Combines detections from both LiDAR and radar sensors:
- **Association**: Matches radar and LiDAR detections using either IOU or Euclidean distance metrics
- **Data Enhancement**: Enhances LiDAR detections with velocity and acceleration data from radar
- **Complementary Detection**: Adds radar-only detections for objects not seen by LiDAR, filtering by velocity threshold

### Filtering
Filtering improves detection quality by removing false positives and irrelevant detections.

- **Road Area Filter**: Filters objects based on whether they are on or off the drivable road area.
- **Detection Range Filter**: Restricts detections to objects within a relevant operational range.

### Tracking
Tracking maintains detected object identity across multiple frames, providing temporal consistency and enabling velocity estimation.

- **EMA Tracker**: Uses an Exponential Moving Average approach to track objects across frames, utilizing spatial association and velocity propagation.

### Prediction
Prediction estimates the future positions and trajectories of tracked objects.

- **Naive Predictor**: Implements a physics-based constant velocity/constant acceleration model for trajectory prediction.
- **Map-Based Predictor**: Uses map information (lanes, intersections) to generate more realistic trajectory predictions based on the adjacent road structure extracted from Lanelet2 map.

## Data Flow

1. Detection module receives raw sensor data from LiDAR and/or radar
2. LiDAR point clouds are processed using one of the three methods (cluster, SFA, or Vella) and optionally fused with processed detected objects from radar data
3. (Optional) Detected objects are filtered using road area and/or detection range filters
4. Detected objects are tracked using the EMA Tracker, which maintains object identity and improves position, velocity and acceleration estimates
5. Prediction algorithms (Naive Predictor or Map-Based Predictor) generate candidate trajectories for each tracked object
6. Final detected objects with candidate trajectories are published to the local planner module

</details>


# Detection - Traffic Lights

The Traffic Light Detection module is responsible for identifying traffic lights and their states to allow the vehicle to comply with traffic regulations.

![Traffic Light Detection Architecture](/images/nodes/detection_tld.png)

<details>
    <summary><b>Click here for details</b></summary>

## Architecture

The traffic light detection module takes inputs from:
- **Camera images**: Raw camera images from the vehicle cameras
- **Local planner module**: Uses local path information to determine relevant traffic lights
- **Map data**: Uses static map data (Lanelet2 format) to locate nearby traffic lights in the environment
- **External sources**: Can optionally receive traffic light states from external systems via MQTT

It produces outputs to:
- **Local Planning module**: Provides traffic light status information for path planning

## Components

### Camera-based Traffic Light Detection
The module provides two different machine-learning-based approaches for traffic light detection from camera images:

#### Projection-based Detection and Neural Network Classifier (camera_traffic_light_detector)
Uses a deep learning object detection approach that:
- Identifies traffic light locations using map-based projection
- Creates ROIs (Regions of Interest) around projected traffic light locations
- Applies a neural network classifier to determine traffic light state (red, yellow, green, unknown)

#### YOLO-based Detection and Neural Network Classifier (yolo_traffic_light_detector)
Uses a deep learning object detection approach that:
- Detects traffic lights in the entire image using [YOLO](https://arxiv.org/abs/1804.02767)
- Matches detections with map-based traffic light locations using IoU
- Applies a neural network classifier to determine traffic light state (red, yellow, green, unknown)

#### External Traffic Light Integration (mqtt_traffic_light_detector)
Integrates with external traffic light information systems:
- Receives traffic light states from IoT infrastructure via MQTT protocol
- Supports both JSON and binary message formats
- Can dynamically subscribe to relevant nearby traffic lights information based on vehicle position
- Provides fallback mechanisms for handling timeouts and communication issues

### Traffic Light Fusion

#### Majority Merger (traffic_light_majority_merger)
Combines results from multiple camera-based traffic light detectors:
- Takes input from multiple camera sources
- Uses voting to determine the most likely traffic light state
- Prioritizes red/yellow states over green in case of equal votes

#### Priority Merger (traffic_light_priority_merger)
Combines results with priority-based selection:
- Takes input from two traffic light detection sources
- Prioritizes results from the first source (high priority, usually MQTT) over the second source
- Only uses results from the second source when the first source reports "UNKNOWN" state

## Data Flow

1. Relevant traffic light states are received from external sources via MQTT
2. (Optional) Traffic light detection module receives raw camera images and map data, projecting traffic light locations onto the images and classifying them using the neural network classifier
3. Traffic light states sourced from multiple methods are fused using either majority or priority merger methods
4. Final traffic light states are published to the local planner module for path planning

</details>