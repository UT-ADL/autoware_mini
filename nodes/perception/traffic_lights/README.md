# Perception - Traffic Lights

A collection of nodes for detecting traffic lights using camera images and MQTT service. The nodes include camera-based detection, integration with external MQTT traffic light information system, and merging of results from multiple sources.

![TLD merging figure](/images/nodes/detection_tld_merging.png)


## camera_traffic_light_detector

ROS node that detects traffic lights using camera images. It projects traffic light positions from the map onto the camera image, extracts regions of interest (ROIs), and classifies the traffic light state using a neural network.


#### Parameters

| Name | Type | Default Value | Description |
| ----- | ----- | ------------- | ------------ |
| `~onnx_path` | string | - | Path to the ONNX model file for traffic light classification. |
| `~lanelet2_map_path` | string | - | Path of the Lanelet2 map file. |
| `~rectify_image` | bool | `False` | Whether to rectify the image before processing. |
| `~roi_width_extent` | float | `0.65` | ROI box width extension amount in meters. |
| `~roi_height_extent` | float | `0.55` | ROI box height extension amount in meters. |
| `~min_roi_width` | int | `50` | Minimum ROI width in pixels for classification to be performed. |
| `~transform_timeout` | float | `0.06` | Timeout in seconds for waiting for a transform to become available. |
| `~camera_delay_compensation` | float | `0.03` | Compensation time for camera delay in seconds. |


#### Subscribed Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `camera_info` | `sensor_msgs/CameraInfo` | Camera information including calibration parameters. |
| `/planning/local_path` | `autoware_mini/Path` | Local path used to determine relevant traffic lights. |
| `image_raw` | `sensor_msgs/Image` | Raw camera image used for traffic light detection. |


#### Published Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `traffic_light_status` | `autoware_mini/StopLineStatusArray` | Array of detected traffic light states. |
| `traffic_light_roi` | `sensor_msgs/Image` | Visualization of the regions of interest and detection results. |


## mqtt_traffic_light_detector

ROS node that subscribes to MQTT topic, processes received messages and publishes traffic light status. Traffic light status from MQTT service is aggregated into signal groups that directly correspond to stop lines.


#### Parameters

| Name | Type | Default Value | Description |
| ----- | ----- | ------------- | ------------ |
| `~lanelet2_map_path` | string | - | Name of the Lanelet2 map. |
| `~mqtt_host` | string | `traffic.traffest.com` | Hostname or IP address of the MQTT broker. |
| `~mqtt_port` | int | `8883` | Port number of the MQTT broker. |
| `~mqtt_topic` | string | `Tartu/#` | MQTT topic to subscribe to. |
| `~enable_automatic_subscribe` | bool | `True` | Whether to automatically subscribe to traffic lights based on vehicle position. |
| `~automatic_subscription_range` | float | `200` | Range in meters for automatic subscription to traffic lights. |
| `~timeout` | float | `2.0` | Time in seconds before timing out while waiting for a message. |
| `~id_string` | string | ` mqtt` | String added to traffic light result and displayed in rviz. |


#### Subscribed Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `/localization/current_pose` | `geometry_msgs/PoseStamped` | Current vehicle pose (only used when enable_automatic_subscribe is True). |


#### Published Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `traffic_light_status` | `autoware_mini/StopLineStatusArray` | Array of traffic light results. |


## traffic_light_majority_merger

ROS node that merges traffic light results from two cameras using majority voting strategy. For each stop line, it counts the detection results from both cameras and selects the most frequent one.


#### Parameters

| Name | Type | Default Value | Description |
| ----- | ----- | ------------- | ------------ |
| `~id_string` | string | ` cam` | String added to traffic light result and displayed in rviz. |


#### Subscribed Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `camera1/traffic_light_status` | `autoware_mini/StopLineStatusArray` | Traffic light results from the first camera. |
| `camera2/traffic_light_status` | `autoware_mini/StopLineStatusArray` | Traffic light results from the second camera. |


#### Published Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `traffic_light_status` | `autoware_mini/StopLineStatusArray` | Merged traffic light results. |


## traffic_light_priority_merger

ROS node that merges traffic light results from two sources based on priority rules. It prioritizes non-unknown results from the first source, falling back to the second source when needed.


#### Parameters

This node has no specific parameters.


#### Subscribed Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `tfl_status_topic_1` | `autoware_mini/StopLineStatusArray` | Traffic light results from the first (priority) source. |
| `tfl_status_topic_2` | `autoware_mini/StopLineStatusArray` | Traffic light results from the second source. |


#### Published Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `traffic_light_status` | `autoware_mini/StopLineStatusArray` | Merged traffic light results based on priority rules. |


## yolo_traffic_light_detector

ROS node that detects traffic lights using a YOLO model. It projects traffic light positions from the map onto the camera image and matches them with YOLO detections to determine traffic light states.


#### Parameters

| Name | Type | Default Value | Description |
| ----- | ----- | ------------- | ------------ |
| `~onnx_path` | string | - | Path to the ONNX model file for YOLO traffic light detection. |
| `~lanelet2_map_path` | string | - | Name of the Lanelet2 map file. |
| `~rectify_image` | bool | `False` | Whether to rectify the image before processing. |
| `~roi_width_extent` | float | `0.65` | ROI box width extension amount in meters. |
| `~roi_height_extent` | float | `0.55` | ROI box height extension amount in meters. |
| `~min_roi_width` | int | `15` | Minimum ROI width in pixels for detection to be considered. |
| `~transform_timeout` | float | `0.06` | Timeout in seconds for waiting for a transform to become available. |
| `~iou_threshold` | float | `0.05` | Threshold for IOU-based matching between map and YOLO ROIs. |
| `~camera_delay_compensation` | float | `0.2` | Compensation time for camera delay in seconds. |


#### Subscribed Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `camera_info` | `sensor_msgs/CameraInfo` | Camera information including calibration parameters. |
| `/planning/local_path` | `autoware_mini/Path` | Local path used to determine relevant traffic lights. |
| `image_raw` | `sensor_msgs/Image` | Raw camera image used for traffic light detection. |


#### Published Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `traffic_light_status` | `autoware_mini/StopLineStatusArray` | Array of detected traffic light states. |
| `traffic_light_roi` | `sensor_msgs/Image` | Visualization of the regions of interest, YOLO detections, and matching results. |
