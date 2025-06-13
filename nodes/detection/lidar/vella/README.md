# Detection - vella

## vella_detector

ROS node that converts the Vella track messages to Autoware Detected Object messages by applying filtering, transforming and adding metadata to the messages.

### Parameters

| Name | Type | Default value | Description |
| --- | --- | --- | --- |
| `output_frame` | string | 'map' | Frame ID for Autoware DetectedObjectArray. |
| `~confidence_filter` | double | 0.5 | Filter out Vella tracks with a confidence score less than this threshold. |
| `~track_length_filter` | int | 0 | Filter out Vella tracks with track lengths less than this threshold. |
| `~lidar_frame` | string | 'lidar_center' | Frame ID for Vella tracks. Vella does not populate the frame ID of VDK/tracks messages. |
| `~transform_timeout` | double | 0.06 | Transform timeout when transforming poses. |

### Subscribed Topics

| Name | Type | Description |
| --- | --- | --- |
| `/vdk/tracks` | `vella_msgs/Track3DArray` | Vella tracks to be converted to Autoware DetectedObject messages |

### Published Topics

| Name | Type | Description |
| --- | --- | --- |
| `detected_objects` | `autoware_mini/DetectedObjectArray` | Autoware DetectedObjectArray generated from Vella tracks. |
