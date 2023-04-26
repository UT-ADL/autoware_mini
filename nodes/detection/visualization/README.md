# Detection - visualization


## detected_objects_visualizer


ROS node to visualize detected objects by publishing markers to RViz.

#### Parameters

No parameters.


#### Subscribed Topics

| Name | Type | Description |
| --- | --- | --- |
| `/detected_objects` | `autoware_msgs/DetectedObjectArray `| Detected objects to be visualized. |


#### Published Topics

| Name | Type | Description |
| --- | --- | --- |
| `/detected_objects_markers` | `visualization_msgs/MarkerArray `| Markers of detected objects for visualization in RViz. |
