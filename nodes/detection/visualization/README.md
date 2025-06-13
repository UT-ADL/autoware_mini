# Detection - visualization


## detected_objects_visualizer

ROS node to visualize detected objects by publishing markers and bounding boxes to RViz.


#### Parameters

No parameters.


#### Subscribed Topics

| Name | Type | Description |
| --- | --- | --- |
| `detected_objects` | `autoware_mini/DetectedObjectArray` | Detected objects to be visualized. |


#### Published Topics

| Name | Type | Description |
| --- | --- | --- |
| `detected_objects_markers` | `visualization_msgs/MarkerArray` | Markers of detected objects for visualization in RViz. |
| `detected_objects_bboxes` | `jsk_recognition_msgs/BoundingBoxArray` | 3D bounding boxes of detected objects for visualization in RViz. |


## predicted_trajectory_visualizer

ROS node to visualize predicted trajectories of detected objects by publishing markers to RViz.


#### Parameters

| Name | Type | Default Value | Description |
| --- | --- | --- | --- |
| `/planning/use_object_width` | bool | - | Whether to use object width for trajectory visualization. |


#### Subscribed Topics

| Name | Type | Description |
| --- | --- | --- |
| `predicted_objects` | `autoware_mini/DetectedObjectArray` | Predicted objects whose trajectories will be visualized. |


#### Published Topics

| Name | Type | Description |
| --- | --- | --- |
| `predicted_objects_markers` | `visualization_msgs/MarkerArray` | Markers of predicted trajectories for visualization in RViz. |
