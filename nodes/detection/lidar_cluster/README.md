# Detection - lidar_cluster


## cluster_detector

ROS node that clusters point cloud data and publishes detected objects.


#### Parameters

| Name | Type | Default Value | Description |
| ----- | ----- | ------------- | ------------ |
| `min_cluster_size` | int | `5` | Minimum number of points in a cluster. Clusters smaller than this will be ignored. |
| `bounding_box_type` | string | `"axis_aligned"` | Type of bounding box. It can be `"axis_aligned"` for axis-aligned bounding box, or `"min_area"` for minimum area bounding box. |
| `enable_pointcloud` | bool | `False` | Whether to publish point cloud data for each detected object. |
| `enable_convex_hull` | bool | `True` | Whether to calculate a convex hull for the cluster points. |
| `output_frame` | string | `"map"` | Target frame for the detected objects. |
| `transform_timeout` | float | `0.05` | Timeout in seconds for waiting for a transform to become available.


#### Subscribed Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `/points_clustered` | `sensor_msgs/PointCloud2` | Clustered point cloud data.


#### Published Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `/detected_objects` | `autoware_msgs/DetectedObjectArray` | Detected objects. Each object includes its label, position, orientation, dimensions, and color. The `valid` flag indicates whether the object is valid. If `enable_pointcloud` is true, each object also includes a point cloud representing the cluster of points belonging to that object. |



## ground_removal

ROS node for filtering ground points from a 3D point cloud.


#### Parameters

| Name        | Type   | Default | Description                                               |
| ----------- | ------ | ------- | --------------------------------------------------------- |
| `~min_x`    | float  | -60.0   | Minimum x value of the points to consider                 |
| `~max_x`    | float  | 60.0    | Maximum x value of the points to consider                 |
| `~min_y`    | float  | -60.0   | Minimum y value of the points to consider                 |
| `~max_y`    | float  | 60.0    | Maximum y value of the points to consider                 |
| `~min_z`    | float  | -2.0    | Minimum z value of the points to consider                 |
| `~max_z`    | float  | 1.0     | Maximum z value of the points to consider                 |
| `~cell_size`| float  | 1.0     | The size of a cell in the grid used for filtering          |
| `~tolerance`| float  | 0.1     | Maximum distance to ground level for a point to be ground |
| `~filter`   | string | 'none'  | Filtering method for smoothing ground levels              |


#### Subscribed Topics

| Name         | Type                               | Description                                          |
| ------------ | ---------------------------------- | ---------------------------------------------------- |
| `/points_raw`| `sensor_msgs/PointCloud2`          | The raw point cloud to filter                        |


#### Published Topics

| Name                 | Type                               | Description                                                      |
| -------------------- | ---------------------------------- | ---------------------------------------------------------------- |
| `/points_ground`     | `sensor_msgs/PointCloud2`          | The ground points of the filtered point cloud                     |
| `/points_no_ground`  | `sensor_msgs/PointCloud2`          | The non-ground points of the filtered point cloud                 |



## points_clusterer

This node subscribes to a point cloud topic and clusters the points using DBSCAN algorithm. It downsamples the input data to a fixed size before clustering, to reduce processing time. It publishes the clustered point cloud data to a separate topic.


#### Parameters

| Name | Type | Default value | Description |
| --- | --- | --- | --- |
| `~sample_size` | int | `10000` | Number of points to downsample from the input point cloud. |
| `~cluster_epsilon` | float | `1.0` | The maximum distance between two points to be considered as part of the same cluster. |
| `~cluster_min_size` | int | `7` | Minimum number of points required to form a cluster. |


#### Subscribed Topics

| Name | Type | Description |
| --- | --- | --- |
| `/points_no_ground` | `sensor_msgs/PointCloud2` | The input point cloud data. |


#### Published Topics

| Name | Type | Description |
| --- | --- | --- |
| `/points_clustered` | `sensor_msgs/PointCloud2` | The clustered point cloud data. |

