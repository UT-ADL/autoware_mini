# Detection - LiDAR Cluster

A collection of ROS nodes for processing LiDAR point clouds, performing objects detection using clustering methods. Steps include ground removal, clustering using DBSCAN method, and detected object construction.
![Cluster Detection Pipeline](/images/nodes/detection_cluster.png)

## points_concatenator

ROS node that concatenates two point clouds into one.

#### Parameters

No explicitly defined parameters in code.

#### Subscribed Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `points1` | `sensor_msgs/PointCloud2` | First point cloud to be concatenated. |
| `points2` | `sensor_msgs/PointCloud2` | Second point cloud to be concatenated. |

#### Published Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `points_concatenated` | `sensor_msgs/PointCloud2` | Concatenated point cloud from the two input point clouds. |

## points_clusterer

ROS node that performs clustering on filtered point cloud data using DBSCAN algorithm.

#### Parameters

| Name | Type | Default Value | Description |
| ----- | ----- | ------------- | ------------ |
| `~cluster_epsilon` | float | `0.6` | The maximum distance between two points to be considered as part of the same cluster. |
| `~cluster_min_size` | int | `4` | Minimum number of points required to form a cluster. |
| `~cluster_in_2d` | bool | `True` | Whether to perform clustering in 2D (x-y plane) or 3D. |

#### Subscribed Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `points_filtered` | `sensor_msgs/PointCloud2` | Filtered point cloud data for clustering. |

#### Published Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `points_clustered` | `sensor_msgs/PointCloud2` | Clustered point cloud data with labels. |

## naive_ground_removal

ROS node that removes ground points from a point cloud using a grid-based method.

#### Parameters

| Name | Type | Default Value | Description |
| ----- | ----- | ------------- | ------------ |
| `~min_x` | float | `-60.0` | Minimum x value of the points to consider. |
| `~max_x` | float | `75.0` | Maximum x value of the points to consider. |
| `~min_y` | float | `-60.0` | Minimum y value of the points to consider. |
| `~max_y` | float | `60.0` | Maximum y value of the points to consider. |
| `~min_z` | float | `-2.5` | Minimum z value of the points to consider. |
| `~max_z` | float | `0.05` | Maximum z value of the points to consider. |
| `~cell_size` | float | `0.6` | Size of grid cells used for ground estimation. |
| `~tolerance` | float | `0.15` | Maximum distance to ground level for a point to be considered ground. |
| `~filter` | string | `average` | Filter method to use ('none', 'median', 'average', 'minimum'). |
| `~filter_size` | int | `3` | Size of the filter kernel. |
| `~filter_iterations` | int | `1` | Number of iterations for the filter. |

#### Subscribed Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `points_raw` | `sensor_msgs/PointCloud2` | Raw point cloud data. |

#### Published Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `points_ground` | `sensor_msgs/PointCloud2` | Ground points extracted from the point cloud. |
| `points_no_ground` | `sensor_msgs/PointCloud2` | Non-ground points from the point cloud. |

## jcp_ground_removal

ROS node that removes ground points from a point cloud using [JCP](https://github.com/wangx1996/Fast-Ground-Segmentation-Based-on-JPC) (Jump-Convolution-Process) algorithm.

#### Parameters

| Name | Type | Default Value | Description |
| ----- | ----- | ------------- | ------------ |
| `~range_img_width` | int | `1024` | Width of the range image. |
| `~range_img_height` | int | `32` | Height of the range image. |
| `~sensor_height` | float | `2.11` | Height of the LiDAR sensor from the ground. |
| `~delta_R` | float | `1` | Parameter for jump point detection. |

#### Subscribed Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `points_raw` | `sensor_msgs/PointCloud2` | Raw point cloud data. |

#### Published Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `points_ground` | `sensor_msgs/PointCloud2` | Ground points extracted from the point cloud. |
| `points_no_ground` | `sensor_msgs/PointCloud2` | Non-ground points from the point cloud. |

## cluster_detector

ROS node that detects objects from clustered point cloud data.

#### Parameters

| Name | Type | Default Value | Description |
| ----- | ----- | ------------- | ------------ |
| `~min_cluster_size` | int | `4` | Minimum number of points in a cluster. Clusters smaller than this will be ignored. |
| `~bounding_box_type` | string | `min_area` | Type of bounding box. Can be 'axis_aligned' or 'min_area'. |
| `~transform_timeout` | float | `0.06` | Timeout in seconds for waiting for a transform to become available. |
| `/detection/output_frame` | string | `map` | Target frame for the detected objects. |

#### Subscribed Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `points_clustered` | `sensor_msgs/PointCloud2` | Clustered point cloud data with labels. |

#### Published Topics

| Name | Type | Description |
| ----- | ----- | ------------ |
| `detected_objects` | `autoware_mini/DetectedObjectArray` | Detected objects with their properties including position, heading, dimensions, and convex hull. |
