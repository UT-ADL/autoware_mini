# Global plannning

Currently there is one global planner based on Lanelet2 map.

## lanelet2_global_planner

Uses Lanelet2 map to generate the global path. Currently loads map itself. `/current_pose` is used as a starting point and multiple goals can be added. Each added goal will add the new route segment from previous goal to global path.

##### Parameters

* `lanelet2_map` - full path to map file `*.osm`
* `distance_to_centerline_limit` - used to check start point and goal point distances from the lanelet centerline. If limit exceeded the goal point is not considered. 


##### Subscribes

| Topic | Type | Comment |
| --- | --- | --- |
| `goal` |  [geometry_msgs/PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html) | goal point pose entered in Rviz or published to topic |
| `current_pose` | [geometry_msgs/PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html) | current location and orientation of the car in `map` frame |
| `cancel_global_path` | `Bool` | If `True` is sent then the global path will be cleared and empty message is published to `path` |

##### Publishes

| Topic | Type | Comment |
| --- | --- | --- |
| `path` | `autoware_msgs/Lane` | Lane - array of waypoints with global path. Path is visualized using `waypoint_visualizer` node. |

## lanelet2_map_loader and visualizer

`lanelet2_map_loader` will load lanelet2 map and call also the `lanelet2_map_visualizer` that will create the markers for rviz visualization.

##### Parameters

* `lanelet2_map` - full path to map file `*.osm`

##### Publishes

| Topic | Type | Comment |
| --- | --- | --- |
| `lanelet2_map_markers` | `visualization_msgs/MarkerArray` | Visualizes different lanelet2 map elemets, like: lanelet centerlines, left- and right edges, traffic light bulbs, stop lines and crosswalks. |