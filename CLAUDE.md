# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Guidelines for Claude

- **Action**: Do not jump into implementation or change files unless clearly instructed to make changes. When the user's intent is ambiguous, default to providing information, doing research, and providing recommendations rather than taking action.
- **Code review**: Keep code reviews concise. Focus only on problems; skip complimenting the user for following the guidelines.

## Project Overview

Autoware Mini is a minimalistic Python-based autonomous vehicle software stack built on ROS 1 (Noetic). It's designed for teaching and research, validated with real-world deployment on a Lexus vehicle in Tartu, Estonia.

## Repository

- **GitLab**: `autonomous-driving-lab/autoware_mini` (project ID: 2286) at gitlab.cs.ut.ee - main branch: `main`
- **GitHub mirror**: `UT-ADL/autoware_mini` - main branch: `release`
- **Merge requests**: Unless specified otherwise, create merge requests for merging into `main` branch (GitLab)

## Build Commands

```bash
# Build the workspace (run from ~/autoware_mini_ws)
catkin build

# Release build for optimized performance
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace after building
source devel/setup.bash
```

## Running the System

```bash
# Lightweight planner simulation (no GPU required)
roslaunch autoware_mini start_sim.launch

# With real-time traffic lights from Tartu
roslaunch autoware_mini start_sim.launch tfl_detector:=mqtt

# Bag playback (recorded sensor data)
roslaunch autoware_mini start_bag.launch
roslaunch autoware_mini start_bag.launch detector:=lidar_sfa  # GPU detector

# CARLA simulation (requires CARLA_ROOT set)
roslaunch autoware_mini start_carla.launch

# Real Lexus vehicle
roslaunch autoware_mini start_lexus.launch
```

Common launch arguments: `detector:=` (lidar_cluster, lidar_sfa, radar, lidar_cluster_radar_fusion), `tfl_detector:=` (none, mqtt, camera, yolo)

### Restarting the System

If ROS processes are left hanging after a crash or interrupted launch:

```bash
# Kill all existing ROS nodes
rosnode kill -a

# Kill the rosmaster
pkill roscore

# Verify rosmaster is not running (should show error)
rostopic list
```

### Running on Neuron Server (neuron.hpc.ut.ee)

1. Launch scripts without RViz (no display available):
   - For launch files: `launch_rviz:=false`
   - For bash scripts: `--no_rviz`

2. Bag file locations:
   - `/data/bag_cache` - Locally cached bag files
   - `/data/Bolt/bagfiles` - All bag files ever recorded

## Architecture

The stack follows a modular pipeline:

```
SENSORS → LOCALIZATION → PERCEPTION → GLOBAL PLANNING → LOCAL PLANNING → CONTROL → VEHICLE
```

### Key Modules (all in nodes/)

- **localization/** - Vehicle position via GNSS (Novatel OEM7) or simulation ground truth
- **perception/** - Object detection (lidar_cluster, lidar_sfa neural network, radar), tracking (EMA), prediction, traffic lights
- **planning/global/** - Route planning using Lanelet2 maps, lane changes, waypoint management
- **planning/local/** - Trajectory generation with 17+ rule-based safety checkers (collision, traffic lights, crosswalks, etc.)
- **control/** - Path following via Pure Pursuit or Stanley controllers

### Python Library (src/autoware_mini/)

Shared utilities: `geometry.py` (vector math), `lanelet2.py` (map ops), `path.py` (trajectory ops), `collision.py`, `detection.py`, `transform.py`, `visualization.py`

### Configuration (config/)

- `perception.yaml` - LiDAR/radar parameters, clustering, neural network settings
- `planning.yaml` - Safety distances, speed limits, checker parameters
- `control.yaml` - Controller gains
- `localization.yaml` - GNSS and coordinate system
- `vehicle.yaml` - Vehicle dimensions

### Custom Messages (msg/)

Key types: `DetectedObject`, `DetectedObjectArray`, `Path`, `Waypoint`, `VehicleCommand`, `VehicleStatus`, `StopLineStatus`

## Key Dependencies

- **lanelet2** - Critical for map-based planning
- **pyproj** - Coordinate transformations (WGS84/UTM/LEST97)
- **shapely** - Geometric operations and collision detection
- **onnxruntime-gpu** - Neural network inference (optional, for SFA detector)
- **numpy < 1.24** - Required due to deprecated np.float usage in some dependencies

## Testing

No formal unit test framework. Testing is done via scenario-based validation.

### Bag Scenario Testing (scripts/bag_scenarios/)

Closed-loop planner testing using detections from recorded bags. Run from scripts/bag_scenarios/:

```bash
./crosswalks_tests.sh       # Pedestrian crosswalk scenarios
./give_way_bus_tests.sh     # Bus give-way scenarios
./lexus_tests.sh            # Real Lexus recording scenarios
./objects_behind_tests.sh   # Objects behind ego vehicle
./planner_tests.sh          # General planner scenarios
./swerving_tests.sh         # Swerving maneuver scenarios
./tartu_demo_tests.sh       # Tartu demo map scenarios
./yielding_tests.sh         # Yielding scenarios
```

Options: `--rate <value>` (playback speed), `--no_rviz` (headless mode)

Metrics: ADE, FDE, collision score, max deceleration. Results (CSV + PNG plots) in `data/bag_scenarios/<map>/results/`. For running suites and committing results, use `/run_bag_scenarios`; for regenerating the source bags, use `/regenerate_bag_scenarios`.

### CARLA Scenario Testing

```bash
# With Scenario Runner
roslaunch autoware_mini start_carla.launch use_scenario_runner:=true

# Route-based scenarios
roslaunch autoware_mini start_carla.launch use_scenario_runner:=true route_id:=0
```

- **Bag scenarios** (data/bag_scenarios/) - Pre-recorded detections for planner testing
- **OpenSCENARIO** (data/scenarios/) - Structured test scenarios for CARLA
- **RViz visualization** - Primary debugging tool

## Benchmarking

Bag files are in `data/bags/`. Use just the bag filename (not full path) for `bag_file:=`. Standard benchmark bag: `2023-05-18-14-41-03_sensors_Kaubamaja_with_set_dest.bag`. For map visualization benchmarks, use `tartu_large.osm` (the largest map).

### Callback Benchmarking

Measures execution time of individual callbacks using `@time_callback` decorator from `autoware_mini.timer`. Add the decorator to the callback you want to measure, then remove it after benchmarking. Prints per-call and running average times to stdout.

```bash
PYTHONUNBUFFERED=1 roslaunch autoware_mini start_bag.launch bag_file:=<name>.bag launch_rviz:=false loop:=false 2>&1 > /tmp/output.log
grep "avg time" /tmp/output.log
```

Use `PYTHONUNBUFFERED=1` to ensure output is not buffered. Use `loop:=false` to stop after one playback.

### Delay Benchmarking

Measures end-to-end latency from sensor input to topic output using `rostopic delay` (difference between message header timestamp and receive time). The bag plays once automatically (no loop). Use `benchmark_type:=hz` to measure publish rate instead of delay.

```bash
roslaunch autoware_mini start_bag.launch benchmark_topic:=/planning/local_path bag_file:=2023-05-18-14-41-03_sensors_Kaubamaja_with_set_dest.bag launch_rviz:=false > /tmp/bench.log 2>&1
grep "average delay" /tmp/bench.log | tail -3
```

Both methods can be combined — adding `benchmark_topic` also disables looping, so `loop:=false` is not needed:

```bash
PYTHONUNBUFFERED=1 roslaunch autoware_mini start_bag.launch benchmark_topic:=/planning/local_path bag_file:=<name>.bag launch_rviz:=false 2>&1 > /tmp/output.log
grep -E "avg time|average delay" /tmp/output.log
```

## Development Notes

- All nodes are Python scripts in nodes/ subdirectories
- Launch files compose the full stack from modular components
- RViz is the primary debugging tool; configs in config/rviz/
- Maps stored as Lanelet2 format in data/maps/

## Code Style

### General Principles

- **Keep code short**: Shorter code is easier to comprehend, contains fewer bugs, and is usually faster due to fewer algorithmic steps and better use of optimized library functions.
- **KISS and DRY**: Keep It Simple Stupid; Don't Repeat Yourself. Avoid convoluted nested if-then-else clauses; minimize nesting depth and ensure each outcome is reached through one branch only.
- **One operation per line**: Do not pack too much into one line. Use intermediate variables to break up complex expressions and improve readability.
- **Minimize variables**: Avoid needless intermediate variables; reuse existing ones when possible.
- **Don't extract single-use helpers**: Don't introduce a new function/method that is only called from one place — inline it. Extract only when there are at least two callers or the inlined code is so long it harms readability.
- **Avoid trivial aliases**: Don't introduce a local variable that just renames an existing reference (e.g., `line = self.path.linestring` then `line.length`, `line.project(...)`). Inline the access. Only alias when it caches an expensive recomputation or the rename materially aids readability.

### Python Style

- **Imports**: Use module-level imports (`import numpy as np`, `import math`) for scientific libraries (numpy, scipy, shapely). Use `from X import Y` for: ROS message types, autoware_mini helpers, tf.transformations, ros_numpy, dynamic_reconfigure configs, and single-item stdlib imports like `from copy import deepcopy`. Never use `from X import *`. Order: 1) standard/third-party modules with rospy, 2) ROS imports (messages, tf, ros_numpy), 3) autoware_mini imports.
- **Scalar math**: Use `math` module for scalars, NumPy only for array operations.
- **Empty checks**: Use `if lst` / `if not lst` for lists. Avoid `len()` when not counting.
- **Loop over elements**: Prefer `for item in items` over `for i in range(len(items))`. When iterating over multiple parallel arrays, use `for a, b, c in zip(xs, ys, zs)` instead of indexing by `i`.
- **Tuple unpacking**: Use `x, y = point` instead of `point[0]`, `point[1]`. Also use in for loops: `for x, y in coords`. Minimize index usage; prefer semantic variable names.
- **NumPy structured arrays**: Use `array['field'][i]` instead of `array[i]['field']` — the latter creates an expensive void scalar per row. Avoid multi-field fancy indexing (`array[['f1','f2']]`) as it creates temporary structured arrays; use per-field loops instead.
- **Threading**: Use `threading.Lock` when updating multiple related member variables together. Single assignments are atomic in CPython; compound operations like `+=` are not.

### NumPy Style

- **Empty checks**: Use `arr.size > 0` / `arr.size == 0` for NumPy arrays.
- **Loop over structured arrays**: Use `for i in range(len(arr))` with field indexing `arr['field'][i]`, as iterating over rows creates intermediate void objects.
- **Dimension-agnostic code**: When operating on 2D slices of potentially 3D data, use `[:2]` slicing rather than checking dimensions explicitly. Let extra dimensions pass through unchanged.
- **Safe division**: Use `np.where(x == 0, 1, x)` to guard against division by zero. Do not use `np.clip` for this — it silently changes valid small/negative values.
- **Use `np.asarray()`**: Prefer `np.asarray(x)` over `np.array(x, copy=False)` — they are equivalent but `np.asarray` is the idiomatic no-copy form. Use `np.array()` only when you need a copy or when creating a new array from literal data.

### Shapely Style

- **Use creation functions, not constructors**: Use `shapely.linestrings(coords)`, `shapely.polygons(coords)` instead of `shapely.LineString(coords)`, `shapely.Polygon(coords)`. The creation functions are faster as they avoid Python-level object initialization overhead. `shapely.Point()` is fine — the speedup from `shapely.points()` is negligible for single points.
- **Use accessor functions for array operations**: Use `shapely.get_coordinates(geom)` instead of `np.array(geom.coords)` when you need a NumPy array for computation. Use `shapely.get_num_points(geom)` instead of `len(geom.coords)`. Note: `shapely.get_coordinates()` returns 2D by default; use `include_z=True` when z values are needed. For looping (`for x, y, z in geom.coords`) and single element access (`geom.coords[i]`), `.coords` is fine.

### ROS Patterns

- **Node structure**: Class-based. Constructor order: parameters → local variables → publishers → subscribers → timers. Main block: init node, instantiate class, call `run()` method containing `rospy.spin()`.
- **Message construction**: Use keyword arguments when constructing ROS messages (e.g., `Point(x=1.0, y=2.0, z=0.0)`), not positional arguments. Positional arguments don't work with ROS2 messages, so keyword arguments keep the code portable.
- **Publishers/subscribers**: Use `queue_size=1` and `tcp_nodelay=True` to minimize latency.
- **Fixed-rate publishing**: Use `rospy.Timer()` instead of `rospy.Rate()` loops. This fits the class-based structure and allows combining timers with subscribers.
- **Launch files**: All nodes must have `required="true"` so failures are immediately visible.
- **Logging**: Use `rospy.log*()` in nodes, `warnings` module in library code (src/autoware_mini/). Use `rospy.logwarn_throttle()` / `rospy.logerr_throttle()` for repeated warnings in callbacks.
- **Initialization message**: End `__init__` with `rospy.loginfo("%s - initialized", rospy.get_name())`.
- **Exception handling**: Wrap `message_filters` and `rospy.Timer` callbacks in try/except. Not needed for `rospy.Subscriber` callbacks (ROS catches exceptions internally).
- **Transform exceptions**: Catch both `tf2_ros.TransformException` and `rospy.ROSTimeMovedBackwardsException` (latter occurs during bag replay when time jumps backward).
- **Parameter namespaces**: `~param` for node-local, `module/param` for module-local (no leading `/` or `~`), `/module/param` for global (e.g., `/vehicle/wheel_base`).
- **Parameter member variables**: Name `self.<param_name>` to mirror the YAML parameter name (e.g., `self.parallel_checkers = rospy.get_param("~parallel_checkers")`).
- **Bag playback remapping**: Add remappings to `start_bag.launch` for new topics to prevent duplicate publishing when replaying bags.

### Naming

- **velocity** refers to vectors, **speed** refers to scalars.
