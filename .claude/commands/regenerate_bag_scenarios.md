---
allowed-tools: Bash, Read, Edit, Glob, Grep
description: Regenerate bag scenarios from source recordings
---

# Regenerate Bag Scenarios

Regenerate scenario bags from source recordings. Needed after perception-pipeline changes or when source bags have been updated.

**Arguments**: `$ARGUMENTS` — optional test set (`yielding`, `crosswalks`, `tartu_demo`, `lexus`, `swerving`, `give_way_bus`, `planner`). If omitted, regenerates ALL.

## Gotchas (read before running)

- **Neuron only.** Regeneration needs `/data/bag_cache/` (source bags) and benefits from the GPU. The laptop has neither — run this on neuron.
- **Never parallel.** Each `create_scenarios.sh` / `create_geojson_scenarios.sh` call starts its own `roslaunch` with the perception pipeline. Two concurrent launches on port 11311 crash on roscore conflict; even with separate `ROS_MASTER_URI` they fight over CPU/GPU (SFA detector, fusion, EMA tracker) and the generated detections become timing-dependent. No backgrounding, no `&&`-chains, no second terminal — one suite, wait, next.
- **Source the ROS env in every Bash call.** Same reason as in `/run_bag_scenarios`: without `source $WORKSPACE/devel/setup.bash`, `rospack find` returns empty and the scripts silently write to bogus paths while still exiting 0. Verify the expected `.bag` files exist and are non-empty.
- **Two commits, bags first:** regeneration produces new `.bag` files *and*, via `/run_bag_scenarios` afterwards, updated result CSVs/PNGs. Commit the `.bag` files first ("Regenerate bag scenarios"), then the results ("Update bag scenario test results"). This lets reviewers bisect whether a metric change comes from the bag data or from the branch code.

## Process

### 1. Check environment

```bash
source $(catkin locate)/devel/setup.bash
rospack find autoware_mini
ls /data/bag_cache/         # source bags must be here
cd $(rospack find autoware_mini)/scripts/bag_scenarios
```

### 2. Build TensorRT engines

Build (or refresh) the TensorRT engines so the traffic-light detector and other ONNX models are ready for inference — otherwise the first scenario stalls for minutes building engines on the fly.

```bash
scripts/tensorrt/build_engines.py
```

Engines are cached under `~/.ros` and only need rebuilding when the ONNX model, GPU, or TensorRT version changes; re-running is a no-op when cached.

### 3. Regenerate

Append `--no_rviz`-equivalent launch args only if the underlying launch accepts them; these scripts don't expose the flag. RViz on the laptop (debugging), off on neuron (no display) — neuron is the primary host here anyway.

**Single test set** (`$ARGUMENTS` given):
```bash
./create_scenarios.sh ../../data/bag_scenarios/<map>/<testset>_bags.csv /data/bag_cache
```
Map is `tartu_large` for `crosswalks`, `yielding`, `lexus`, `swerving`, `give_way_bus`; `tartu_demo` for `tartu_demo`.

Planner uses GeoJSON:
```bash
./create_geojson_scenarios.sh ../../data/bag_scenarios/tartu_large/planner_geojson.csv ../../data/bag_scenarios/tartu_large/geojson
```

**All** (sequentially; smallest first):
```bash
./create_scenarios.sh ../../data/bag_scenarios/tartu_demo/tartu_demo_bags.csv      /data/bag_cache
./create_scenarios.sh ../../data/bag_scenarios/tartu_large/give_way_bus_bags.csv   /data/bag_cache
./create_scenarios.sh ../../data/bag_scenarios/tartu_large/lexus_bags.csv          /data/bag_cache
./create_scenarios.sh ../../data/bag_scenarios/tartu_large/swerving_bags.csv       /data/bag_cache
./create_scenarios.sh ../../data/bag_scenarios/tartu_large/yielding_bags.csv       /data/bag_cache
./create_scenarios.sh ../../data/bag_scenarios/tartu_large/crosswalks_bags.csv     /data/bag_cache
./create_geojson_scenarios.sh ../../data/bag_scenarios/tartu_large/planner_geojson.csv ../../data/bag_scenarios/tartu_large/geojson
```

### 4. Verify

Spot-check a regenerated bag for expected topics and message counts:
```bash
rosbag info <bag_file>
```

### 5. Commit the new bags

Stage only the regenerated `.bag` files (and any touched `*_bags.csv` / `*_tests.csv`) and commit as e.g. "Regenerate bag scenarios". Do not include the `results/` folder in this commit.

### 6. Update test results

Run `/run_bag_scenarios` to regenerate the result CSVs/PNGs on the new bags and produce the second commit following that command's process.

## Troubleshooting

- **Missing source bags**: check the required names in the `*_bags.csv` file against `/data/bag_cache/`.
- **Detection failing**: verify `nvidia-smi`, check env is sourced, scan console output for errors.
- **Disk space**: `df -h /data/bag_cache` — intermediate bags can be large.
