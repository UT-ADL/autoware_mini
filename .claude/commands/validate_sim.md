# Simulation Validation Command

Validate the autoware_mini simulation by running the data collection script and analyzing the results.

## Instructions

1. **Run the validation script**:
   ```bash
   bash $(rospack find autoware_mini)/scripts/validation/validate_sim.sh [timeout_seconds]
   ```
   Default timeout is 200 seconds. The script outputs files to `/tmp/validate_sim_<timestamp>/`.
   RViz is enabled by default (disabled on neuron).

2. **Read and analyze the output files**:

   **roslaunch.log** - Read the full log and check for:
   - `[ERROR]` or `[FATAL]` messages
   - Python tracebacks or exceptions
   - Node crashes or unexpected terminations
   - Significant `[WARN]` messages (startup warnings about "not received" are OK)
   - TCP connection warnings are benign and can be ignored

   **vehicle_status.csv** - Check the header row for column names, then extract:
   - Speed data (look for `field.speed` column)
   - Max speed and final speed

   **current_pose.csv** - Check the header row for column names, then extract:
   - Position data (look for `field.pose.position.x` and `field.pose.position.y` columns)
   - Final position

3. **Determine PASS/FAIL** based on expected behavior:
   - Car should reach ~40 km/h max speed (35-50 km/h acceptable)
   - Car should stop (speed < 1 km/h) after completing the route (~180-190 seconds)
   - Final position should be within 10m of goal (x=2.35, y=14.45)
   - No runtime errors in the log

4. **Report results**:
   - Overall PASS/FAIL assessment
   - Key metrics: max speed, final speed, final position, distance to goal
   - Any errors or significant warnings found

## Troubleshooting

If the car doesn't move:
- Check if global path was generated in the log
- Check for planner errors in roslaunch.log
- Initial pose may not be on the road
