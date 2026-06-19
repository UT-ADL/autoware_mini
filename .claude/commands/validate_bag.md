# Bag Playback Validation Command

Validate the autoware_mini bag playback by running the data collection script and analyzing the results.

## Instructions

1. **Run the validation script**:
   ```bash
   bash $(rospack find autoware_mini)/scripts/validation/validate_bag.sh
   ```
   The script outputs files to `/tmp/validate_bag_<timestamp>/`.
   It waits for the bag playback to complete automatically (no timeout needed).
   RViz is enabled by default (disabled on neuron).

2. **Read and analyze the output files**:

   **roslaunch.log** - Read the full log and check for:
   - `average delay:` output from rostopic delay benchmark
   - `[ERROR]` or `[FATAL]` messages
   - Python tracebacks or exceptions
   - Node crashes or unexpected terminations
   - Significant `[WARN]` messages (startup warnings about "not received" are OK)
   - TCP connection warnings are benign and can be ignored

   **vehicle_status.csv** - Check the header row for column names, then extract:
   - Speed data (look for `field.speed` column)
   - Verify car was moving (speed > 0)

   **current_pose.csv** - Check the header row for column names, then extract:
   - Position data (look for `field.pose.position.x` and `field.pose.position.y` columns)
   - Verify position data was received

3. **Determine PASS/FAIL** based on expected behavior:
   - Average delay should be in **100-200ms** range (0.100 - 0.200 seconds)
   - Car should be moving during playback (speed > 0)
   - No runtime errors in the log

4. **Report results**:
   - Overall PASS/FAIL assessment
   - Average delay and whether it's in the expected range
   - Confirmation that car was moving
   - Any errors or significant warnings found

## Expected Behavior

- Bag playback starts after 6 second delay
- Detection pipeline delay should be 100-200ms
- Car should be moving (this is recorded sensor data, not simulation)
- No runtime errors (startup warnings are OK)

## Troubleshooting

If delay is too high (>200ms):
- Check CPU load - detection may be CPU-bound
- Check if GPU is being used for detection

If delay is too low (<100ms):
- May indicate detection is not processing properly
- Check that predicted_objects topic is being published

If car is not moving:
- Check if bag file exists and is valid
- Check for errors in localization
