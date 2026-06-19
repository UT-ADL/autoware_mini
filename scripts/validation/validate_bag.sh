#!/bin/bash
# Bag playback validation data collection script
# Runs start_bag.launch with benchmarking enabled, records topic data to CSV files
# The launch file exits automatically when the bag finishes playing

set -e

OUTPUT_DIR="/tmp/validate_bag_$(date +%Y%m%d_%H%M%S)"

# Enable RViz unless running on neuron
if [[ "$(hostname)" == "neuron" ]]; then
    LAUNCH_RVIZ=false
else
    LAUNCH_RVIZ=true
fi

mkdir -p "$OUTPUT_DIR"

echo "Output directory: $OUTPUT_DIR"
echo "RViz: $LAUNCH_RVIZ"

# Kill any existing ROS processes
echo "Killing any existing ROS processes..."
pkill -f roslaunch 2>/dev/null || true
pkill -f roscore 2>/dev/null || true
sleep 2

# Launch bag playback with benchmarking, capture log
echo "Launching bag playback with benchmarking..."
roslaunch autoware_mini start_bag.launch \
    benchmark_topic:=/detection/predicted_objects \
    launch_rviz:=$LAUNCH_RVIZ \
    > "$OUTPUT_DIR/roslaunch.log" 2>&1 &
ROSLAUNCH_PID=$!

# Wait for startup (bag has 6s delay built-in)
echo "Waiting for startup (10s)..."
sleep 10

# Start recording topics to CSV (rostopic echo -p outputs CSV format)
echo "Recording topics to CSV..."
rostopic echo -p /vehicle/vehicle_status > "$OUTPUT_DIR/vehicle_status.csv" &
VEHICLE_PID=$!

rostopic echo -p /localization/current_pose > "$OUTPUT_DIR/current_pose.csv" &
POSE_PID=$!

# Wait for bag playback to finish (roslaunch exits automatically when bag ends)
echo "Waiting for bag playback to complete..."
wait $ROSLAUNCH_PID 2>/dev/null || true

# Stop recording
echo "Stopping recording..."
kill $VEHICLE_PID $POSE_PID 2>/dev/null || true

echo ""
echo "=== Validation data collection complete ==="
echo "Output directory: $OUTPUT_DIR"
echo ""
echo "Files:"
ls -la "$OUTPUT_DIR"
echo ""
echo "To analyze results, read:"
echo "  - $OUTPUT_DIR/roslaunch.log (contains benchmark delay output)"
echo "  - $OUTPUT_DIR/vehicle_status.csv"
echo "  - $OUTPUT_DIR/current_pose.csv"
