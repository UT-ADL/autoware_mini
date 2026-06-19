#!/bin/bash
# Simulation validation data collection script
# Runs start_sim.launch, sets initial pose and goal, records topic data to CSV files

set -e

TIMEOUT=${1:-200}  # Default 200 seconds
OUTPUT_DIR="/tmp/validate_sim_$(date +%Y%m%d_%H%M%S)"

# Enable RViz unless running on neuron
if [[ "$(hostname)" == "neuron" ]]; then
    LAUNCH_RVIZ=false
else
    LAUNCH_RVIZ=true
fi

mkdir -p "$OUTPUT_DIR"

echo "Output directory: $OUTPUT_DIR"
echo "Timeout: ${TIMEOUT}s"
echo "RViz: $LAUNCH_RVIZ"

# Kill any existing simulation
echo "Killing any existing ROS processes..."
pkill -f roslaunch 2>/dev/null || true
pkill -f roscore 2>/dev/null || true
sleep 2

# Launch simulation, capture log
echo "Launching simulation..."
roslaunch autoware_mini start_sim.launch launch_rviz:=$LAUNCH_RVIZ \
    > "$OUTPUT_DIR/roslaunch.log" 2>&1 &
ROSLAUNCH_PID=$!

# Wait for startup
echo "Waiting for startup (8s)..."
sleep 8

# Publish initial pose (tartu_demo)
echo "Publishing initial pose..."
rostopic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped "header:
  frame_id: 'map'
pose:
  pose:
    position: {x: 0.15, y: 2.22, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: -0.775, w: 0.632}
  covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.07]" &

# Publish goal
echo "Publishing goal..."
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "header:
  frame_id: 'map'
pose:
  position: {x: 2.35, y: 14.45, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: -0.795, w: 0.607}" &

# Start recording topics to CSV (rostopic echo -p outputs CSV format)
echo "Recording topics to CSV..."
rostopic echo -p /vehicle/vehicle_status > "$OUTPUT_DIR/vehicle_status.csv" &
VEHICLE_PID=$!

rostopic echo -p /localization/current_pose > "$OUTPUT_DIR/current_pose.csv" &
POSE_PID=$!

# Wait for timeout
echo "Running for ${TIMEOUT}s..."
sleep "$TIMEOUT"

# Stop recording
echo "Stopping recording..."
kill $VEHICLE_PID $POSE_PID 2>/dev/null || true

# Stop simulation
echo "Stopping simulation..."
kill -INT $ROSLAUNCH_PID 2>/dev/null || true
wait $ROSLAUNCH_PID 2>/dev/null || true

echo ""
echo "=== Validation data collection complete ==="
echo "Output directory: $OUTPUT_DIR"
echo ""
echo "Files:"
ls -la "$OUTPUT_DIR"
echo ""
echo "To analyze results, read:"
echo "  - $OUTPUT_DIR/roslaunch.log"
echo "  - $OUTPUT_DIR/vehicle_status.csv"
echo "  - $OUTPUT_DIR/current_pose.csv"
