#!/bin/bash

# Remote-assistance launcher for the operator's computer (e.g. Clevon PC):
# detects ROS_IP, opens a browser to the cameras (the camera app runs on the
# NVIDIA Drive at startup), starts RViz, the StreamDeck button controller, and
# the wheel force-feedback daemon. Ctrl+C shuts everything down.
#
# Usage: ./remote_assistance.sh [ROS_MASTER_IP] [NVIDIA_IP] [CAMERA_PORT]
#
# With the real car (default master IP 192.168.100.100):
#   ./remote_assistance.sh
#
# With simulation on a separate PC (provide host IP as argument):
#   ./remote_assistance.sh 192.168.1.50
#   Note: ROS_IP must also be set manually on the host PC.

set -e

MASTER_IP="${1:-192.168.100.100}"
NVIDIA_IP="${2:-192.168.100.200}"
CAMERA_PORT="${3:-8080}"

# Directory of this script.
SCRIPT_DIR="$(dirname "${BASH_SOURCE[0]}")"

# Set ROS environment
export ROS_MASTER_URI="http://${MASTER_IP}:11311/"
export ROS_IP=$(ip route get "$MASTER_IP" | sed -n '/src/{s/.*src *\([^ ]*\).*/\1/p;q}')

if [ -z "$ROS_IP" ]; then
    echo "Error: Could not determine ROS_IP for master at $MASTER_IP" >&2
    exit 1
fi

echo "ROS_MASTER_URI=$ROS_MASTER_URI"
echo "ROS_IP=$ROS_IP"

# PIDs of everything we launch in the background, so cleanup can kill them on exit.
BG_PIDS=()

# Open browser to view cameras
CAMERA_URL="http://${NVIDIA_IP}:${CAMERA_PORT}"
echo "Opening camera viewer at $CAMERA_URL"
firefox "$CAMERA_URL" &
BG_PIDS+=($!)

# StreamDeck button controller. Fails loudly if its deps aren't installed —
# run ./setup.sh first.
python3 "$SCRIPT_DIR/button_controller/button_controller.py" &
BG_PIDS+=($!)
echo "Started button controller (PID $!)"

# Serial peripheral bridge: forwards engage button + blinker lever to the
# browser teleop UI over ws://127.0.0.1:8765 (Firefox has no Web Serial).
python3 "$SCRIPT_DIR/control_station_blinkers_button/blinkers_button_teleop.py" &
BG_PIDS+=($!)
echo "Started blinkers/button teleop bridge (PID $!)"

# Wheel force-feedback daemon. With no FFB-capable wheel it stays up and reports
# no_device — the browser falls back to read-only steering, and a wheel plugged
# in later is picked up automatically (no restart needed).
python3 "$SCRIPT_DIR/wheel_ffb/wheel_ffb.py" &
BG_PIDS+=($!)
echo "Started wheel FFB daemon (PID $!)"

# Ensure all child processes are killed on exit
cleanup() {
    for pid in "${BG_PIDS[@]}"; do kill "$pid" 2>/dev/null || true; done
    wait 2>/dev/null
}
trap cleanup EXIT INT TERM

# Launch RViz
roslaunch autoware_mini rviz.launch node_name:=remote_rviz
