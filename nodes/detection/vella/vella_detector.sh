#!/bin/bash

vella_run object_list --lidar-model vlp32 --sensor-height "$1" --offline --system-extrinsics-path "$2" --odometry-topic "$3"  --ignore-radius "$4" --no-gui

