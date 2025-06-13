#!/bin/bash

# wait Carla to start
sleep 15

# launch the script
$CARLA_ROOT/PythonAPI/examples/generate_traffic.py --async
