#!/bin/bash

host=$1                    # Carla host
port=$2                    # Carla port

# wait for Carla ROS bridge to start
sleep 5

# launch the script
exec $CARLA_ROOT/PythonAPI/examples/generate_traffic.py --host $host --port $port --asynch
