#!/bin/bash

# wait Carla to start
sleep 10

# get command line arguments
ARGS=$*
# strip everything after end_of_args
ARGS=${ARGS%%end_of_args*}
# launch the script
$CARLA_ROOT/PythonAPI/examples/generate_traffic.py $ARGS