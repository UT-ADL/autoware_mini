#!/bin/bash
ARGS=$*
$CARLA_ROOT/PythonAPI/examples/generate_traffic.py ${ARGS%%end_of_args*}