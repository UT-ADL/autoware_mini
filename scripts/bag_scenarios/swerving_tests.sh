#!/bin/bash

script_dir="$(dirname "$0")"

$script_dir/evaluate_scenarios.sh 'swerving' 'tartu_large' $@
