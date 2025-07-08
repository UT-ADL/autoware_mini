#!/bin/bash

name=$1
script_dir=$(realpath "$(dirname "$0")")
blacklist_file=$(dirname "$(dirname "$script_dir")")/config/record_bag/blacklist.txt

cd /media/$USER/ExtremePro/$USER

rosbag record -a -O $(date +"%Y-%m-%d-%H-%M-%S")_$name -x "$(grep -v -P '^#(.*)' $blacklist_file | xargs | sed -e 's/ /|/g')"
