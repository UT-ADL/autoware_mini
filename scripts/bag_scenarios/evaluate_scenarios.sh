#!/bin/bash

scenario_list=$1
map_name=$2
rate=1
launch_rviz=true

# parse optional arguments
while [[ "$#" -gt 2 ]]; do
    case $3 in
        --rate)
            rate="$4"
            shift 2
            ;;
        --no_rviz)
            launch_rviz=false
            shift
            ;;
        *)
            echo "Usage: $0 <scenario_list> <map_name> [--rate <value>] [--no_rviz]"
            exit 1
            ;;
    esac
done

# get the root directory of Autoware Mini
aw_mini_root="$(rospack find autoware_mini)"

# path of the CSV file with list of scenarios
source_csv_file="$aw_mini_root/data/bag_scenarios/$map_name/${scenario_list}_tests.csv"
results_csv_file="$aw_mini_root/data/bag_scenarios/$map_name/results/${scenario_list}_results.csv"

# make results CSV file empty
> $results_csv_file

# launch the stack once in background
roslaunch autoware_mini start_sim.launch scenario_runner:=true map_name:=$map_name launch_rviz:=$launch_rviz &
LAUNCH_PID=$!
trap 'kill -INT $LAUNCH_PID 2>/dev/null; wait $LAUNCH_PID 2>/dev/null; exit 1' INT TERM

# wait for the stack to finish initialization
sleep 10

# read the scenarios from the source csv file and run them
IFS=$'\n'
for line in $(tail -n +2 "$source_csv_file"); do
    # check if the stack is still running
    if ! kill -0 $LAUNCH_PID 2>/dev/null; then
        echo "ERROR: Stack process died unexpectedly" >&2
        exit 1
    fi

    # split line into variables using IFS=','
    IFS=',' read -r scenario_name ade_threshold fde_threshold collision_threshold max_deceleration_threshold <<< "$line"

    results_plot_file="$aw_mini_root/data/bag_scenarios/$map_name/results/$scenario_name.png"

    # wait for metrics calculator to be ready (handles respawn from previous iteration)
    until rosnode ping -c 1 /vehicle/scenario_metrics_calculator 2>/dev/null; do sleep 0.1; done

    # set scenario parameters via rosparam (separate calls to avoid clearing other params in the namespace)
    # must be after rosnode ping to avoid roslaunch <param> defaults overwriting these values
    rosparam set /vehicle/scenario_metrics_calculator/scenario_name "$scenario_name"
    rosparam set /vehicle/scenario_metrics_calculator/ade_csv_file "$results_csv_file"
    rosparam set /vehicle/scenario_metrics_calculator/ade_plot_file "$results_plot_file"
    rosparam set /vehicle/scenario_metrics_calculator/ade_threshold "$ade_threshold"
    rosparam set /vehicle/scenario_metrics_calculator/fde_threshold "$fde_threshold"
    rosparam set /vehicle/scenario_metrics_calculator/collision_threshold "$collision_threshold"
    rosparam set /vehicle/scenario_metrics_calculator/max_deceleration_threshold "$max_deceleration_threshold"

    # cancel previous route before starting new scenario
    # first call requests stopping, second call force-clears if vehicle was still moving
    rosservice call /planning/cancel_route
    rosservice call /planning/cancel_route

    # play the scenario bag (exits when bag ends)
    rosbag play $aw_mini_root/data/bag_scenarios/$map_name/$scenario_name.bag --clock --wait-for-subscribers --rate=$rate __name:=player

    # save metrics by killing the node (triggers shutdown_hook)
    rosnode kill /vehicle/scenario_metrics_calculator
done

# shut down the stack
kill -INT $LAUNCH_PID
wait $LAUNCH_PID 2>/dev/null

# calculate the average ADE over all scenarios
average_ade=$(LC_NUMERIC="C" awk -F',' 'NR>1 {sum+=$2; count++} END {if (count > 0) printf "%.2f\n", sum/count}' "$results_csv_file")
average_fde=$(LC_NUMERIC="C" awk -F',' 'NR>1 {sum+=$4; count++} END {if (count > 0) printf "%.2f\n", sum/count}' "$results_csv_file")
average_collision_score=$(LC_NUMERIC="C" awk -F',' 'NR>1 {sum+=$6; count++} END {if (count > 0) printf "%.2f\n", sum/count}' "$results_csv_file")
average_max_deceleration=$(LC_NUMERIC="C" awk -F',' 'NR>1 {sum+=$8; count++} END {if (count > 0) printf "%.2f\n", sum/count}' "$results_csv_file")

# calculate success rates
ade_success_rate=$(LC_NUMERIC="C" awk -F',' 'NR>1 {total++; if ($3=="SUCCESS") success++} END {if (total > 0) printf "%.2f\n", success/total}' "$results_csv_file")
fde_success_rate=$(LC_NUMERIC="C" awk -F',' 'NR>1 {total++; if ($5=="SUCCESS") success++} END {if (total > 0) printf "%.2f\n", success/total}' "$results_csv_file")
collision_success_rate=$(LC_NUMERIC="C" awk -F',' 'NR>1 {total++; if ($7=="SUCCESS") success++} END {if (total > 0) printf "%.2f\n", success/total}' "$results_csv_file")
max_deceleration_rate=$(LC_NUMERIC="C" awk -F',' 'NR>1 {total++; if ($9~/^SUCCESS/) success++} END {if (total > 0) printf "%.2f\n", success/total}' "$results_csv_file")

# append the results csv with the average scores and success rates
echo "AVERAGE,${average_ade},${ade_success_rate},${average_fde},${fde_success_rate},${average_collision_score},${collision_success_rate},${average_max_deceleration},${max_deceleration_rate}" >> "$results_csv_file"
echo -e "\e[36mAverage ADE: $average_ade\nADE success rate: $ade_success_rate\n\
Average FDE: $average_fde\nFDE success rate: $fde_success_rate\n\
Average collision score: $average_collision_score\nCollision success rate: $collision_success_rate\n\
Average max deceleration: $average_max_deceleration\nMax deceleration success rate: $max_deceleration_rate\e[0m\n"
