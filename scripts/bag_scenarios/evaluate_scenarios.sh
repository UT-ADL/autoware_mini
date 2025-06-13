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

# read the scenarios from the source csv file and run them
IFS=$'\n'
for line in $(tail -n +2 "$source_csv_file"); do
    # split line into variables using IFS=','
    IFS=',' read -r scenario_name ade_threshold speed_smoothness_threshold <<< "$line"
    
    roslaunch autoware_mini start_sim.launch map_name:=$map_name scenario_name:=$scenario_name ade_csv_file:=$results_csv_file \
            ade_plot_file:=$aw_mini_root/data/bag_scenarios/$map_name/results/$scenario_name.png \
            ade_threshold:=$ade_threshold speed_smoothness_threshold:=$speed_smoothness_threshold \
            rate:=$rate launch_rviz:=$launch_rviz
done

# calculate the avergare ADE over all scenarios
average_ade=$(LC_NUMERIC="C" awk -F',' 'NR>1 {sum+=$2; count++} END {if (count > 0) printf "%.2f\n", sum/count}' "$results_csv_file")
average_smoothness=$(LC_NUMERIC="C" awk -F',' 'NR>1 {sum+=$4; count++} END {if (count > 0) printf "%.2f\n", sum/count}' "$results_csv_file")

# calculate success rates
ade_success_rate=$(LC_NUMERIC="C" awk -F',' 'NR>1 {total++; if ($3=="SUCCESS") success++} END {if (total > 0) printf "%.2f\n", success/total}' "$results_csv_file")
smoothness_success_rate=$(LC_NUMERIC="C" awk -F',' 'NR>1 {total++; if ($5~/^SUCCESS/) success++} END {if (total > 0) printf "%.2f\n", success/total}' "$results_csv_file")

# append the results csv with the average scores and success rates
echo "AVERAGE,${average_ade},${ade_success_rate},${average_smoothness},${smoothness_success_rate}" >> "$results_csv_file"
echo -e "\e[36mAverage ADE: $average_ade\nADE success rate: $ade_success_rate\nAverage speed smoothness: $average_smoothness\nSpeed smoothness success rate: $smoothness_success_rate\e[0m\n"