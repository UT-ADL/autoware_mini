#!/bin/bash

map_name=$1
scenario_name=$2
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
            echo "Usage: $0 <map_name> <scenario_name> [--rate <value>] [--no_rviz]"
            exit 1
            ;;
    esac
done

# get the root directory of Autoware Mini
aw_mini_root="$(rospack find autoware_mini)"

# find the tests CSV file that contains this scenario
source_csv_file=$(grep -l "^$scenario_name," "$aw_mini_root/data/bag_scenarios/$map_name/"*_tests.csv)
scenario_list=$(basename "$source_csv_file" _tests.csv)
results_csv_file="$aw_mini_root/data/bag_scenarios/$map_name/results/${scenario_list}_results.csv"
tmp_csv_file="$aw_mini_root/data/bag_scenarios/$map_name/results/${scenario_name}_results_tmp.csv"

IFS=',' read -r name ade_threshold fde_threshold collision_threshold max_deceleration_threshold <<< "$(awk -F',' -v key="$scenario_name" '$1== key { print $0 }' "$source_csv_file")"

roslaunch autoware_mini start_sim.launch map_name:=$map_name scenario_name:=$scenario_name ade_csv_file:=$tmp_csv_file \
ade_plot_file:=$aw_mini_root/data/bag_scenarios/$map_name/results/$scenario_name.png ade_threshold:=$ade_threshold fde_threshold:=$fde_threshold \
collision_threshold:=$collision_threshold max_deceleration_threshold:=$max_deceleration_threshold rate:=$rate launch_rviz:=$launch_rviz

# remove windows style line ending
line=$(awk -F',' -v key="$scenario_name" '$1==key {print $0}' "$tmp_csv_file" | tr -d '\r\n')

IFS=',' read -r \
    name ade_score ade_rating fde_score fde_rating \
    collision_score collision_rating max_deceleration max_deceleration_rating \
    <<< "$line"

LC_NUMERIC="C" awk -F',' \
    -v key="$scenario_name" \
    -v new_ade_score="$ade_score" \
    -v new_ade_rating="$ade_rating" \
    -v new_fde_score="$fde_score" \
    -v new_fde_rating="$fde_rating" \
    -v new_col_score="$collision_score" \
    -v new_col_rating="$collision_rating" \
    -v new_dec_score="$max_deceleration" \
    -v new_dec_rating="$max_deceleration_rating" \
'
BEGIN {
    OFS=","
}

# Skip the AVERAGE row
$1 == "AVERAGE" { next }

NR == 1 {
    # print header immediately
    print $0
    next
}

{
    sub(/\r$/, "")   # remove trailing \r
    sub(/\n$/, "")   # remove trailing \n

    # ------------------------------------------
    # MODIFY VALUES
    # ------------------------------------------

    if ($1 == key) {
        $2 = new_ade_score
        $3 = new_ade_rating
        $4 = new_fde_score
        $5 = new_fde_rating
        $6 = new_col_score
        $7 = new_col_rating
        $8 = new_dec_score
        $9 = new_dec_rating
    }

    print $0

    # ------------------------------------------
    # ACCUMULATE NUMERIC COLUMNS FOR AVERAGES
    # ------------------------------------------

    ade_sum += $2
    fde_sum += $4
    col_sum += $6
    dec_sum += $8

    # convert ratings to numeric (SUCCESS=1, FAILURE=0)
    ade_rate_sum += ($3 == "SUCCESS")
    fde_rate_sum += ($5 == "SUCCESS")
    col_rate_sum += ($7 == "SUCCESS")
    dec_rate_sum += ($9 == "SUCCESS")

    count++
}

END {
    # compute averages
    printf "AVERAGE,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f", \
        ade_sum/count, ade_rate_sum/count, fde_sum/count, fde_rate_sum/count, \
        col_sum/count, col_rate_sum/count, dec_sum/count, dec_rate_sum/count
}
' "$results_csv_file" > "$results_csv_file.tmp" && mv "$results_csv_file.tmp" "$results_csv_file"

rm "$tmp_csv_file"
