#!/bin/bash

usage() {
    echo "Usage: $0 <path_to_csv_file> <path_to_bag_file_source_directory> [--copy_from_source <src_dir> [--no_cleanup]]"
    echo " "
    echo "Example: $0 ~/autoware_mini_ws/src/autoware_mini/data/bag_scenarios/tartu_large/crosswalks_bags.csv /data/bag_cache"
    echo "This script will rerecord the bag with new detection (cluster and sfa) and then convert it to a scenario."
    echo "  - bags must be present in the provided folder"
    echo "  - final scenarios will be saved in the same folder as the csv file"
    echo "  - if --copy_from_source is provided, files will be copied from <src_dir> to <path_to_bag_file_source_directory>"
    echo "      - if --no_cleanup is provided, copied files will not be deleted from the local machine"
    echo " "
    exit 1
}

# Ensure script has at least two arguments
if [ "$#" -lt 2 ]; then
    usage
fi

CSV_FILE=""
BAG_DIR=""
SRC_DIR=""
NO_CLEANUP=false
COPIED_FILES=()

# Parse arguments
while [[ "$#" -gt 0 ]]; do
    case "$1" in
        --copy_from_source)
            if [ -z "$2" ]; then
                usage
            fi
            SRC_DIR=$2
            shift 2
            ;;
        --no_cleanup)
            NO_CLEANUP=true
            shift
            ;;
        -*)
            usage
            ;;
        *)
            if [ -z "$CSV_FILE" ]; then
                CSV_FILE=$1
            elif [ -z "$BAG_DIR" ]; then
                BAG_DIR=$1
            else
                usage
            fi
            shift
            ;;
    esac
done

# Ensure required arguments are set
if [ -z "$CSV_FILE" ] || [ -z "$BAG_DIR" ]; then
    usage
fi

END_TO_GOAL_TIME=10
DETECTORS=("lidar_cluster") # lidar_sfa,radar,lidar_cluster_radar_fusion,lidar_sfa_radar_fusion

# Derive parameters from the CSV filename (remove the path and extension)
SCENARIO_TYPE=$(basename "$CSV_FILE" .csv)
SCENARIO_DIR=$(dirname "$CSV_FILE")
SCENARIO_MAP=$(basename $SCENARIO_DIR)

# Location of the autoware_mini package
AUTOWARE_MINI_DIR="$(rospack find autoware_mini)"

# Function to read parameters from the CSV file and call the processing function
read_params() {
    # Open the CSV file on a different file descriptor (e.g., 3) to avoid stdin issues
    exec 3< "$CSV_FILE"

    # Loop through each line in the CSV file
    while IFS=", " read -r BAG_FILE START DURATION SCENARIO_NUMBER REGENERATE<&3; do

        # Skip lines that are empty or start with a '#'
        [[ -z "$BAG_FILE" || "$BAG_FILE" == \#* ]] && continue

        echo "Processing BAG_FILE: $BAG_DIR/$BAG_FILE with map: $SCENARIO_MAP, START: $START, DURATION: $DURATION, SCENARIO_NUMBER: $SCENARIO_NUMBER"

        # Loop through the detector values
        for DETECTOR in $DETECTORS; do

            # Copy files from SRC_DIR to BAG_DIR if SRC_DIR is specified
            if [[ -n "$SRC_DIR" ]]; then
                echo "Copying $BAG_FILE from $SRC_DIR to $BAG_DIR..."
                rsync -ah --no-perms --progress "$SRC_DIR/$BAG_FILE" "$BAG_DIR/"
                if [ $? -ne 0 ]; then
                    echo "Error: Failed to copy $BAG_FILE from $SRC_DIR to $BAG_DIR"
                    exit 1
                fi
                COPIED_FILES+=("$BAG_DIR/$BAG_FILE")
                echo "File $BAG_FILE copied successfully."
            fi            

            # put together output filename
            OUTPUT_FILE="${SCENARIO_TYPE%_bags}_${SCENARIO_NUMBER}.bag"
            
            # Call the processing function with the parameters
            process_bag "$BAG_DIR" "$BAG_FILE" "$START" "$DURATION" "$END_TO_GOAL_TIME" "$SCENARIO_NUMBER" "$DETECTOR" "$OUTPUT_FILE" "$REGENERATE" &
            wait
        done

    done

    # Close the file descriptor
    exec 3<&-
}

process_bag() {
    local BAG_DIR="$1"
    local BAG_FILE="$2"
    local START="$3"
    local DURATION="$4"
    local END_TO_GOAL_TIME="$5"
    local SCENARIO_NUMBER="$6"
    local DETECTOR="$7"
    local OUTPUT_FILE="$8"
    local REGENERATE="$9"

    # Step 1: Launch the ROS bag with specified parameters in the background
    echo "  Rerecord bag using ${DETECTOR} detector"

    if [ "$REGENERATE" == "true" ]; then
        CMD="roslaunch autoware_mini start_bag.launch \
            bag_file:=${BAG_FILE} \
            bag_folder:=${BAG_DIR} \
            launch_rviz:=\"false\" \
            map_name:=$SCENARIO_MAP \
            detector:=${DETECTOR} \
            start:=${START} \
            record_bag:=${OUTPUT_FILE}"
        
        # Add duration parameter only if DURATION is set
        [ -n "$DURATION" ] && CMD+=" duration:=$((DURATION + END_TO_GOAL_TIME))"
        
        # Execute the command
        eval $CMD #> /dev/null 2>&1 < /dev/null
        wait
    fi

    ARGS=""
    if [ -n "$DURATION" ]; then
        ARGS+=" --end_time $DURATION"
    fi

    if [ -n "$END_TO_GOAL_TIME" ]; then
        ARGS+=" --end_to_goal_time $END_TO_GOAL_TIME"
    fi

    # Step 2: Run the scenario creation script
    echo "  Creating scenario ${OUTPUT_FILE}"
    $AUTOWARE_MINI_DIR/scripts/bag_scenarios/create_scenario_bag.py $BAG_DIR/${OUTPUT_FILE} ${SCENARIO_DIR}/${OUTPUT_FILE} \
    $ARGS < /dev/null

    # Step 3: Remove the rerecorded bag file after processing
    rm $BAG_DIR/${OUTPUT_FILE}

    # Step 4: Remove the copied files if --no_cleanup is not set
    if [ "$NO_CLEANUP" = false ]; then
        for FILE in "${COPIED_FILES[@]}"; do
            echo "Cleaning up copied file: $FILE"
            rm "$FILE"
        done
        COPIED_FILES=()
    fi
}

# Start reading parameters and processing bags
read_params

echo "All commands executed successfully."