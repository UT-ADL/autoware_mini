#!/bin/bash

usage() {
    echo "Usage: $0 <path_to_csv_file> <path_to_geojson_files_directory>"
    echo " "
    echo "Example: $0 ~/autoware_mini_ws/src/autoware_mini/data/bag_scenarios/tartu_large/planner_geojson.csv /data/bag_scenarios/tartu_large/geojson"
    echo "This script will convert geojson trajectory files listed in the CSV into scenario bags."
    echo "  - geojson files must be present in the provided folder"
    echo "  - final scenarios will be saved in the same bag_scenarios folder"
    echo " "
    exit 1
}

# Ensure script has at least two arguments
if [ "$#" -lt 2 ]; then
    usage
fi

CSV_FILE=$1
GEOJSON_DIR=$2


# Derive parameters from the CSV filename (remove the path and extension)
SCENARIO_TYPE=$(basename "$CSV_FILE" .csv)
SCENARIO_DIR=$(dirname "$CSV_FILE")
SCENARIO_MAP=$(basename "$SCENARIO_DIR")

# Location of the autoware_mini package
AUTOWARE_MINI_DIR="$(rospack find autoware_mini)"

# Export traffic lights for the scenario map
MAP_FILE="$AUTOWARE_MINI_DIR/data/maps/${SCENARIO_MAP}.osm"
if [ -f "$MAP_FILE" ]; then
    echo "Exporting traffic lights for map: $SCENARIO_MAP"
    $AUTOWARE_MINI_DIR/scripts/bag_scenarios/export_traffic_lights.py "$MAP_FILE"
else
    echo "Warning: Map file not found: $MAP_FILE, skipping traffic light export"
fi

# Function to read parameters from the CSV file and call the processing function
read_params() {
    # Open the CSV file on a different file descriptor (e.g., 3) to avoid stdin issues
    exec 3< "$CSV_FILE"

    # Loop through each line in the CSV file
    while IFS=',' read -r BAG_FILE START DURATION SCENARIO_NAME REGENERATE<&3; do
        # Trim whitespace from variables
        BAG_FILE=$(echo "$BAG_FILE" | xargs)
        SCENARIO_NAME=$(echo "$SCENARIO_NAME" | xargs)

        # Skip lines that are empty or start with a '#'
        [[ -z "$BAG_FILE" || "$BAG_FILE" == \#* ]] && continue

        echo "Processing: $GEOJSON_DIR/$BAG_FILE"

        # put together output filename
        OUTPUT_FILE="${SCENARIO_TYPE%_geojson}_${SCENARIO_NAME}.bag"

        # Call the processing function with the parameters (run in foreground so FD3 isn't inherited by a background job)
        process_bag "$GEOJSON_DIR" "$BAG_FILE" "$OUTPUT_FILE"

    done

    # Close the file descriptor
    exec 3<&-
}

process_bag() {
    local GEOJSON_DIR="$1"
    local BAG_FILE="$2"
    local OUTPUT_FILE="$3"


    # Step 2: Run the scenario creation script
    # echo "  Creating scenario ${OUTPUT_FILE}"
    $AUTOWARE_MINI_DIR/scripts/bag_scenarios/trajectories_to_scenario.py ${GEOJSON_DIR}/${BAG_FILE} $AUTOWARE_MINI_DIR/data/bag_scenarios/${SCENARIO_MAP}/${OUTPUT_FILE} --add_traffic_lights \
    $ARGS < /dev/null

}

# Start reading parameters and processing bags
read_params

echo "All commands executed successfully."