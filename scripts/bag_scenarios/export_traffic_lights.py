#!/usr/bin/env python3
"""
Export traffic light stop lines from a Lanelet2 map to a GeoJSON file.
"""

import argparse
import json
import logging
from pathlib import Path
from typing import Optional, Tuple

import shapely
import yaml
import lanelet2
from lanelet2.io import Origin, load
from lanelet2.projection import UtmProjector
from autoware_mini.lanelet2 import get_traffic_light_stop_lines

DEFAULT_TIMING = {"offset": 0, "red_duration": 10, "green_duration": 8}
# Max Hausdorff distance (in meters) between a new stop line and an existing
# one for the timing to be considered a carry-over. Tight enough to reject
# different stop lines at the same intersection (typically separated by a
# lane width), loose enough to absorb small map refinements.
MATCH_TOLERANCE_M = 1.0


def setup_logger() -> None:
    """Configure logging for the script."""
    logging.basicConfig(
        format="%(levelname)s: %(message)s",
        level=logging.INFO,
    )


def load_yaml_origin(yaml_path: Path) -> Tuple[Optional[float], Optional[float]]:
    """Load UTM origin coordinates from a YAML config file."""
    if not yaml_path.exists():
        logging.warning(f"YAML file not found: {yaml_path}")
        return None, None

    try:
        with yaml_path.open("r") as f:
            cfg = yaml.safe_load(f) or {}
        return float(cfg.get("utm_origin_lat")), float(cfg.get("utm_origin_lon"))
    except Exception as e:
        logging.warning(f"Failed to read YAML {yaml_path}: {e}")
        return None, None


def load_lanelet2_map(map_path: Path, yaml_path: Path, use_custom_origin: bool):
    """Load a Lanelet2 map with projection based on YAML origin or default."""
    utm_origin_lat, utm_origin_lon = load_yaml_origin(yaml_path)
    projector = UtmProjector(Origin(utm_origin_lat, utm_origin_lon), use_custom_origin, False)
    lanelet2_map = load(str(map_path), projector)
    return lanelet2_map, projector


def load_existing_traffic_lights(geojson_path: Path, projector) -> list:
    """Load existing traffic light entries as UTM linestrings keyed by geometry."""
    if not geojson_path.exists():
        return []

    try:
        with geojson_path.open("r") as f:
            data = json.load(f)
    except Exception as e:
        logging.warning(f"Failed to load existing traffic lights from {geojson_path}: {e}")
        return []

    existing = []
    for feature in data.get("features", []):
        props = feature["properties"]
        utm_coords = []
        for lon, lat, z in feature["geometry"]["coordinates"]:
            utm_point = projector.forward(lanelet2.core.GPSPoint(lat, lon, z))
            utm_coords.append((utm_point.x, utm_point.y, utm_point.z))
        existing.append({
            "line": shapely.linestrings(utm_coords),
            "offset": props.get("offset", DEFAULT_TIMING["offset"]),
            "red_duration": props.get("red_duration", DEFAULT_TIMING["red_duration"]),
            "green_duration": props.get("green_duration", DEFAULT_TIMING["green_duration"]),
        })

    logging.info(f"Loaded {len(existing)} existing traffic light configurations")
    return existing


def find_geometry_match(new_line, existing: list, used: set) -> Optional[dict]:
    """Return the unused existing entry whose geometry best matches new_line, or None.

    Matching is geometry-only (Hausdorff distance) because the lanelet ref_line id
    of a stop line is not stable across map edits — relying on the id would lose
    user-edited timings on every re-export.
    """
    best_idx = None
    best_dist = MATCH_TOLERANCE_M
    for i, entry in enumerate(existing):
        if i in used:
            continue
        d = shapely.hausdorff_distance(new_line, entry["line"])
        if d < best_dist:
            best_dist = d
            best_idx = i
    if best_idx is None:
        return None
    # Mark as consumed so two new stop lines can't both inherit the same timing.
    used.add(best_idx)
    return existing[best_idx]


def extract_traffic_light_features(lanelet2_map, projector, existing_traffic_lights: list) -> list:
    """Extract traffic light stop lines as GeoJSON features.

    On match, only the timing fields (offset / red_duration / green_duration) are
    carried over from the existing geojson. The geometry and id are always taken
    from the new lanelet map, so small edits to the stop line in the map are
    reflected in the next export rather than being overwritten by stale coords.
    """
    features = []
    tfl_stop_lines = get_traffic_light_stop_lines(lanelet2_map)
    used = set()
    matched = 0

    for stop_line_id, stop_line in tfl_stop_lines.items():
        # Timing comes from the matched existing entry (if any); defaults otherwise.
        match = find_geometry_match(stop_line, existing_traffic_lights, used)
        timing = match if match is not None else DEFAULT_TIMING
        if match is not None:
            matched += 1

        # Geometry always comes from the new lanelet map — re-project UTM -> GPS.
        coords = []
        for (x, y, z) in stop_line.coords:
            gps_point = projector.reverse(lanelet2.core.BasicPoint3d(x, y, z))
            coords.append([gps_point.lon, gps_point.lat, z])

        features.append({
            "type": "Feature",
            "properties": {
                "id": stop_line_id,
                "offset": timing["offset"],
                "red_duration": timing["red_duration"],
                "green_duration": timing["green_duration"],
            },
            "geometry": {
                "type": "LineString",
                "coordinates": coords,
            },
        })

    defaulted = len(tfl_stop_lines) - matched
    logging.info(
        f"Matched {matched}/{len(tfl_stop_lines)} stop lines to existing timing "
        f"(Hausdorff <= {MATCH_TOLERANCE_M} m); {defaulted} use default timing."
    )
    return features


def export_geojson(features: list, output_path: Path) -> None:
    """Write features to a GeoJSON file."""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w") as f:
        json.dump({"type": "FeatureCollection", "features": features}, f, indent=2)
    logging.info(f"Exported traffic light stop lines to {output_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Export traffic light stop lines from Lanelet2 map to GeoJSON")
    parser.add_argument("map_file_path", type=Path, help="Path to the .osm map file")
    parser.add_argument("--use-custom-origin", action="store_true", help="Use custom origin coordinates (default: False)")
    args = parser.parse_args()

    setup_logger()

    yaml_file_path = args.map_file_path.with_suffix(".yaml")
    lanelet2_map, projector = load_lanelet2_map(
        args.map_file_path, yaml_file_path, args.use_custom_origin
    )

    # Load existing traffic light configurations
    map_name = args.map_file_path.stem
    output_dir = args.map_file_path.parent.parent / "bag_scenarios" / map_name / "geojson"
    output_path = output_dir / "traffic_lights.geojson"
    existing_traffic_lights = load_existing_traffic_lights(output_path, projector)

    features = extract_traffic_light_features(lanelet2_map, projector, existing_traffic_lights)

    export_geojson(features, output_path)

if __name__ == "__main__":
    main()
