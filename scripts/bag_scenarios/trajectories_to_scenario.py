#!/usr/bin/env python3
"""
Generate a ROS bag scenario from trajectories stored in a GeoJSON file.
Optionally includes traffic light stop line statuses.
"""

import argparse
import json
import yaml
import math
import time
import logging
from pathlib import Path

import shapely.geometry
import lanelet2
import rospy
import rosbag

from autoware_mini.geometry import get_orientation_from_heading
from autoware_mini.msg import DetectedObjectArray, DetectedObject, StopLineStatus, StopLineStatusArray
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, PoseStamped, TwistStamped, TransformStamped

# Constants
FREQUENCY = 10  # Hz

# Default property values
DEFAULT_SPEED = 0.0  # km/h
DEFAULT_ACCELERATION = 0.0  # m/s²
DEFAULT_ELEVATION = 0.0  # meters
DEFAULT_DELAY = 0.0  # seconds
DEFAULT_DURATION = 10.0  # seconds
DEFAULT_EGO_DURATION = 10.0  # seconds
DEFAULT_LENGTH = 4.0  # meters
DEFAULT_WIDTH = 2.0  # meters
DEFAULT_HEIGHT = 1.7  # meters

def setup_logger() -> None:
    """Configure simple logger output."""
    logging.basicConfig(format="%(levelname)s: %(message)s", level=logging.INFO)


def read_geojson(file_path: Path) -> dict:
    """Read a GeoJSON file from disk."""
    with file_path.open("r") as f:
        return json.load(f)


def get_heading_at_distance(linestring: shapely.geometry.LineString, distance: float) -> float:
    """Compute heading (radians) along a linestring at given distance."""
    point_front = linestring.interpolate(distance + 0.1)
    point_back = linestring.interpolate(max(0.0, distance - 0.1))
    return math.atan2(point_front.y - point_back.y, point_front.x - point_back.x)


def convert_to_detected_objects(geojson_data: dict, projector: lanelet2.projection.UtmProjector):
    """Convert non-ego trajectories to DetectedObject timelines."""
    all_detected_objects = []
    trajectories = []

    for obj_id, feature in enumerate(geojson_data.get("features", [])):
        if feature["properties"].get("label") == "ego":
            continue

        coords = []
        for lon, lat in feature["geometry"]["coordinates"]:
            gps_point = lanelet2.core.GPSPoint(lat, lon, 0.0)
            utm_point = projector.forward(gps_point)
            coords.append((utm_point.x, utm_point.y))
        trajectory = shapely.geometry.LineString(coords)
        trajectories.append(trajectory)

        props = feature["properties"]
        label = props.get("label")
        if label is None:
            label = "unknown"
        # Use `or` pattern instead of dict.get(key, default) to handle both missing keys
        # and explicit null values in GeoJSON (null becomes None in Python, which is falsy)
        initial_speed = float(props.get("speed") or DEFAULT_SPEED) / 3.6
        acceleration = float(props.get("acceleration") or DEFAULT_ACCELERATION)
        elevation = float(props.get("elevation") or DEFAULT_ELEVATION)
        delay = float(props.get("delay") or DEFAULT_DELAY)
        duration = float(props.get("duration") or DEFAULT_DURATION)
        length = float(props.get("length") or DEFAULT_LENGTH)
        width = float(props.get("width") or DEFAULT_WIDTH)

        delay_steps = int(delay * FREQUENCY)
        duration_steps = int(duration * FREQUENCY)
        dt = 1.0 / FREQUENCY

        timeline = [None] * duration_steps
        distance_along = 0.0
        current_speed = initial_speed

        for step in range(delay_steps, duration_steps):
            delta_distance = current_speed * dt
            distance_along = min(trajectory.length, distance_along + delta_distance)
            current_speed = max(0.0, current_speed + acceleration * dt)
            p = trajectory.interpolate(distance_along)
            heading = get_heading_at_distance(trajectory, distance_along)

            # Only add object height offset if elevation is specified (non-zero), to avoid triggering 3D lanelet matching
            point = Point(x=p.x, y=p.y, z=(elevation + DEFAULT_HEIGHT / 2.0) if elevation else 0.0)
            half_l, half_w = length / 2.0, width / 2.0

            dx, dy = half_l * math.cos(heading), half_l * math.sin(heading)
            wx, wy = -half_w * math.sin(heading), half_w * math.cos(heading)

            convex_hull = [
                p.x + dx + wx, p.y + dy + wy, elevation,
                p.x + dx - wx, p.y + dy - wy, elevation,
                p.x - dx - wx, p.y - dy - wy, elevation,
                p.x - dx + wx, p.y - dy + wy, elevation,
                p.x + dx + wx, p.y + dy + wy, elevation
            ]

            obj = DetectedObject()
            obj.id = obj_id
            obj.label = label
            obj.centroid = point
            obj.center = point
            obj.valid = True
            obj.dimensions.x = length
            obj.dimensions.y = width
            obj.dimensions.z = DEFAULT_HEIGHT
            obj.heading = heading
            obj.convex_hull = convex_hull

            timeline[step] = obj

        all_detected_objects.append(timeline)

    return all_detected_objects, trajectories


def extract_ego_data(geojson_data: dict, projector: lanelet2.projection.UtmProjector):
    """Extract ego vehicle initial pose, goal, velocity, and trajectory."""
    for feature in geojson_data.get("features", []):
        if feature["properties"].get("label") != "ego":
            continue

        props = feature["properties"]
        duration = float(props.get("duration") or DEFAULT_EGO_DURATION)
        elevation = float(props.get("elevation") or DEFAULT_ELEVATION)
        coords = []
        for lon, lat in feature["geometry"]["coordinates"]:
            gps_point = lanelet2.core.GPSPoint(lat, lon, 0.0)
            utm_point = projector.forward(gps_point)
            coords.append((utm_point.x, utm_point.y))

        trajectory = shapely.geometry.LineString(coords)
        start_point = trajectory.interpolate(0)
        dir_point = trajectory.interpolate(0.1)
        heading = math.atan2(dir_point.y - start_point.y, dir_point.x - start_point.x)

        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = "map"
        initial_pose.pose.pose.position.x = start_point.x
        initial_pose.pose.pose.position.y = start_point.y
        initial_pose.pose.pose.position.z = elevation
        initial_pose.pose.pose.orientation = get_orientation_from_heading(heading)

        end_point = trajectory.interpolate(trajectory.length)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position = Point(x=end_point.x, y=end_point.y, z=elevation)

        speed = float(props.get("speed") or DEFAULT_SPEED) / 3.6
        velocity = TwistStamped()
        velocity.header.frame_id = "map"
        velocity.twist.linear.x = speed

        return initial_pose, goal_pose, velocity, duration, trajectory

    return None, None, None, None, None


def create_traffic_light_messages(trajectories: list, projector: lanelet2.projection.UtmProjector, bag_duration: float, output_bag: Path):
    """Create StopLineStatusArray messages based on lanelet2 traffic light GeoJSON."""
    map_name = "tartu_large"
    map_dir = output_bag.parent.parent
    tl_geojson_path = map_dir / map_name / "geojson" / "traffic_lights.geojson"

    traffic_lights = read_geojson(tl_geojson_path)
    selected_props = []

    for feature in traffic_lights["features"]:
        coords = []
        for lon, lat, z in feature["geometry"]["coordinates"]:
            gps_point = lanelet2.core.GPSPoint(lat, lon, z)
            utm_point = projector.forward(gps_point)
            coords.append((utm_point.x, utm_point.y))
        stop_line = shapely.geometry.LineString(coords)
        if any(stop_line.intersects(t) for t in trajectories):
            selected_props.append(feature["properties"])

    num_steps = int(bag_duration * FREQUENCY)
    tl_messages = []

    for step in range(num_steps):
        current_time = float(step) / FREQUENCY
        status_array = StopLineStatusArray()

        for props in selected_props:
            tl_id = props["id"]
            offset = props["offset"]
            red_dur = props["red_duration"]
            green_dur = props["green_duration"]
            cycle = red_dur + green_dur
            time_in_cycle = (current_time + offset) % cycle

            status = (
                StopLineStatus.STATUS_STOP
                if time_in_cycle < red_dur
                else StopLineStatus.STATUS_GO
            )

            stop_line_status = StopLineStatus()
            stop_line_status.stop_line_id = tl_id
            stop_line_status.status = status
            status_array.statuses.append(stop_line_status)

        tl_messages.append(status_array)

    return tl_messages


def write_to_rosbag(
    goal_delay: float, detected_objects_topic: str, traffic_light_status_topic: str, base_link_to_car_front: float,
    detected_objects, initial_pose, goal_pose, velocity, bag_duration: float, ego_traj, output_bag: Path, tl_messages=None
):
    """Write the scenario data into a ROS bag."""
    with rosbag.Bag(str(output_bag), "w") as bag:
        start_time = time.time()
        steps_10hz = int(bag_duration * FREQUENCY)
        steps_50hz = steps_10hz * 5
        ego_step = ego_traj.length / steps_50hz

        # Detected objects
        for step in range(steps_10hz):
            msg_time = rospy.Time.from_sec(start_time + step / FREQUENCY)
            obj_array = DetectedObjectArray()
            obj_array.header.stamp = msg_time
            obj_array.header.frame_id = "map"

            for tl in detected_objects:
                if step < len(tl) and isinstance(tl[step], DetectedObject):
                    obj_array.objects.append(tl[step])
            bag.write(detected_objects_topic, obj_array, t=msg_time)

            if tl_messages and step < len(tl_messages):
                tl_messages[step].header.stamp = msg_time
                tl_messages[step].type = StopLineStatusArray.TRAFFIC_LIGHT
                bag.write(traffic_light_status_topic, tl_messages[step], t=msg_time)

        # write only goal pose as ego trajectory into /tf
        ref_goal_point = ego_traj.interpolate(max(0.0, ego_traj.length - base_link_to_car_front))
        ref_heading = get_heading_at_distance(ego_traj, ego_traj.length)
        ref_orientation = get_orientation_from_heading(ref_heading)

        for step in range(steps_50hz):
            msg_time = rospy.Time.from_sec(start_time + step / (FREQUENCY * 5))
            tf_msg = TFMessage()
            t = TransformStamped()
            t.header.stamp = msg_time
            t.header.frame_id = "map"
            t.child_frame_id = "lexus_shadow"
            t.transform.translation.x = ref_goal_point.x
            t.transform.translation.y = ref_goal_point.y
            t.transform.translation.z = goal_pose.pose.position.z
            t.transform.rotation = ref_orientation
            tf_msg.transforms.append(t)
            bag.write("/tf", tf_msg, t=msg_time)

        # Initial conditions
        initial_pose.header.stamp = rospy.Time.from_sec(start_time)
        bag.write("/initialpose", initial_pose, t=initial_pose.header.stamp)

        goal_pose.header.stamp = rospy.Time.from_sec(start_time + goal_delay)
        bag.write("/move_base_simple/goal", goal_pose, t=goal_pose.header.stamp)

        velocity.header.stamp = rospy.Time.from_sec(start_time)
        bag.write("/initialvelocity", velocity, t=velocity.header.stamp)


def main() -> None:
    parser = argparse.ArgumentParser(description="Create ROS bag scenario from trajectories")
    parser.add_argument("input_geojson", type=Path, help="Path to input trajectory GeoJSON file")
    parser.add_argument("output_bag", type=Path, help="Path to output ROS bag file")
    parser.add_argument("--goal_delay", type=float, default=0.2, help="Delay goal from start time (default: 0.2)")
    parser.add_argument("--detected_objects_topic", default="/perception/detected_objects")
    parser.add_argument("--traffic_light_status_topic", default="/perception/traffic_light_status")
    parser.add_argument("--base_link_to_car_front", type=float, default=4.5,
                        help="Distance from base_link to car front (default: 4.5 meters)")
    parser.add_argument(
        "--add_traffic_lights",
        action="store_true",
        help="Include traffic light messages if exported from Lanelet2",
    )
    args = parser.parse_args()

    setup_logger()

    # read parameters from yaml file
    map_name = args.input_geojson.parent.parent.name
    maps_dir = args.input_geojson.parent.parent.parent.parent / "maps"
    yaml_path = maps_dir / f"{map_name}.yaml"
    with open(yaml_path, 'r') as f:
        map_params = yaml.safe_load(f)
    utm_origin_lat = map_params.get('utm_origin_lat')
    utm_origin_lon = map_params.get('utm_origin_lon')

    origin = lanelet2.io.Origin(utm_origin_lat, utm_origin_lon)
    projector = lanelet2.projection.UtmProjector(origin, True, False)

    geojson_data = read_geojson(args.input_geojson)
    detected_objects, trajectories = convert_to_detected_objects(geojson_data, projector)
    initial_pose, goal_pose, velocity, bag_duration, ego_traj = extract_ego_data(geojson_data, projector)

    trajectories.append(ego_traj)
    tl_messages = None

    if args.add_traffic_lights:
        tl_messages = create_traffic_light_messages(trajectories, projector, bag_duration, args.output_bag)

    write_to_rosbag(args.goal_delay, args.detected_objects_topic, args.traffic_light_status_topic, args.base_link_to_car_front,
                    detected_objects, initial_pose, goal_pose, velocity, bag_duration, ego_traj, args.output_bag, tl_messages)

    # Get path relative to autoware_mini package
    script_dir = Path(__file__).resolve().parent
    autoware_mini_dir = script_dir.parent.parent
    relative_path = args.output_bag.resolve().relative_to(autoware_mini_dir)
    logging.info(f"Successfully created bag {relative_path}")


if __name__ == "__main__":
    main()
