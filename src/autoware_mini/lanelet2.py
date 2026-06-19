import lanelet2
from collections import defaultdict
import shapely
import numpy as np
import math
import rospy
import warnings


def load_lanelet2_map(lanelet2_map_path, print_errors=False):
    """
    Load a lanelet2 map from a file and return it.
    :param lanelet2_map_path: name of the lanelet2 map file
    :param print_errors: print errors if True
    :return: lanelet2 map
    """

    use_custom_origin = rospy.get_param("/localization/use_custom_origin")
    utm_origin_lat = rospy.get_param("/localization/utm_origin_lat")
    utm_origin_lon = rospy.get_param("/localization/utm_origin_lon")

    origin = lanelet2.io.Origin(utm_origin_lat, utm_origin_lon)
    projector = lanelet2.projection.UtmProjector(origin, use_custom_origin, False)

    lanelet2_map, errors = lanelet2.io.loadRobust(lanelet2_map_path, projector)
    if errors and print_errors:
        for error in errors:
            rospy.logwarn(rospy.get_name() + ": " + error)

    return lanelet2_map

def utm_origin():
    """
    Load the origin of the local UTM coordinate system in (lat, lon) format and transform it to UTM35N coordinates
    :param utm_origin_lat: local utm origin latitude
    :param utm_origin_lon: local utm origin longitude
    :return: utm coordinates of utm_origin lat lon point
    """

    utm_origin_lat = rospy.get_param("/localization/utm_origin_lat")
    utm_origin_lon = rospy.get_param("/localization/utm_origin_lon")

    # origin point of the UTM35N coordinate system
    origin = lanelet2.io.Origin(utm_origin_lat, utm_origin_lon)
    projector = lanelet2.projection.UtmProjector(origin, False, False)

    gps_point = lanelet2.core.GPSPoint(utm_origin_lat, utm_origin_lon, 0)
    utm_point = projector.forward(gps_point)
    return utm_point.x, utm_point.y

def get_linestrings_in_area(lanelet2_map, x, y, extent):
    """
    Get all linestrings within a given area
    :param lanelet2_map: lanelet2 map
    :param x: x-coordinate of the point
    :param y: y-coordinate of the point
    :param extent: the half-length of the bounding box in both x and y directions
    :return: {line_id: line, ...}
    """

    search_box = lanelet2.core.BoundingBox2d(lanelet2.core.BasicPoint2d(x - extent, y - extent), lanelet2.core.BasicPoint2d(x + extent, y + extent))
    return lanelet2_map.lineStringLayer.search(search_box)

def get_lanelets_in_range(lanelet2_map, x, y, z, radius, subtypes=None, max_height_difference=0):
    """
    Get all lanelets within a given radius, optionally filtering by subtype and height.
    :param lanelet2_map: lanelet2 map
    :param x: x-coordinate of the point
    :param y: y-coordinate of the point
    :param z: z-coordinate of the point for height filtering
    :param radius: maximum distance between geometries. If zero, only primitives containing the element are returned.
    :param subtypes: (optional) list of subtypes to filter by
    :param max_height_difference: maximum allowed height above lanelet centerline
    :return: list of lanelets
    """

    # Find lanelets within range
    lanelets = lanelet2.geometry.findWithin2d(lanelet2_map.laneletLayer, lanelet2.core.BasicPoint2d(x, y), radius)

    # Apply height filtering if z coordinate is valid (z=0 means unknown, e.g. in bag scenarios)
    if z > 0:
        point3d = lanelet2.core.BasicPoint3d(x, y, z)
        lanelets = [(dist, lanelet) for dist, lanelet in lanelets
                    if abs(z - lanelet2.geometry.project(lanelet.centerline, point3d).z) < max_height_difference]

    # Apply subtype filtering and extract lanelet objects
    if subtypes is not None:
        lanelets = [lanelet for _, lanelet in lanelets
                    if "subtype" in lanelet.attributes and lanelet.attributes["subtype"] in subtypes]
    else:
        lanelets = [lanelet for _, lanelet in lanelets]

    return lanelets

def get_crosswalks(lanelet2_map, segment_length):
    """
    Find all crosswalks on map, convert to shapely geometries, and return as arrays.
    :param lanelet2_map: lanelet2 map
    :param segment_length: max segment length for boundary densification
    :return: (crosswalks, crosswalk_polygons) numpy arrays
    """

    crosswalks = []
    polygons = []
    for lanelet in lanelet2_map.laneletLayer:
        if lanelet.attributes:
            if lanelet.attributes["subtype"] == "crosswalk":
                polygon = shapely.polygons([(p.x, p.y) for p in lanelet.polygon2d()])
                shapely.prepare(polygon)
                left_boundary = shapely.linestrings([(p.x, p.y) for p in lanelet.leftBound])
                right_boundary = shapely.linestrings([(p.x, p.y) for p in lanelet.rightBound])
                left_boundary = shapely.segmentize(left_boundary, max_segment_length=segment_length)
                right_boundary = shapely.segmentize(right_boundary, max_segment_length=segment_length)
                crosswalks.append({
                    'polygon': polygon,
                    'left_coords': shapely.get_coordinates(left_boundary),
                    'right_coords': shapely.get_coordinates(right_boundary),
                    'left_centroid': left_boundary.centroid,
                    'right_centroid': right_boundary.centroid,
                })
                polygons.append(polygon)

    return np.array(crosswalks), np.array(polygons)

def get_stop_lines(lanelet2_map, x=None, y=None, extent=None, subtypes=None, return_subtypes=False, return_speeds=False):
    """
    Get stop lines from the map, optionally within a bounding box and/or filtered by subtype.
    :param lanelet2_map: lanelet2 map
    :param x: (optional) x-coordinate of the bounding box center
    :param y: (optional) y-coordinate of the bounding box center
    :param extent: (optional) half-length of the bounding box in both x and y directions
    :param subtypes: (optional) list of subtypes to filter by
    :param return_subtypes: if True, also return a dict mapping stop_line_id to subtype
    :param return_speeds: if True, also return a dict mapping stop_line_id to speed (km/h) or NaN
    :return: {stop_line_id: linestring, ...}, optionally {stop_line_id: subtype, ...}, optionally {stop_line_id: speed, ...}
    """

    if x is None or y is None or extent is None:
        linestrings = lanelet2_map.lineStringLayer
    else:
        linestrings = get_linestrings_in_area(lanelet2_map, x, y, extent)

    filtered_linestrings = {}
    if return_subtypes:
        filtered_subtypes = {}
    if return_speeds:
        filtered_speeds = {}

    for line in linestrings:
        if "type" not in line.attributes or line.attributes["type"] != "stop_line":
            continue
        if subtypes is not None and ("subtype" not in line.attributes or line.attributes["subtype"] not in subtypes):
            continue
        filtered_linestrings[line.id] = shapely.linestrings([(p.x, p.y, p.z) for p in line])
        if return_subtypes:
            filtered_subtypes[line.id] = line.attributes["subtype"]
        if return_speeds:
            filtered_speeds[line.id] = float(line.attributes["speed"]) if "speed" in line.attributes else np.nan

    result = (filtered_linestrings,)
    if return_subtypes:
        result += (filtered_subtypes,)
    if return_speeds:
        result += (filtered_speeds,)
    return result if len(result) > 1 else result[0]

def get_stop_lines_api_id(lanelet2_map, x = None, y = None, extent = None):
    """
    Retrieve stop line ids within a specified area from a given point on a Lanelet2 map.
    :param lanelet2_map: the Lanelet2 map
    :param x: x-coordinate of the given point
    :param y: y-coordinate of the given point
    :param extent: the half-length of the bounding box in both x and y directions
    :return: A dictionary of stop line ids and api keys that fall within the search area
    """
    if x is None or y is None or extent is None:
        linestrings = lanelet2_map.lineStringLayer
    else:
        linestrings = get_linestrings_in_area(lanelet2_map, x, y, extent)

    stop_line_ids = {}
    for line in linestrings:
        if line.attributes and line.attributes["type"] == "stop_line" and "api_id" in line.attributes:
            stop_line_ids[line.id] = line.attributes["api_id"]

    return stop_line_ids

def get_traffic_light_stop_lines(lanelet2_map):
    """
    Iterate over all regulatory_elements with subtype traffic light and extract only the stop lines 
    and their geometries that have traffic light associated to them.
    Organize the data into a dictionary indexed by stop line id.
    :param lanelet2_map: lanelet2 map
    :return: {stop_line_id: linestring, ...}
    """

    stop_lines = {}

    for reg_el in lanelet2_map.regulatoryElementLayer:
        if reg_el.attributes["subtype"] == "traffic_light":
            # ref_line is the stop line and there is only 1 stop line per traffic light reg_el
            if "ref_line" not in reg_el.parameters:
                warnings.warn(f"Traffic light regulatory element {reg_el.id} has no ref_line parameter.")
                continue

            stop_line_id = reg_el.parameters["ref_line"][0].id
            stop_lines[stop_line_id] = shapely.linestrings([(p.x, p.y, p.z) for p in reg_el.parameters["ref_line"][0]])

    return stop_lines

def get_right_of_way_regulatory_elements(lanelet2_map):
    """
    Iterate over all regulatory_elements with subtype right_of_way and extract all linked right_of_way lanelets with their end line polygons and headings.
    :param lanelet2_map: lanelet2 map
    :return: {stop_line_id: [polygons], ...}, {stop_line_id: [headings], ...}
    """

    right_of_way_polygons = {}
    right_of_way_headings = {}

    for reg_el in lanelet2_map.regulatoryElementLayer:
        if reg_el.attributes["subtype"] == "right_of_way":
            # skip if no ref_line (stop line) is defined or no right_of_way lanelets are defined
            if "ref_line" not in reg_el.parameters or "right_of_way" not in reg_el.parameters:
                warnings.warn(f"Right of way regulatory element {reg_el.id} is missing ref_line or right_of_way parameters.")
                continue

            # collect all the stop line ids (usually only one)
            stop_line_ids = []
            for stop_line in reg_el.parameters["ref_line"]:
                stop_line_ids.append(stop_line.id)

            # create following arrays for each right_of_way lanelet
            lanelet_polygons = []
            lanelet_headings = []
            for lanelet in reg_el.parameters["right_of_way"]:
                # TODO possibly could additionally check if lanelet is of subtype right_of_way (applies after map changes)
                coords = [(p.x, p.y, p.z) for p in lanelet.polygon3d()]
                lanelet_polygons.append(shapely.polygons(coords))

                centerline = lanelet.centerline
                heading = math.atan2(centerline[-1].y - centerline[-2].y, centerline[-1].x - centerline[-2].x)
                lanelet_headings.append(heading)

            # for every stop line part of right of way regulatory element store all lanelet data
            for stop_line_id in stop_line_ids:
                right_of_way_polygons[stop_line_id] = np.array(lanelet_polygons)
                right_of_way_headings[stop_line_id] = np.array(lanelet_headings)

    return right_of_way_polygons, right_of_way_headings

def get_traffic_light_bboxes(lanelet2_map):
    """
    Iterate over all regulatory_elements with subtype traffic light and extract all linked traffic lights with their four corners.
    :param lanelet2_map: lanelet2 map
    :return: {stop_line_id: {traffic_light_id: [(tlx, tly, tlz), (trx, try, trz), (blx, bly, blz), (brx, bry, brz)], ...}, ...}
    """

    traffic_lights = defaultdict(dict)

    for reg_el in lanelet2_map.regulatoryElementLayer:
        if reg_el.attributes["subtype"] == "traffic_light":
            # ref_line is the stop line and there is only 1 stop line per traffic light reg_el
            stop_line_id = reg_el.parameters["ref_line"][0].id

            for tfl in reg_el.parameters["refers"]:
                tfl_height = float(tfl.attributes["height"])
                traffic_light_id = tfl.id

                # data stored in order of: top_left, top_right, bottom_left, bottom_right
                traffic_light_data = [(tfl[0].x, tfl[0].y, tfl[0].z + tfl_height),
                                      (tfl[1].x, tfl[1].y, tfl[1].z + tfl_height),
                                      (tfl[0].x, tfl[0].y, tfl[0].z),
                                      (tfl[1].x, tfl[1].y, tfl[1].z)]

                traffic_lights[stop_line_id][traffic_light_id] = traffic_light_data

    return traffic_lights

def get_stop_lines_center(lanelet2_map):
    """
    Iterate over all regulatory_elements with subtype traffic light and extract the stop lines centers.
    Organize the data into dictionary indexed by stop line id that contains stop line center coordinates and respective traffic light ids
    :param lanelet2_map: lanelet2 map
    :return: {stop_line_id: [[(center_x, center_y), [PlIds]], ...], ...}
    """

    stop_line_centers = {}

    for reg_el in lanelet2_map.regulatoryElementLayer:
        if reg_el.attributes["subtype"] == "traffic_light":
            # ref_line is the stop line and there is only 1 stop line per traffic light reg_el
            link = reg_el.parameters["ref_line"][0]
            # Get all geometry points of the stop line
            line_points = [[point.x, point.y] for point in link]
            # Extract center point from stop line
            center_x, center_y = np.mean(line_points, axis=0)
            # Extract traffic light (Pole) ids for the same stop line
            plIds = [tfl.id for tfl in reg_el.parameters["refers"]]

            stop_line_centers[link.id] = [(center_x, center_y), plIds] 

    return stop_line_centers

def find_following_lane_change_lanelet(lanelet, route, is_left_side):
    """
    Checks if lane change is possible on the following lanelet.
    If yes then return the following lanelet
    :param lanelet: current lanelet
    :param route: lanelet2 route object
    :param is_left_side: wether the current lanelet is to the left of the adjancent lanelet
    :return: the following lanelet if it is suitable for a lane change, None otherwise
    """
    # All following relations of the current lanelet
    following_relations = route.followingRelations(lanelet)
    
    if is_left_side:
        adjacent_relation = route.leftRelation(lanelet)
    else:
        adjacent_relation = route.rightRelation(lanelet)

    # Return None if there are no adajncent relations 
    if adjacent_relation is None:
        return None
    
    # All following relations of the adjacent lanelet
    adjacent_following_relations = route.followingRelations(adjacent_relation.lanelet)

    for following_relation in following_relations:
        # Get the adjancent relation of the follwing relaton
        if is_left_side:
            following_adjacent_relation = route.leftRelation(following_relation.lanelet)
        else:
            following_adjacent_relation = route.rightRelation(following_relation.lanelet)
        
        if following_adjacent_relation is None:
            continue
        
        # Suitable following lanelet is found if the its adjancent lanelet matches the current lanelet's follower
        for adjacent_following_relation in adjacent_following_relations:
            if adjacent_following_relation.lanelet == following_adjacent_relation.lanelet:
                return following_relation.lanelet

    return None

def follow_lanelets(routing_graph, current_lanelet, remaining_distance):
    """
    Recursively find following lanelets for a given distance and return all possible trajectories
    :param routing_grpah: lanelet2 routing graph
    :param current_lanelet: current lanelet
    :param remaining_distance: remaining distance to follow
    :return: list of possible trajectories
    """

    current_lanelet_length = lanelet2.geometry.length2d(current_lanelet)

    if remaining_distance <= current_lanelet_length:
        return [[current_lanelet]]  # Base case: return a single-lanelet trajectory

    next_lanelets = routing_graph.following(current_lanelet)
    if not next_lanelets:
        return [[current_lanelet]]

    remaining_distance -= current_lanelet_length

    trajectories = []  # Store all possible trajectories
    for next_lanelet in next_lanelets:
        # Recursively follow the lanelets
        following_trajectories = follow_lanelets(routing_graph, next_lanelet, remaining_distance)
        for traj in following_trajectories:
            trajectories.append([current_lanelet] + traj)  # Add current lanelet to each path

    return trajectories

def get_height_at_position(lanelet2_map, x, y, z, search_radius=0.0, max_height_difference=None):
    """
    Get the height at a given position on the lanelet2 map
    :param lanelet2_map: lanelet2 map
    :param x: x-coordinate of the point
    :param y: y-coordinate of the point
    :param z: z-coordinate of the point
    :param search_radius: 2D radius (m) within which to search for lanelets; 0 matches only lanelets containing (x, y)
    :param max_height_difference: skip lanelets whose centerline height differs from z by more than this many meters; None disables the check (e.g. when input z is unknown, as with an RViz-set initial pose)
    :return: height at the given position, or input z if no matching lanelet is found
    """

    point2d = lanelet2.core.BasicPoint2d(x, y)
    nearest = lanelet2.geometry.findWithin2d(lanelet2_map.laneletLayer, point2d, search_radius)
    for _, lanelet in nearest:
        point3d = lanelet2.core.BasicPoint3d(x, y, z)
        projected_point = lanelet2.geometry.project(lanelet.centerline, point3d)
        # Skip overlapping lanelets at a clearly different height (e.g. bridge above the road)
        if max_height_difference is not None and abs(projected_point.z - z) > max_height_difference:
            continue
        return projected_point.z

    return z