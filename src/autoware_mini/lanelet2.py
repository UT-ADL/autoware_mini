from lanelet2.io import Origin, loadRobust
from lanelet2.projection import UtmProjector
from lanelet2.core import GPSPoint, BasicPoint2d, BoundingBox2d, BasicPoint3d
from lanelet2.geometry import length2d, findNearest, project, findWithin2d
import shapely
import numpy as np
import rospy


def load_lanelet2_map(lanelet2_map_path):
    """
    Load a lanelet2 map from a file and return it
    :param lanelet2_map_path: name of the lanelet2 map file
    :param coordinate_transformer: coordinate transformer
    :param use_custom_origin: use custom origin
    :param utm_origin_lat: utm origin latitude
    :param utm_origin_lon: utm origin longitude
    :return: lanelet2 map
    """

    # get parameters
    coordinate_transformer = rospy.get_param("/localization/coordinate_transformer")
    use_custom_origin = rospy.get_param("/localization/use_custom_origin")
    utm_origin_lat = rospy.get_param("/localization/utm_origin_lat")
    utm_origin_lon = rospy.get_param("/localization/utm_origin_lon")

    # Load the map using Lanelet2
    if coordinate_transformer == "utm":
        projector = UtmProjector(Origin(utm_origin_lat, utm_origin_lon), use_custom_origin, False)
    else:
        raise ValueError('Unknown coordinate_transformer for loading the Lanelet2 map ("utm" should be used): ' + coordinate_transformer)

    lanelet2_map, errors = loadRobust(lanelet2_map_path, projector)

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
    origin = Origin(utm_origin_lat, utm_origin_lon)
    projector = UtmProjector(origin, False, False)

    gps_point = GPSPoint(utm_origin_lat, utm_origin_lon, 0)
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

    search_box = BoundingBox2d(BasicPoint2d(x - extent, y - extent), BasicPoint2d(x + extent, y + extent))
    return lanelet2_map.lineStringLayer.search(search_box)

def get_lanelets_in_range(lanelet2_map, x, y, radius, subtypes=None):
    """
    Get all lanelets within a given radius, optionally filtering by subtype.
    :param lanelet2_map: lanelet2 map
    :param x: x-coordinate of the point
    :param y: y-coordinate of the point
    :param radius: maximum distance between geometries. If zero, only primitives containing the element are returned.
    :param subtypes: (optional) list of subtypes to filter by
    :return: list of lanelets
    """

    lanelets = findWithin2d(lanelet2_map.laneletLayer, BasicPoint2d(x, y), radius)

    if subtypes is not None:
        return [lanelet for _, lanelet in lanelets if "subtype" in lanelet.attributes and lanelet.attributes["subtype"] in subtypes]
    else:
        return [lanelet for _, lanelet in lanelets]

def get_stop_lines_in_area(lanelet2_map, x, y, extent, subtypes=None):
    """
    Get all stop lines within a given area, optionally filtering by subtype.
    :param lanelet2_map: lanelet2 map
    :param x: x-coordinate of the point
    :param y: y-coordinate of the point
    :param extent: the half-length of the bounding box in both x and y directions
    :param subtype: (optional) list of subtypes to filter by
    :return: list of stopline linestrings
    """

    linestrings = get_linestrings_in_area(lanelet2_map, x, y, extent)
    stop_lines = []
    for line in linestrings:
        if "type" in line.attributes and line.attributes["type"] == "stop_line":
            if subtypes is not None:
                if "subtype" not in line.attributes or line.attributes["subtype"] not in subtypes:
                    continue
            stop_lines.append(shapely.linestrings([(p.x, p.y, p.z) for p in line]))
    return stop_lines

def get_crosswalks(lanelet2_map):
    """
    Find all crosswalks on map and return as a list 
    :param lanelet2_map: lanelet2 map
    :return: crosswalk lanelets
    """

    crosswalks = []
    for lanelet in lanelet2_map.laneletLayer:
        if lanelet.attributes:
            if lanelet.attributes["subtype"] == "crosswalk":
                crosswalks.append(lanelet)

    return crosswalks

def get_stop_lines_using_subtype(lanelet2_map, subtypes):
    """
    Get all stop lines with a specific subtype
    :param lanelet2_map: lanelet2 map
    :param subtype: list of subtype's to search for
    :return: {line_id: linestring, ...}
    """

    filtered_lines = {}
    for line in lanelet2_map.lineStringLayer:
        if "type" in line.attributes and line.attributes["type"] == "stop_line":
            if "subtype" in line.attributes and line.attributes["subtype"] in subtypes:
                filtered_lines[line.id] = shapely.linestrings([(p.x, p.y, p.z) for p in line])
    return filtered_lines

def get_traffic_light_stop_lines(lanelet2_map):
    """
    Get all stop lines that are associated with traffic lights
    :param lanelet2_map: lanelet2 map
    :return: {line_id: line, ...}
    """

    lines = {}
    for reg_el in lanelet2_map.regulatoryElementLayer:
        if reg_el.attributes["subtype"] == "traffic_light" and "ref_line" in reg_el.parameters:
            for line in reg_el.parameters["ref_line"]:
                lines[line.id] = shapely.linestrings([(p.x, p.y, p.z) for p in line])
    return lines

# TODO: Add function to get all stop lines that are associated with traffic lights join with next function

def get_stop_lines_api_id(lanelet2_map, x = None, y = None, extent = None):
    """
    Retrieve stop line ids within a specified area from a given point on a Lanelet2 map.
    :param lanelet2_map: the Lanelet2 map
    :param x: x-coordinate of the given point
    :param y: y-coordinate of the given point
    :param extent: the half-length of the bounding box in both x and y directions
    :return: A dictionary of stopline ids and api keys that fall within the search area
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

def get_stoplines_trafficlights(lanelet2_map):
    """
    Iterate over all regulatory_elements with subtype traffic light and extract the stoplines and sinals.
    Organize the data into dictionary indexed by stopline id that contains a traffic_light id and the four coners of the traffic light.
    :param lanelet2_map: lanelet2 map
    :return: {stopline_id: {traffic_light_id: {'top_left': [x, y, z], 'top_right': [...], 'bottom_left': [...], 'bottom_right': [...]}, ...}, ...}
    """

    signals = {}

    for reg_el in lanelet2_map.regulatoryElementLayer:
        if reg_el.attributes["subtype"] == "traffic_light":
            # ref_line is the stop line and there is only 1 stopline per traffic light reg_el
            linkId = reg_el.parameters["ref_line"][0].id
            
            for tfl in reg_el.parameters["refers"]:
                tfl_height = float(tfl.attributes["height"])
                # plId represents the traffic light (pole), one stop line can be associated with multiple traffic lights
                plId = tfl.id

                traffic_light_data = {'top_left': [tfl[0].x, tfl[0].y, tfl[0].z + tfl_height], 
                                      'top_right': [tfl[1].x, tfl[1].y, tfl[1].z + tfl_height], 
                                      'bottom_left': [tfl[0].x, tfl[0].y, tfl[0].z], 
                                      'bottom_right': [tfl[1].x, tfl[1].y, tfl[1].z]}


                # signals is a dictionary indexed by stopline id and contains dictionary of traffic lights indexed by pole id
                # which in turn contains a dictionary of traffic light corners
                signals.setdefault(linkId, {}).setdefault(plId, traffic_light_data)

    return signals

def get_stoplines_center(lanelet2_map):
    """
    Iterate over all regulatory_elements with subtype traffic light and extract the stoplines centers.
    Organize the data into dictionary indexed by stopline id that contains stopline center coordinates and respective traffic light ids
    :param lanelet2_map: lanelet2 map
    :return: {stopline_id: [[(center_x, center_y), [PlIds]], ...], ...}
    """

    stopline_centers = {}

    for reg_el in lanelet2_map.regulatoryElementLayer:
        if reg_el.attributes["subtype"] == "traffic_light":
            # ref_line is the stop line and there is only 1 stopline per traffic light reg_el
            link = reg_el.parameters["ref_line"][0]
            # Get all geometry points of the stopline
            line_points = [[point.x, point.y] for point in link]
            # Extract center point from stopline
            center_x, center_y = np.mean(line_points, axis=0)
            # Extract traffic light (Pole) ids for the same stopline
            plIds = [tfl.id for tfl in reg_el.parameters["refers"]]

            stopline_centers[link.id] = [(center_x, center_y), plIds] 

    return stopline_centers

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

    current_lanelet_length = length2d(current_lanelet)

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

def get_height_at_position(lanelet2_map, x, y, z, num_lanelets=1, max_height_difference=3):
    """
    Get the height at a given position on the lanelet2 map
    :param lanelet2_map: lanelet2 map
    :param x: x-coordinate of the point
    :param y: y-coordinate of the point
    :param z: z-coordinate of the point
    :param num_lanelets: number of lanelets to consider (default is 1)
    :param max_height_difference: maximum height difference allowed
    :return: height at the given position
    """

    point2d = BasicPoint2d(x, y)
    nearest = findNearest(lanelet2_map.laneletLayer, point2d, num_lanelets) #TODO: currently handles initialization only in 2d space
    for _, lanelet in nearest:
        point3d = BasicPoint3d(x, y, z)
        projected_point = project(lanelet.centerline, point3d)
        # Avoid matching with lanelets that have large height difference compared to the current position
        if num_lanelets != 1 and abs(projected_point.z - z) > max_height_difference: 
            continue
        return projected_point.z

    return z