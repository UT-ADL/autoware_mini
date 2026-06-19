import shapely
import numpy as np
import warnings

def normalize_vectors(v):
    """Normalize vectors along the last axis, returning unit vectors. Zero vectors are left as zero."""
    norm = np.linalg.norm(v, axis=-1, keepdims=True)
    return v / np.where(norm == 0, 1, norm)

def compute_offset_2d(points, dists):
    """
    Core offset computation with per-vertex distances. Offset is computed
    in 2D (xy), extra dimensions (e.g. z) are preserved unchanged.

    Parameters
    ----------
    points : np.ndarray (n, V, D)
        Coordinates for n curves, each with V vertices. D is 2 or 3.
    dists : np.ndarray (n, V)
        Per-vertex offset distances.

    Returns
    -------
    np.ndarray (n, V, D)
        Offset coordinates.
    """
    xy = points[:, :, :2]

    # Compute directions and normals
    dirs = normalize_vectors(xy[:, 1:] - xy[:, :-1])  # (n, V-1, 2)
    normals = np.stack([-dirs[:, :, 1], dirs[:, :, 0]], axis=2)  # (n, V-1, 2)

    # Start points
    start = xy[:, 0] + dists[:, 0:1] * normals[:, 0]  # (n, 2)

    # End points
    end = xy[:, -1] + dists[:, -1:] * normals[:, -1]  # (n, 2)

    # Interior points: average adjacent normals with clamped miter scaling
    n1 = normals[:, :-1]  # (n, V-2, 2)
    n2 = normals[:, 1:]   # (n, V-2, 2)
    n_avg = normalize_vectors(n1 + n2)

    dot_product = np.sum(n_avg * n1, axis=2)  # (n, V-2)
    scale = dists[:, 1:-1] / np.clip(dot_product, 0.5, None)
    interior = xy[:, 1:-1] + n_avg * scale[:, :, np.newaxis]  # (n, V-2, 2)

    result = points.copy()
    result[:, 0, :2] = start
    result[:, 1:-1, :2] = interior
    result[:, -1, :2] = end

    return result

def offset_curve(coords, distances):
    """
    Compute parallel offset curves from a single geometry at N uniform offsets.

    Parameters
    ----------
    coords : np.ndarray (V, D)
        Single coordinate array. D is 2 or 3 (xy or xyz).
    distances : array-like of float
        1D array (N,) of uniform offset distances. Positive offsets left, negative right.

    Returns
    -------
    np.ndarray (N, V, D)
        Offset coordinate arrays.
    """
    distances = np.asarray(distances, dtype=np.float64)

    points = np.broadcast_to(coords, (distances.shape[0], coords.shape[0], coords.shape[1]))
    dist_full = np.broadcast_to(distances[:, np.newaxis], (distances.shape[0], coords.shape[0]))

    return compute_offset_2d(points, dist_full)

def offset_curves(coords_list, distances):
    """
    Compute parallel offset curves from N geometries at N uniform offsets.

    Parameters
    ----------
    coords_list : list of np.ndarray
        List of N coordinate arrays with varying vertex counts, each (V_i, D).
        D is 2 or 3 (xy or xyz).
    distances : array-like of float
        1D array (N,) of uniform offset distances. Positive offsets left, negative right.

    Returns
    -------
    np.ndarray of np.ndarray
        Array of N coordinate arrays, each with the same shape as the corresponding input.
    """
    distances = np.asarray(distances, dtype=np.float64)
    assert len(coords_list) == distances.shape[0], "Number of coordinate arrays must match number of distances"
    if distances.size == 0:
        return np.array([], dtype=object)

    lengths = np.array([len(c) for c in coords_list])
    max_len = lengths.max()

    # Pad to uniform shape by repeating last vertex (preserves correct normals at boundaries)
    padded = np.empty((distances.shape[0], max_len, coords_list[0].shape[1]))
    for i, c in enumerate(coords_list):
        padded[i, :len(c)] = c
        padded[i, len(c):] = c[-1]

    dist_full = np.broadcast_to(distances[:, np.newaxis], (distances.shape[0], max_len))
    offset = compute_offset_2d(padded, dist_full)

    results = np.empty(distances.shape[0], dtype=object)
    for i in range(distances.shape[0]):
        results[i] = offset[i, :lengths[i]]

    return results

def offset_points(coords, distances):
    """
    Compute parallel offset curves with per-vertex variable offset.

    Parameters
    ----------
    coords : np.ndarray (V, D)
        Single coordinate array. D is 2 or 3 (xy or xyz).
    distances : np.ndarray (N, V)
        Per-vertex offset distances for each of N output curves.
        Positive offsets left, negative right.

    Returns
    -------
    np.ndarray (N, V, D)
        Offset coordinate arrays.
    """
    distances = np.asarray(distances, dtype=np.float64)

    points = np.broadcast_to(coords, (distances.shape[0], coords.shape[0], coords.shape[1]))

    return compute_offset_2d(points, distances)

def ensure_point(geometry):
    """
    Ensure that geometry is a shapely Point.
    :param geometry: shapely geometry
    :return: shapely Point
    """
    if isinstance(geometry, shapely.Point):
        return geometry
    else:
        # return first Point from geometry
        warnings.warn(f"{geometry} converted to a single point.")
        return shapely.points(shapely.get_coordinates(geometry, include_z=True)[0])

def ensure_points(arr):
    """
    Ensure that array contains only shapely Point geometries.
    :param arr: array of shapely geometries
    :return: array of shapely Points
    """
    return np.fromiter(map(ensure_point, arr), dtype=object)

def calculate_cross_track_error(linestrings, points):
    """
    Calculate signed cross track error (perpendicular distance from point to linestring).
    Works on both single geometries and arrays.

    :param linestrings: shapely linestring or array of linestrings
    :param points: shapely point or array of points
    :return: cross track error(s) - positive = left of line, negative = right
    """
    distances = shapely.line_locate_point(linestrings, points)

    # if distance is negative it is measured from the end of the linestring in reverse direction
    pos1 = shapely.line_interpolate_point(linestrings, np.maximum(0, distances - 0.1))
    pos2 = shapely.line_interpolate_point(linestrings, distances + 0.1)

    pos1_coords = shapely.get_coordinates(pos1)
    pos2_coords = shapely.get_coordinates(pos2)
    point_coords = shapely.get_coordinates(points)

    dx = pos2_coords[..., 0] - pos1_coords[..., 0]
    dy = pos2_coords[..., 1] - pos1_coords[..., 1]
    segment_length = np.hypot(dx, dy)

    numerator = dx * (pos1_coords[..., 1] - point_coords[..., 1]) - (pos1_coords[..., 0] - point_coords[..., 0]) * dy
    return numerator / segment_length

def calculate_linestring_heading_at_distance(linestrings, distances, delta=0.1):
    """
    Calculate heading of linestring(s) at specified distance(s) along the line.
    Works on both single geometries and arrays.

    :param linestrings: shapely linestring or array of linestrings
    :param distances: distance(s) along the linestring(s) in meters
    :param delta: half-window distance for heading calculation, applied both backward and forward (default 0.1m)
    :return: heading(s) in radians
    """
    back_points = shapely.line_interpolate_point(linestrings, np.maximum(0, distances - delta))
    fwd_points = shapely.line_interpolate_point(linestrings, distances + delta)

    back_coords = shapely.get_coordinates(back_points)
    fwd_coords = shapely.get_coordinates(fwd_points)

    return np.arctan2(fwd_coords[..., 1] - back_coords[..., 1],
                      fwd_coords[..., 0] - back_coords[..., 0])

def side_of_linestring(linestring, position):
    """
    Check on which side of the linestring the position is.
    :param linestring: shapely linestring
    :param position: shapely point
    :return: positive for right side, negative for left side
    """

    distance_from_path_start = linestring.project(position)

    # if distance is negative it is measured from the end of the linestring in reverse direction
    pos1 = linestring.interpolate(max(0, distance_from_path_start - 0.1))
    pos2 = linestring.interpolate(distance_from_path_start + 0.1)

    numerator = (pos2.x - pos1.x) * (pos1.y - position.y) - (pos1.x - position.x) * (pos2.y - pos1.y)
    return numerator

def get_boundary_points(geometry, max_segment_length=0.5):
    """
    Get densified boundary/exterior points from a geometry (without interior grid points).
    :param geometry: shapely geometry (Polygon, MultiPolygon, LineString, Point, or GeometryCollection)
    :param max_segment_length: maximum segment length for densification
    :return: Nx2 numpy array of coordinates
    """
    all_coords_list = []

    # handle MultiPolygon, GeometryCollection, or single geometry
    geoms = list(geometry.geoms) if hasattr(geometry, 'geoms') else [geometry]

    for geom in geoms:
        if geom.is_empty:
            continue

        if hasattr(geom, 'exterior'):
            # Polygon - get densified exterior points only
            densified_exterior = shapely.segmentize(geom.exterior, max_segment_length=max_segment_length)
            all_coords_list.append(shapely.get_coordinates(densified_exterior))
        else:
            # LineString or Point - get densified coordinates
            densified_geom = shapely.segmentize(geom, max_segment_length=max_segment_length)
            all_coords_list.append(shapely.get_coordinates(densified_geom))

    if all_coords_list:
        return np.vstack(all_coords_list)
    return np.empty((0, 2))

def linesubstring(line, start_dist, end_dist):
    """
    Extract portion of a LineString between two distances.
    Numpy equivalent of shapely.ops.substring.
    :param line: shapely LineString
    :param start_dist: distance along the line to start the substring
    :param end_dist: distance along the line to end the substring
    :return: shapely LineString substring
    """
    coords = np.array(line.coords)

    # Compute cumulative distances without np.insert
    diffs = coords[1:] - coords[:-1]
    distances = np.empty(len(coords))
    distances[0] = 0.0
    np.cumsum(np.sqrt(np.sum(diffs * diffs, axis=1)), out=distances[1:])
    total_length = distances[-1]

    # Clamp with Python builtins (avoids numpy override dispatch overhead)
    start_dist = max(0.0, min(float(start_dist), total_length))
    end_dist = max(0.0, min(float(end_dist), total_length))

    # Find indices of vertices strictly between start and end
    idx_start = np.searchsorted(distances, start_dist, side='right')
    idx_end = np.searchsorted(distances, end_dist, side='left')

    # Interpolate start point
    if start_dist <= 0.0:
        start_point = coords[0]
    elif start_dist >= total_length:
        start_point = coords[-1]
    else:
        i = idx_start
        t = (start_dist - distances[i - 1]) / (distances[i] - distances[i - 1])
        start_point = coords[i - 1] + t * (coords[i] - coords[i - 1])

    # Interpolate end point
    if end_dist <= 0.0:
        end_point = coords[0]
    elif end_dist >= total_length:
        end_point = coords[-1]
    else:
        j = min(idx_end, len(coords) - 1)
        if distances[j] == distances[j - 1]:
            end_point = coords[j]
        else:
            t = (end_dist - distances[j - 1]) / (distances[j] - distances[j - 1])
            end_point = coords[j - 1] + t * (coords[j] - coords[j - 1])
    
    coords_to_return = np.concatenate([start_point[np.newaxis], coords[idx_start:idx_end], end_point[np.newaxis]])

    return shapely.linestrings(coords_to_return)
