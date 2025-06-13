import numpy as np
import shapely
import shapely.ops
import mapbox_earcut as earcut
from geometry_msgs.msg import Point

def triangulate_polygon(polygon_points):
    """
    Triangulates a polygon with earcut algorithm

    Parameters:
    - polygon_points: Exterior points of the polygon

    Returns:
    - List of triangle points. Every set of 3 points is treated as a triangle
    """
    coords = np.array(polygon_points, dtype=np.float32)
    triangles = earcut.triangulate_float32(coords[:, :2], [len(coords)])

    triangle_points = [Point(x=x, y=y, z=z) for x, y, z in coords[triangles]]
    return triangle_points

def triangulate_linestring(linestring, width, z_offset=0):
    """
    Triangulates a polygon defined by centerline and width using earcut algorithm

    Parameters:
    - linestring: Shapely LineString centerline
    - z_offset: Offset for z-coordinates

    Returns:
    - List of triangle points. Every set of 3 points is treated as a triangle
    """

    buffer = linestring.buffer(width / 2, cap_style="flat")
    number_of_exterior_points = len(buffer.exterior.coords)
    indexes = [number_of_exterior_points]
    coords = list(buffer.exterior.coords)

    if buffer.interiors:
        number_of_points_in_each_hole = [len(hole.coords) for hole in buffer.interiors]
        # indexes indicate the end of each ring, first ring is exterior and the rest are holes.
        # the last index is the end of the last hole (or exterior if no holes present)
        indexes = np.concatenate([indexes, (number_of_exterior_points + np.cumsum(number_of_points_in_each_hole))])
        coords = np.concatenate([coords] + [hole.coords for hole in buffer.interiors])

    triangles = earcut.triangulate_float32(coords, indexes)

    # Extract z coordinates from linestring for each triangle point
    points = shapely.points(coords)
    distances = linestring.project(points)
    points_on_linestring = linestring.interpolate(distances)

    triangle_points = [Point(x=coords[i][0], y=coords[i][1], z=points_on_linestring[i].z + z_offset) for i in triangles]

    return triangle_points

def triangulate_path(linestring, width, split_length=100, z_offset=0):
    """
    Triangulates a polygon defined by centerline and width using earcut algorithm. Splits the linestring into shorter segments.

    Parameters:
    - linestring: Shapely LineString centerline
    - split_length: Maximum line segment length
    - z_offset: Offset for z-coordinates

    Returns:
    - List of triangle points. Every set of 3 points is treated as a triangle
    """

    segments = split_line_fixed_length(linestring, split_length)

    triangle_points = []
    for segment in segments:
        points = triangulate_linestring(segment, width, z_offset)
        triangle_points.extend(points)

    return triangle_points

def split_line_fixed_length(line, segment_length):
    """
    Splits a LineString into fixed-length segments

    Parameters:
    - line: Shapely LineString
    - segment_length: Desired length of each segment

    Returns:
    - List of LineString segments
    """
    line_length = line.length

    # Return if line already smaller than the desired segment length
    if line_length <= segment_length:
        return [line]
    
    split_lines = []
    split_distances = np.arange(0, line_length, segment_length)

    for i in range(len(split_distances) - 1):
        segment = shapely.ops.substring(line, split_distances[i], split_distances[i+1])
        split_lines.append(segment)

    # Last segment is usually shorter than the desired length
    split_lines.append(shapely.ops.substring(line, split_distances[-1], line_length))

    return split_lines