import numpy as np
import shapely
import mapbox_earcut as earcut
from geometry_msgs.msg import Point

from autoware_mini.shapely import offset_curve

def triangulate_polygon(polygon_points):
    """
    Triangulates a polygon with earcut algorithm

    Parameters:
    - polygon_points: Exterior points of the polygon

    Returns:
    - List of triangle points. Every set of 3 points is treated as a triangle
    """
    coords = np.asarray(polygon_points, dtype=np.float32)
    indexes = np.array([len(coords)], dtype=np.uint32)
    triangles = earcut.triangulate_float32(coords[:, :2], indexes)

    triangle_points = [Point(x=x, y=y, z=z) for x, y, z in coords[triangles].tolist()]
    return triangle_points

def triangulate_linestring(linestring, width, z_offset=0):
    """
    Triangulates a polygon defined by centerline and width using offset curves

    Parameters:
    - linestring: Shapely LineString centerline
    - width: Width of the polygon
    - z_offset: Offset for z-coordinates

    Returns:
    - List of triangle points. Every set of 3 points is treated as a triangle
    """

    coords = shapely.get_coordinates(linestring, include_z=True)
    left, right = offset_curve(coords, [width / 2, -width / 2])

    left[:, 2] += z_offset
    right[:, 2] += z_offset

    # Build triangle strip: two triangles per quad segment
    n = len(left) - 1
    triangles = np.empty((n * 6, 3))
    triangles[0::6] = left[:-1]
    triangles[1::6] = right[:-1]
    triangles[2::6] = left[1:]
    triangles[3::6] = right[:-1]
    triangles[4::6] = right[1:]
    triangles[5::6] = left[1:]

    return [Point(x=x, y=y, z=z) for x, y, z in triangles.tolist()]
