import shapely
import numpy as np

def offset_curve(linestring, offset_distance):

    coords = np.array(linestring.coords)

    if len(coords) < 2:
        raise ValueError("Need at least two points to compute an offset")

    # Compute segment directions
    vectors = np.diff(coords[:,:2], axis=0)
    norms = np.linalg.norm(vectors, axis=1, keepdims=True)
    segment_dirs = np.divide(vectors, norms, where=norms != 0)

    # Average directions for each point
    dirs = np.vstack([segment_dirs[0], (segment_dirs[:-1] + segment_dirs[1:])/2, segment_dirs[-1]])

    # Perpendicular vectors
    perps = np.fliplr(dirs)
    perps[:, 0] *= -1

    # Offset coordinates
    coords[:,:2] += offset_distance * perps

    return shapely.linestrings(coords)