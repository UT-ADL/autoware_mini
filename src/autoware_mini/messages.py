import numpy as np
from autoware_mini.msg import LocalPath


def float32_multiarray_to_numpy(multiarray):
    dims = tuple(map(lambda x: x.size, multiarray.layout.dim))
    data = multiarray.data[multiarray.layout.data_offset:]
    return np.array(data, dtype=np.float32).reshape(dims)


def path_to_local_path(path_msg):
    """Convert a Path message to a LocalPath message with default values."""
    return LocalPath(header=path_msg.header, waypoints=path_msg.waypoints)
