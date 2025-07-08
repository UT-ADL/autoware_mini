import math
import cupy as cp
from cupyx.scipy.ndimage import convolve
    
class NaiveGroundDetectorFast:
    def __init__(self, min_x, max_x, min_y, max_y, cell_size, tolerance, filter_size, filter_iterations):
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y
        self.cell_size = cell_size
        self.tolerance = tolerance
        self.filter_size = filter_size
        self.filter_iterations = filter_iterations

        self.width = int(math.ceil((self.max_x - self.min_x) / self.cell_size))
        self.height = int(math.ceil((self.max_y - self.min_y) / self.cell_size))
        self.cols = cp.empty((self.width, self.height), dtype=cp.float32)
        self.kernel = cp.ones((self.filter_size, self.filter_size), dtype=cp.float32) / self.filter_size**2

    def detect_ground(self, pointcloud):

        # convert x and y coordinates into indexes
        xi = ((pointcloud[:, 0] - self.min_x) / self.cell_size).astype(cp.int32)
        yi = ((pointcloud[:, 1] - self.min_y) / self.cell_size).astype(cp.int32)
        zi = pointcloud[:, 2]

        # write minimum height for each cell to cols
        # thanks to sorting in descending order,
        # the minimum value will overwrite previous values
        self.cols[...] = cp.nan
        idx = cp.argsort(-zi)
        self.cols[xi[idx], yi[idx]] = zi[idx]
        
        # bring cell minimum lower, if all cells around it are lower
        for _ in range(self.filter_iterations):
            mask_gpu = cp.isnan(self.cols)
            self.cols[mask_gpu] = 0
            cols_filtered = convolve(self.cols, self.kernel, mode='nearest') / convolve((~mask_gpu).astype(cp.float32), self.kernel, mode='nearest')
            cp.fmin(self.cols, cols_filtered, out=self.cols)

        # filter out closest points to minimum point up to some tolerance
        ground_mask = (zi <= (self.cols[xi, yi] + self.tolerance))

        # return ground mask
        return ground_mask