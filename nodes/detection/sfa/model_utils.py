import numpy as np
import torch
from sfa_utils import _sigmoid, decode, post_processing

def get_filtered_lidar(lidar, minX, maxX, minY, maxY, minZ, maxZ, labels=None):

    # Remove the point out of range x,y,z
    mask = np.where((lidar[:, 0] >= minX) & (lidar[:, 0] <= maxX) &
                    (lidar[:, 1] >= minY) & (lidar[:, 1] <= maxY) &
                    (lidar[:, 2] >= minZ) & (lidar[:, 2] <= maxZ))
    lidar = lidar[mask]
    lidar[:, 2] = lidar[:, 2] - minZ

    if labels is not None:
        label_x = (labels[:, 1] >= minX) & (labels[:, 1] < maxX)
        label_y = (labels[:, 2] >= minY) & (labels[:, 2] < maxY)
        label_z = (labels[:, 3] >= minZ) & (labels[:, 3] < maxZ)
        mask_label = label_x & label_y & label_z
        labels = labels[mask_label]
        return lidar, labels
    else:
        return lidar


def makeBEVMap(PointCloud_, BEV_HEIGHT, BEV_WIDTH, DISCRETIZATION, minZ, maxZ):
    Height = BEV_HEIGHT + 1
    Width = BEV_WIDTH + 1

    # Discretize Feature Map
    PointCloud = np.copy(PointCloud_)
    PointCloud[:, 0] = np.int_(np.floor(PointCloud[:, 0] / DISCRETIZATION))
    PointCloud[:, 1] = np.int_(np.floor(PointCloud[:, 1] / DISCRETIZATION) + Width / 2)

    # sort-3times
    sorted_indices = np.lexsort((-PointCloud[:, 2], PointCloud[:, 1], PointCloud[:, 0]))
    PointCloud = PointCloud[sorted_indices]
    _, unique_indices, unique_counts = np.unique(PointCloud[:, 0:2], axis=0, return_index=True, return_counts=True)
    PointCloud_top = PointCloud[unique_indices]

    # Height Map, Intensity Map & Density Map
    heightMap = np.zeros((Height, Width))
    intensityMap = np.zeros((Height, Width))
    densityMap = np.zeros((Height, Width))

    # some important problem is image coordinate is (y,x), not (x,y)
    max_height = float(np.abs(maxZ - minZ))
    heightMap[np.int_(PointCloud_top[:, 0]), np.int_(PointCloud_top[:, 1])] = PointCloud_top[:, 2] / max_height

    normalizedCounts = np.minimum(1.0, np.log(unique_counts + 1) / np.log(64))
    intensityMap[np.int_(PointCloud_top[:, 0]), np.int_(PointCloud_top[:, 1])] = PointCloud_top[:, 3]
    densityMap[np.int_(PointCloud_top[:, 0]), np.int_(PointCloud_top[:, 1])] = normalizedCounts

    RGB_Map = np.zeros((3, Height - 1, Width - 1))
    RGB_Map[2, :, :] = densityMap[:BEV_HEIGHT, :BEV_WIDTH]  # r_map
    RGB_Map[1, :, :] = heightMap[:BEV_HEIGHT, :BEV_WIDTH]  # g_map
    RGB_Map[0, :, :] = intensityMap[:BEV_HEIGHT, :BEV_WIDTH]  # b_map

    return RGB_Map


def do_detect(device, bound_size_x, bound_size_y, BEV_WIDTH, BEV_HEIGHT, K, num_classes, down_ratio, peak_thresh, model, bevmap, is_front):
    if not is_front:
        bevmap = torch.flip(bevmap, [1, 2])

    input_bev_maps = bevmap.unsqueeze(0).to(device, non_blocking=True).float()
    # not removing this call at the moment. Not sure if needed or not. this function calls torch.cuda.synchronize()
    # t1 = time_synchronized()
    outputs = model(input_bev_maps)
    outputs['hm_cen'] = _sigmoid(outputs['hm_cen'])
    outputs['cen_offset'] = _sigmoid(outputs['cen_offset'])
    # detections size (batch_size, K, 10)
    detections = decode(outputs['hm_cen'], outputs['cen_offset'], outputs['direction'], outputs['z_coor'],
                        outputs['dim'], K=K)
    detections = detections.cpu().numpy().astype(np.float32)
    # print(detections)
    detections = post_processing(detections, bound_size_x, bound_size_y, BEV_WIDTH, BEV_HEIGHT, num_classes, down_ratio, peak_thresh)
    # not removing this call at the moment. Not sure if needed or not. this function calls torch.cuda.synchronize()
    # t2 = time_synchronized()
    # Inference speed
    # fps = 1 / (t2 - t1)

    return detections[0], bevmap

def get_num_parameters(model):
    """Count number of trained parameters of the model"""
    if hasattr(model, 'module'):
        num_parameters = sum(p.numel() for p in model.module.parameters() if p.requires_grad)
    else:
        num_parameters = sum(p.numel() for p in model.parameters() if p.requires_grad)

    return num_parameters





