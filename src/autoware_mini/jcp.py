import math

import cv2
import numpy as np
from numba import njit


@njit
def RangeProjection(
    *,
    pcl: np.ndarray,
    per_point_col_idx: np.ndarray,
    per_point_row_idx: np.ndarray,
    per_point_range_xy_m: np.ndarray,
    range_image_: np.ndarray,
    region_: np.ndarray,
    region_minz_: np.ndarray,
    cloud_index_: np.ndarray,
    range_img_width: int,
    range_img_height: int,
    delta_R: float,
    length_: int,
    min_range: float,
    max_range: float,
):
    for i in range(pcl.shape[0]):
        col = per_point_col_idx[i]
        range_m = per_point_range_xy_m[i]
        ind = per_point_row_idx[i]
        if (
            (range_m < min_range)
            or (range_m > max_range)
            or (col < 0)
            or (col > range_img_width)  # looks like off by one error here?
            or (ind < 0)
            or (ind > range_img_height)  # also here
            or (
                (
                    (pcl[i, 0] < 3 and pcl[i, 0] > -2)
                    and (pcl[i, 1] < 1.5 and pcl[i, 1] > -1.5)
                )
            )
            or (pcl[i, 2] < -3 and pcl[i, 2] > 1)
        ):
            continue

        region = int((range_m - min_range) / delta_R)

        region_index = col * length_ + region
        index = col * range_img_height + ind
        range_image_[ind, col] = np.array([0, 255, 0], dtype=range_image_.dtype)
        region_minz_[region_index] = min(region_minz_[region_index], pcl[i, 2])
        region_[ind, col] = region
        cloud_index_[index] = i

    return (range_image_, region_minz_, region_, cloud_index_)


@njit
def RECM(
    *,
    pcl: np.ndarray,
    range_image_: np.ndarray,
    region_: np.ndarray,
    region_minz_: np.ndarray,
    cloud_index_: np.ndarray,
    range_img_width: int,
    range_img_height: int,
    delta_R: float,
    length_: int,
    sensor_height: float,
    th_g_: float,
    sigma_: float,
):
    flag = False
    for i in range(region_minz_.shape[0]):
        if i % length_ == 0:
            flag = False
            region_minz_[i] = min(region_minz_[i], sensor_height + th_g_)
            continue
        else:
            if (i + 1) % length_ == 0:
                continue
            if region_minz_[i] == 100 and not flag:
                region_minz_[i] = sensor_height + th_g_
                continue
            if region_minz_[i] == 100 and flag:  # == comparison with float????
                region_minz_[i] = region_minz_[i - 1]
            flag = True
            if (
                abs(region_minz_[i] - region_minz_[i - 1]) > 0.5
                and abs(region_minz_[i] - region_minz_[i + 1]) > 0.5
            ):
                region_minz_[i] = (region_minz_[i - 1] + region_minz_[i + 1]) / 2

    pre_th = 0.0
    # float region_num = 0 # unused for some reason?
    for i in range(region_minz_.shape[0]):
        if i % length_ == 0:
            pre_th = min(region_minz_[i], float(sensor_height))
        else:
            region_minz_[i] = min(
                region_minz_[i], pre_th + delta_R * math.tan(sigma_ * np.pi / 180)
            )
            pre_th = region_minz_[i]

    for i in range(range_img_width):
        for j in range(range_img_height):
            index = i * range_img_height + j
            region_i = region_[j, i]
            th_height = region_minz_[i * length_ + region_i]
            pt_idx = cloud_index_[index]
            if pt_idx == -1:
                continue
            if pcl[pt_idx, 2] >= (th_height + th_g_):
                range_image_[j, i] = np.array([0, 0, 255], dtype=range_image_.dtype)

    return (range_image_, region_minz_)


@njit
def JCP(
    *,
    pcl: np.ndarray,
    range_image_: np.ndarray,
    relevant_row_col_indices: np.ndarray,
    cloud_index_: np.ndarray,
    range_img_width: int,
    range_img_height: int,
):
    neighborx_ = (
        -2,
        -1,
        0,
        1,
        2,
        -2,
        -1,
        0,
        1,
        2,
        -2,
        -1,
        1,
        2,
        -2,
        -1,
        0,
        1,
        2,
        -2,
        -1,
        0,
        1,
        2,
    )
    neighbory_ = (
        -2,
        -2,
        -2,
        -2,
        -2,
        -1,
        -1,
        -1,
        -1,
        -1,
        0,
        0,
        0,
        0,
        1,
        1,
        1,
        1,
        1,
        2,
        2,
        2,
        2,
        2,
    )

    for rc_idx in range(relevant_row_col_indices.shape[0]):
        row_col_id = relevant_row_col_indices[rc_idx]
        cloud_id = row_col_id[1] * range_img_height + row_col_id[0]
        D = np.zeros(24)
        mask = np.zeros(24, dtype=np.int32)
        sumD = 0.0
        for i in range(24):
            nx = neighborx_[i] + row_col_id[1]
            ny = neighbory_[i] + row_col_id[0]

            ncloud_id = nx * range_img_height + ny

            if (
                nx < 0
                or nx >= range_img_width
                or ny < 0
                or ny >= range_img_height
                or cloud_index_[ncloud_id] == -1
            ):
                D[i] = 0.0
                sumD += D[i]
                mask[i] = -1
                continue
            range_diff = np.linalg.norm(
                pcl[cloud_index_[cloud_id]] - pcl[cloud_index_[ncloud_id]]
            )
            if range_diff > 3:
                D[i] = 0
                sumD += D[i]
            else:
                D[i] = math.exp(-5 * range_diff)
                sumD += D[i]
            if (
                range_image_[ny, nx] == np.array((255, 0, 0), dtype=range_image_.dtype)
            ).all():
                mask[i] = 2
            elif (
                range_image_[ny, nx] == np.array((0, 255, 0), dtype=range_image_.dtype)
            ).all():
                mask[i] = 1
            elif (
                range_image_[ny, nx] == np.array((0, 0, 255), dtype=range_image_.dtype)
            ).all():
                mask[i] = 0

        # W = D / sumD
        W = D / max(sumD, 1e-6)

        score_r = 0.0
        score_g = 0.0
        for i in range(D.shape[0]):
            if mask[i] == 0:
                score_r += W[i]
            elif mask[i] == 1:
                score_g += W[i]

        if score_r > score_g:
            # obstacle: bgr
            range_image_[row_col_id[0], row_col_id[1]] = np.array(
                (0, 0, 255), dtype=range_image_.dtype
            )
        else:
            # ground: bgr
            range_image_[row_col_id[0], row_col_id[1]] = np.array(
                (0, 255, 0), dtype=range_image_.dtype
            )
    return range_image_


def JPCGroundRemove(
    *,
    pcl: np.ndarray,
    range_img_width: int,
    range_img_height: int,
    sensor_height: float,
    delta_R: float,
):
    assert pcl.shape[-1] == 3, pcl.shape
    min_range = 3.0
    max_range = 70.0
    th_g = 0.3
    sigma_ = 7.0
    length_ = int((max_range - min_range) / delta_R)

    range_image_ = np.zeros((range_img_height, range_img_width, 3), dtype=np.uint8)
    region_ = np.zeros((range_img_height, range_img_width), dtype=np.uint8)
    region_minz_ = 100 * np.ones((range_img_width * length_))
    cloud_index_ = -np.ones((range_img_width * range_img_height), dtype=int)

    per_point_col_angle_rad = np.arctan2(pcl[:, 1], pcl[:, 0])
    per_point_col_angle_rad = np.where(
        pcl[:, 1] < 0, per_point_col_angle_rad + 2 * np.pi, per_point_col_angle_rad
    )
    per_point_range_xy_m = np.linalg.norm(pcl[:, :2], axis=-1)

    arcsin_input = pcl[:, 2] / np.maximum(per_point_range_xy_m, 1e-6)
    if (np.abs(arcsin_input) > 1.0).any():
        has_bad_arcsin_input = np.abs(arcsin_input) > 1.0
        arcsin_input[has_bad_arcsin_input] = np.clip(
            arcsin_input[has_bad_arcsin_input],
            a_min=-1.0,
            a_max=1.0,
        )
        if has_bad_arcsin_input.sum() > 0.01 * pcl.shape[0]:
            print(
                f"bad arsin input found for more than 0.1 % of points: {has_bad_arcsin_input.sum()} points"
            )

    elevation_rad = np.arcsin(arcsin_input)
    valid_eles = np.isfinite(elevation_rad)
    max_ele = np.max(elevation_rad[valid_eles])
    min_ele = np.min(elevation_rad[valid_eles])
    per_point_row_idx = np.clip(
        (range_img_height * (elevation_rad - min_ele) / (max_ele - min_ele)).astype(
            np.int32
        ),
        a_min=0,
        a_max=range_img_height - 1,
    )
    per_point_col_idx = (
        (range_img_width - 1) * (per_point_col_angle_rad * 180.0 / np.pi) / 360.0
    ).astype(np.int32)

    range_image_, region_minz_, region_, cloud_index_ = RangeProjection(
        pcl=pcl,
        per_point_col_idx=per_point_col_idx,
        per_point_row_idx=per_point_row_idx,
        per_point_range_xy_m=per_point_range_xy_m,
        range_image_=range_image_,
        region_=region_,
        region_minz_=region_minz_,
        cloud_index_=cloud_index_,
        range_img_width=range_img_width,
        range_img_height=range_img_height,
        delta_R=delta_R,
        length_=length_,
        min_range=min_range,
        max_range=max_range,
    )

    range_image_, region_minz_ = RECM(
        pcl=pcl,
        range_image_=range_image_,
        region_=region_,
        region_minz_=region_minz_,
        cloud_index_=cloud_index_,
        range_img_width=range_img_width,
        range_img_height=range_img_height,
        delta_R=delta_R,
        length_=length_,
        th_g_=th_g,
        sensor_height=sensor_height,
        sigma_=sigma_,
    )

    range_img_b, range_img_g, range_image_r = np.split(range_image_, 3, axis=-1)
    dilationi_kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (5, 5))
    range_image_r = cv2.dilate(
        np.squeeze(range_image_r, axis=-1), dilationi_kernel, iterations=1
    )[..., None]
    range_image_ = np.concatenate([range_img_b, range_img_g, range_image_r], axis=-1)

    relevance_mask = (
        range_image_ == np.array([0, 255, 255], dtype=range_image_.dtype)
    ).all(axis=-1)

    relevant_row_idxs, relevant_col_idxs = np.where(relevance_mask)
    has_valid_cloud_idx = (
        cloud_index_[relevant_row_idxs * range_img_height + relevant_col_idxs] != -1
    )
    range_image_[
        relevant_row_idxs[has_valid_cloud_idx], relevant_col_idxs[has_valid_cloud_idx]
    ] = np.array([255, 0, 0], dtype=range_image_.dtype)
    range_image_[
        relevant_row_idxs[~has_valid_cloud_idx], relevant_col_idxs[~has_valid_cloud_idx]
    ] = np.array([0, 0, 255], dtype=range_image_.dtype)

    relevant_row_col_indices = np.stack(
        [
            relevant_row_idxs[has_valid_cloud_idx],
            relevant_col_idxs[has_valid_cloud_idx],
        ],
        axis=-1,
    )

    range_image_ = JCP(
        pcl=pcl,
        range_image_=range_image_,
        relevant_row_col_indices=relevant_row_col_indices,
        cloud_index_=cloud_index_,
        range_img_width=range_img_width,
        range_img_height=range_img_height,
    )

    # from PIL import Image

    # data = Image.fromarray(range_image_[:, :, [2, 1, 0]])
    # data.save("groundseg.png")
    is_ground = (range_image_ == np.array([0, 255, 0], dtype=np.uint8)).all(axis=-1)
    per_point_is_ground = is_ground[per_point_row_idx, per_point_col_idx]
    return per_point_is_ground


# Algo:
# 1. RangeProjection
#   -> region_ [range_img_height, range_img_width, 1]
#   -> region_minz_ [range_img_height * length_]
#   -> cloud_index_ [range_img_height * range_img_width]
#   -> range_image [range_img_height, range_img_width, 3]: color img
# 2. RECM
# 3. JCP
