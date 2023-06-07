#!/usr/bin/env python3

import rospy
import numpy as np
import cv2

from tf2_ros import TransformListener, Buffer, TransformException
from numpy.lib.recfunctions import structured_to_unstructured
from ros_numpy import numpify

from sensor_msgs.msg import PointCloud2
from std_msgs.msg import ColorRGBA
from autoware_msgs.msg import DetectedObjectArray, DetectedObject

import onnxruntime

from helpers.geometry import get_orientation_from_heading
from helpers.detection import create_hull
from helpers.transform import transform_pose


LIGHT_BLUE = ColorRGBA(0.5, 0.5, 1.0, 0.8)

## Model params
BEV_HEIGHT = 608  # number of rows of the bev image
BEV_WIDTH = 608  # number of columns of the bev image
DOWN_RATIO = 4
NUM_CLASSES = 3  # number of classes to detect
CLASS_NAMES = {0: "pedestrian", 1: "car", 2: "cyclist"}

class SFADetector:
    def __init__(self):
        # Params
        self.onnx_path = rospy.get_param("~onnx_path")  # path of the trained model

        self.MIN_FRONT_X = rospy.get_param("~min_front_x")  # minimum distance on lidar +ve x-axis to process points at (front_detections)
        self.MAX_FRONT_X = rospy.get_param("~max_front_x")  # maximum distance on lidar +ve x-axis to process points at (back detections)
        self.MIN_BACK_X = rospy.get_param("~min_back_x")  # maxmimum distance on lidar -ve x-axis to process points at
        self.MAX_BACK_X = rospy.get_param("~max_back_x")  # minimum distance on lidar -ve x-axis to process points at
        self.MIN_Y = rospy.get_param("~min_y")  # maxmimum distance on lidar -ve y-axis to process points at
        self.MAX_Y = rospy.get_param("~max_y")  # maxmimum distance on lidar +ve y-axis to process points at
        self.MIN_Z = rospy.get_param("~min_z")  # maxmimum distance on lidar -ve z-axis to process points at
        self.MAX_Z = rospy.get_param("~max_z")  # maxmimum distance on lidar +ve z-axis to process points at
        self.DISCRETIZATION = (self.MAX_FRONT_X - self.MIN_FRONT_X) / BEV_HEIGHT  # 3D world discretization to 2D image: distance encoded by 1 pixel
        self.BOUND_SIZE_X = self.MAX_FRONT_X - self.MIN_FRONT_X
        self.BOUND_SIZE_Y = self.MAX_Y - self.MIN_Y
        self.MAX_HEIGHT = np.abs(self.MAX_Z - self.MIN_Z)

        self.score_thresh = rospy.get_param("~score_thresh")  # score filter
        self.top_k = rospy.get_param("~top_k")  # number of top scoring detections to process
        self.output_frame = rospy.get_param("/detection/output_frame")  # transform detected objects from lidar frame to this frame
        self.transform_timeout = rospy.get_param('~transform_timeout') # transform timeout when waiting for transform to output frame

        self.model = self.load_onnx(self.onnx_path) # get onnx model

        rospy.loginfo("%s - loaded ONNX model file %s", rospy.get_name(), self.onnx_path)

        # transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        # Subscribers and Publishers
        self.detected_object_array_pub = rospy.Publisher('detected_objects', DetectedObjectArray, queue_size=1)
        rospy.Subscriber('points_raw', PointCloud2, self.pointcloud_callback, queue_size=1, buff_size=2*1024*1024)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def load_onnx(self, onnx_path):

        return onnxruntime.InferenceSession(onnx_path, providers=['CUDAExecutionProvider'])

    def pointcloud_callback(self, pointcloud):
        """
        pointcloud: raw lidar data points
        return: None - publish autoware DetectedObjects
        """
        try:
            # get the transform from lidar frame to output frame at the time when the poinctloud msg was published
            transform = self.tf_buffer.lookup_transform(self.output_frame, pointcloud.header.frame_id, pointcloud.header.stamp, rospy.Duration(self.transform_timeout))
        except (TransformException, rospy.ROSTimeMovedBackwardsException) as e:
            rospy.logwarn("%s - %s", rospy.get_name(), e)
            return

        # Unpack pointcloud2 msg ype to numpy array
        pcd_array = numpify(pointcloud)

        # Reshape the array into shape -> (num_points, 4). 4 corresponds to the fields -> x,y,z,intensity
        points = structured_to_unstructured(pcd_array[['x', 'y', 'z', 'intensity']], dtype=np.float32)

        detected_objects_array = DetectedObjectArray()
        detected_objects_array.header.stamp = pointcloud.header.stamp
        detected_objects_array.header.frame_id = self.output_frame

        final_detections = self.detect(points)
        detected_objects_array.objects = self.generate_autoware_objects(final_detections, pointcloud.header, transform)
        self.detected_object_array_pub.publish(detected_objects_array)

    def detect(self, points):

        front_points = self.get_filtered_points(points, is_front=True)
        front_bev_map = self.make_bev_map(front_points)
        back_points = self.get_filtered_points(points, is_front=False)
        back_bev_map = self.make_bev_map(back_points)
        back_bev_map = np.flip(back_bev_map, (1, 2))

        input_bev_maps = np.stack((front_bev_map, back_bev_map)).astype(np.float32)
        detections = self.do_detection(input_bev_maps)

        front_detections = self.post_processing(detections[0])
        back_detections = self.post_processing(detections[1])
        back_detections[:, 1:3] *= -1

        return np.concatenate((front_detections, back_detections), axis=0)

    def get_filtered_points(self, points, is_front=True):

        if is_front:
            # Remove the point out of range x,y,z
            mask = (points[:, 0] >= self.MIN_FRONT_X) & (points[:, 0] < self.MAX_FRONT_X) & \
                    (points[:, 1] >= self.MIN_Y) & (points[:, 1] < self.MAX_Y) & \
                    (points[:, 2] >= self.MIN_Z) & (points[:, 2] < self.MAX_Z) \

        else:
            mask = (points[:, 0] >= self.MIN_BACK_X) & (points[:, 0] < self.MAX_BACK_X) & \
                    (points[:, 1] >= self.MIN_Y) & (points[:, 1] < self.MAX_Y) & \
                    (points[:, 2] >= self.MIN_Z) & (points[:, 2] < self.MAX_Z) \

        points = points[mask]
        points[:, 2] -= self.MIN_Z

        return points

    def make_bev_map(self, pointcloud):

        # Discretize Feature Map
        pointcloud = np.copy(pointcloud)
        pointcloud[:, 0] = np.int_(np.floor(pointcloud[:, 0] / self.DISCRETIZATION))
        pointcloud[:, 1] = np.int_(np.floor(pointcloud[:, 1] / self.DISCRETIZATION) + BEV_WIDTH / 2)

        # sort-3times
        sorted_indices = np.lexsort((-pointcloud[:, 2], pointcloud[:, 1], pointcloud[:, 0]))
        pointcloud = pointcloud[sorted_indices]
        _, unique_indices, unique_counts = np.unique(pointcloud[:, 0:2], axis=0, return_index=True, return_counts=True)
        pointcloud_top = pointcloud[unique_indices]

        normalized_counts = np.minimum(1.0, np.log(unique_counts + 1) / np.log(64))

        hid_map = np.zeros((3, BEV_HEIGHT , BEV_WIDTH))
        hid_map[2, np.int_(pointcloud_top[:, 0]), np.int_(pointcloud_top[:, 1])] = normalized_counts # density map
        hid_map[1, np.int_(pointcloud_top[:, 0]), np.int_(pointcloud_top[:, 1])] = pointcloud_top[:, 2] / self.MAX_HEIGHT  # height map
        hid_map[0, np.int_(pointcloud_top[:, 0]), np.int_(pointcloud_top[:, 1])] = pointcloud_top[:, 3]  # intensity map

        return hid_map

    def do_detection(self, bevmaps):

        # unpacking onnx outputs
        hm_cen, cen_offset, directions, z_coors, dimensions = self.model.run([], {"input":bevmaps})
        hm_cen = self.sigmoid(hm_cen)
        cen_offset = self.sigmoid(cen_offset)

        # detections size (batch_size, K, 10)
        detections = self.decode(hm_cen, cen_offset, directions, z_coors, dimensions, self.top_k)

        return detections

    def sigmoid(self, x):
        # Apply the sigmoid function element-wise to the input array
        sig_x = 1 / (1 + np.exp(-x))
        # Clamp the output values to be within the range [1e-4, 1 - 1e-4]
        sig_x = np.clip(sig_x, a_min=1e-4, a_max=1 - 1e-4)

        return sig_x

    def decode(self, hm_cen, cen_offset, direction, z_coor, dim, K=40):
        batch_size, num_classes, height, width = hm_cen.shape
        hm_cen = self.nms(hm_cen)
        scores, inds, clses, ys, xs = self.topk(hm_cen, K=K)
        if cen_offset is not None:
            cen_offset = self.transpose_and_gather_feat(cen_offset, inds)
            cen_offset = cen_offset.reshape(batch_size, K, 2)
            xs = xs.reshape(batch_size, K, 1) + cen_offset[:, :, 0:1]
            ys = ys.reshape(batch_size, K, 1) + cen_offset[:, :, 1:2]
        else:
            xs = xs.reshape(batch_size, K, 1) + 0.5
            ys = ys.reshape(batch_size, K, 1) + 0.5

        direction = self.transpose_and_gather_feat(direction, inds)
        direction = direction.reshape(batch_size, K, 2)
        z_coor = self.transpose_and_gather_feat(z_coor, inds)
        z_coor = z_coor.reshape(batch_size, K, 1)
        dim = self.transpose_and_gather_feat(dim, inds)
        dim = dim.reshape(batch_size, K, 3)
        clses = clses.reshape(batch_size, K, 1).astype(np.float32)
        scores = scores.reshape(batch_size, K, 1)
        # (scores x 1, ys x 1, xs x 1, z_coor x 1, dim x 3, direction x 2, clses x 1)
        # (scores-0:1, ys-1:2, xs-2:3, z_coor-3:4, dim-4:7, direction-7:9, clses-9:10)
        # detections: [batch_size, K, 10]
        detections = np.concatenate([scores, xs, ys, z_coor, dim, direction, clses], axis=2)

        return detections

    def nms(self, heat, kernel=3):
        # Compute max pooling operation using OpenCV
        kernel = np.ones((kernel, kernel))
        hmax = np.zeros(heat.shape)

        # Apply cv2 dilate function on each channel individually
        for batch_num in range(heat.shape[0]):
            hmax[batch_num, 0, :, :] = cv2.dilate(heat[batch_num, 0, :, :], kernel, borderType=cv2.BORDER_CONSTANT, borderValue=-np.inf)
            hmax[batch_num, 1, :, :] = cv2.dilate(heat[batch_num, 1, :, :], kernel, borderType=cv2.BORDER_CONSTANT, borderValue=-np.inf)
            hmax[batch_num, 2, :, :] = cv2.dilate(heat[batch_num, 2, :, :], kernel, borderType=cv2.BORDER_CONSTANT, borderValue=-np.inf)
        # Compute keep mask
        keep = (hmax == heat).astype(np.float32)
        # Apply keep mask to input tensor
        return heat * keep

    def topk(self, scores, K=40):
        batch, cat, height, width = scores.shape
        scores_reshaped = scores.reshape(batch, cat, -1)
        topk_scores, topk_inds = self.get_topk(scores_reshaped, K)

        topk_inds = topk_inds % (height * width)
        topk_ys = np.floor_divide(topk_inds, width).astype(np.float32)
        topk_xs = (topk_inds % width).astype(np.float32)

        topk_scores_reshaped = topk_scores.reshape(batch, -1)
        topk_score, topk_ind = self.get_topk(topk_scores_reshaped, K)
        topk_clses = np.floor_divide(topk_ind, K).astype(np.int32)

        topk_inds = self.gather_feat(topk_inds.reshape(batch, -1, 1), topk_ind).reshape(batch, K)
        topk_ys = self.gather_feat(topk_ys.reshape(batch, -1, 1), topk_ind).reshape(batch, K)
        topk_xs = self.gather_feat(topk_xs.reshape(batch, -1, 1), topk_ind).reshape(batch, K)

        return topk_score, topk_inds, topk_clses, topk_ys, topk_xs

    def get_topk(self,scores, K, dim=-1):
        # Compute the number of dimensions in the input array
        ndim = scores.ndim
        # Convert the specified dimension to a positive index
        if dim < 0:
            dim = ndim + dim
        # Compute the top K values and indices along the specified dimension
        topk_inds = np.argpartition(-scores, K, axis=dim)
        if ndim == 1:
            topk_inds = topk_inds[:K]
        else:
            topk_inds = topk_inds[(slice(None),) * dim + (slice(None, K),)]
        topk_vals = np.take_along_axis(scores, topk_inds, axis=dim)
        # Sort the top K values and indices along the specified dimension
        sort_inds = np.argsort(-topk_vals, axis=dim)
        topk_inds = np.take_along_axis(topk_inds, sort_inds, axis=dim)
        topk_vals = np.take_along_axis(topk_vals, sort_inds, axis=dim)
        # Return the top K values and indices
        return topk_vals, topk_inds

    def gather_feat(self, feat, ind, mask=None):
        dim = feat.shape[2]
        ind = np.repeat(ind[..., np.newaxis], dim, axis=2)
        feat = np.take_along_axis(feat, ind, axis=1)
        if mask is not None:
            mask = np.repeat(mask[..., np.newaxis], dim, axis=2)
            feat = feat[mask]
            feat = feat.reshape(-1, dim)
        return feat

    def transpose_and_gather_feat(self, feat, ind):
        feat = feat.transpose(0, 2, 3, 1)
        # make sure the array has a contiguous memory layout
        feat = np.ascontiguousarray(feat)
        feat = feat.reshape(feat.shape[0], -1, feat.shape[3])
        feat = self.gather_feat(feat, ind)
        return feat

    def post_processing(self, detections):
        """
        :param detections: [batch_size, K, 10]
        # (scores x 1, xs x 1, ys x 1, z_coor x 1, dim x 3, direction x 2, clses x 1)
        # (scores-0:1, xs-1:2, ys-2:3, z_coor-3:4, dim-4:7, direction-7:9, clses-9:10)
        :return:
        """
        # score based filtering
        keep_inds = (detections[:, 0] > self.score_thresh)
        filtered_detections = detections[keep_inds, :]
        if filtered_detections.shape[0] > 0:
            classes = filtered_detections[:, 9]
            scores = filtered_detections[:, 0]
            x = (filtered_detections[:, 2] * DOWN_RATIO) /BEV_HEIGHT * self.BOUND_SIZE_X + self.MIN_FRONT_X
            y = (filtered_detections[:, 1] * DOWN_RATIO) /BEV_HEIGHT * self.BOUND_SIZE_Y + self.MIN_Y
            z = filtered_detections[:, 3] + self.MIN_Z
            heights = filtered_detections[:, 4]
            widths = filtered_detections[:, 5]
            lengths = filtered_detections[:, 6]
            yaw = self.get_yaw(filtered_detections[:,7:9]).astype(np.float32)

            return np.column_stack((classes, x, y, z, heights, widths, lengths, yaw, scores))
        else:
            return np.empty((0,9))

    def get_yaw(self, direction):
        return -np.arctan2(direction[:, 0:1], direction[:, 1:2])

    def generate_autoware_objects(self, detections, header, transform):

        """
        Generate Autoware DetectedObject from Detections
        :param detections: SFA detections
        :param header: time stamp corresponding to poinctloud msg
        :param tf_matrix: 4x4 homogenous transformation matrix to go from lidar frame to output frame
        :param tf_rot: quaternion representing rotation to go from lidar frame to output frame
        :return: AutowareDetectedObject
        """
        detected_objects_list = []
        for i, (cls_id, x, y, z, height, width, length, yaw, score) in enumerate(detections):

            detected_object = DetectedObject()
            detected_object.id = i
            detected_object.header.frame_id = self.output_frame
            detected_object.header.stamp = header.stamp
            detected_object.label = CLASS_NAMES[cls_id]
            detected_object.color = LIGHT_BLUE
            detected_object.valid = True
            detected_object.score = score
            detected_object.pose.position.x = x
            detected_object.pose.position.y = y
            detected_object.pose.position.z = z
            detected_object.pose.orientation = get_orientation_from_heading(yaw)
            detected_object.pose = transform_pose(detected_object.pose, transform)
            detected_object.pose_reliable = True

            # object dimensions
            detected_object.dimensions.x = length
            detected_object.dimensions.y = width
            detected_object.dimensions.z = height
            # Populate convex hull
            detected_object.convex_hull = create_hull(detected_object, self.output_frame, header.stamp)

            detected_objects_list.append(detected_object)

        return detected_objects_list

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('sfa_detector', log_level=rospy.INFO)
    node = SFADetector()
    node.run()
