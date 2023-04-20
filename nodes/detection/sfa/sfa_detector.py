#!/usr/bin/env python3

from __future__ import print_function
from __future__ import division

import rospy
import ros_numpy
import math
import torch
import torch.nn.functional as F
import numpy as np
import cv2
import tf
from tf.transformations import quaternion_multiply, quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Quaternion, Point, PolygonStamped, Pose
from sensor_msgs.msg import PointCloud2

from std_msgs.msg import ColorRGBA
from autoware_msgs.msg import DetectedObjectArray, DetectedObject

import fpn_resnet

LIGHT_BLUE = ColorRGBA(0.5, 0.5, 1.0, 0.8)

class SFAInference:
    def __init__(self):
        rospy.loginfo(self.__class__.__name__ + " - Initializing")

        # Params
        self.model_path = rospy.get_param("~model_path", './kilynuaron_weights_70.pth')  # path of the trained model
        self.front_only = rospy.get_param("~front_only", True) # Enable detections for only front FOV. Setting it to false runs detections on 360 FOV

        self.MIN_FRONT_X = rospy.get_param("~minX", 0)  # minimum distance on lidar +ve x-axis to process points at (front_detections)
        self.MAX_FRONT_X = rospy.get_param("~maxX", 50)  # maximum distance on lidar +ve x-axis to process points at (back detections)
        self.MIN_BACK_X = rospy.get_param("~maxBackX", -50)  # maxmimum distance on lidar -ve x-axis to process points at
        self.MAX_BACK_X = rospy.get_param("~maxBackX", 0)  # minimum distance on lidar -ve x-axis to process points at
        self.MIN_Y = rospy.get_param("~minY", -25)  # maxmimum distance on lidar -ve y-axis to process points at
        self.MAX_Y = rospy.get_param("~maxY", 25)  # maxmimum distance on lidar +ve y-axis to process points at
        self.MIN_Z = rospy.get_param("~minZ", -2.73)  # maxmimum distance on lidar -ve z-axis to process points at
        self.MAX_Z = rospy.get_param("~maxZ", 1.27)  # maxmimum distance on lidar +ve z-axis to process points at

        self.BEV_HEIGHT = rospy.get_param("~bev_height", 608)  # number of rows of the bev image
        self.BEV_WIDTH = rospy.get_param("~bev_height", 608)  # number of columns of the bev image

        self.K = rospy.get_param("~K", 50)  # number of top scoring detections to process
        self.DOWN_RATIO = rospy.get_param("~down_ratio", 4)
        self.NUM_CLASSES = rospy.get_param("~num_classes", 3) # number of classes to detect
        self.PEAK_THRESH = rospy.get_param("~peak_thresh", 0.2) # score filter

        self.only_front = rospy.get_param("~only_front", True)   # only front detections. If False, both front and back detections enabled
        self.output_frame = rospy.get_param("~output_frame", 'map')  # transform detected objects from lidar frame to this frame
        self.lidar_frame = rospy.get_param("~lidar_frame", 'lidar_center') # frame_id in which objects are published

        self.DISCRETIZATION = (self.MAX_FRONT_X - self.MIN_FRONT_X) / self.BEV_HEIGHT # 3D world discretization to 2D image: distance encoded by 1 pixel
        self.BOUND_SIZE_X = self.MAX_FRONT_X - self.MIN_FRONT_X
        self.BOUND_SIZE_Y = self.MAX_Y - self.MIN_Y

        self.device = torch.device('cuda:0') # device to run inference on

        # model configs
        self.NUM_LAYERS = 18
        self.HEAD_CONV = 64
        self.HEADS = {
            'hm_cen': self.NUM_CLASSES,
            'cen_offset': 2,
            'direction': 2,
            'z_coor': 1,
            'dim': 3
        }

        self.model = self.get_model(self.model_path) # Get saved model from path

        # KITTI class names
        self.class_names = {0: "pedestrian",
                            1: "car",
                            2: "cyclist"}

        # transform listener
        self.tf_listener = tf.TransformListener()

        # Subscribers and Publishers
        self.detected_object_array_pub = rospy.Publisher('detected_objects', DetectedObjectArray, queue_size=1)
        rospy.Subscriber('points_raw', PointCloud2, self.pointcloud_callback, queue_size=1)

    def get_model(self, weights_path):
        """
        model_path: path of the loaded model
        return: Loaded model with its trained parameters. In inference mode
        """

        model = fpn_resnet.get_pose_net(num_layers=self.NUM_LAYERS, heads=self.HEADS, head_conv=self.HEAD_CONV, imagenet_pretrained=False)
        model.load_state_dict(torch.load(weights_path, map_location='cpu'))
        rospy.loginfo(self.__class__.__name__ + "- Loaded weights from {}\n".format(weights_path))

        model = model.to(device=self.device)
        model.eval()

        return model

    def pointcloud_callback(self, pointcloud):
        """

        pointcloud: raw lidar data points
        return: None - publish autoware DetectedObjects
        """
        # get the transform (translation and rotation) from lidar_frame to output frame at the time when the poinctloud msg was published
        trans, tf_rot = self.tf_listener.lookupTransform(self.output_frame, self.lidar_frame, pointcloud.header.stamp)
        # convert the looked up transform to a 4x4 homogenous transformation matrix.
        tf_matrix = self.tf_listener.fromTranslationRotation(trans, tf_rot)
        # Unpack pointcloud2 msg ype to numpy array
        pcd_array = ros_numpy.numpify(pointcloud)

        # Reshape the array into shape -> (num_points, 4). 4 corresponds to the fields -> x,y,z,intensity
        points = np.zeros((pcd_array.shape[0], 4))
        points[:, 0] = pcd_array['x']
        points[:, 1] = pcd_array['y']
        points[:, 2] = pcd_array['z']
        points[:, 3] = pcd_array['intensity']

        detected_objects_array = DetectedObjectArray()
        detected_objects_array.header.stamp = pointcloud.header.stamp
        detected_objects_array.header.frame_id = self.output_frame

        # check whether to run inference only on the front FOV of the lidar
        if not self.only_front:
            # run the detection pipline
            front_dets, back_dets = self.detect(points, only_front=False)
            detected_objects_array.objects += self.generate_autoware_objects(front_dets, pointcloud.header, tf_matrix, tf_rot)
            detected_objects_array.objects += self.generate_autoware_objects(back_dets, pointcloud.header, tf_matrix, tf_rot)
        else:
            front_dets = self.detect(points, only_front=True)
            detected_objects_array.objects += self.generate_autoware_objects(front_dets, pointcloud.header, tf_matrix, tf_rot)

        # publish detected objects
        self.detected_object_array_pub.publish(detected_objects_array)


    def detect(self, points, only_front):
        with torch.no_grad():
            front_lidar = self.get_filtered_lidar(points)
            front_bev_map = self.make_bev_map(front_lidar)
            front_bev_map = torch.from_numpy(front_bev_map)

            front_detections, front_bevmap = self.do_detection(front_bev_map, is_front=True)
            front_final_dets = self.convert_det_to_real_values(front_detections)

            if not only_front:
                back_lidar = self.get_filtered_lidar(points, is_front=False)
                back_bev_map = self.make_bev_map(back_lidar)
                back_bev_map = torch.from_numpy(back_bev_map)
                back_detections, back_bevmap, = self.do_detection(back_bev_map, is_front=False)
                back_final_dets = self.convert_det_to_real_values(back_detections)
                if back_final_dets.shape[0] != 0:
                    back_final_dets[:, 1] = back_final_dets[:, 1] * -1
                    back_final_dets[:, 2] = back_final_dets[:, 2] * -1

                return front_final_dets , back_final_dets
            else:
                return front_final_dets

    def get_filtered_lidar(self, lidar, is_front=True):

        if is_front:
            # Remove the point out of range x,y,z
            mask = np.where((lidar[:, 0] >= self.MIN_FRONT_X) & (lidar[:, 0] <= self.MAX_FRONT_X) &
                            (lidar[:, 1] >= self.MIN_Y) & (lidar[:, 1] <= self.MAX_Y) &
                            (lidar[:, 2] >= self.MIN_Z) & (lidar[:, 2] <= self.MAX_Z))

        elif not is_front:
            mask = np.where((lidar[:, 0] >= self.MIN_BACK_X) & (lidar[:, 0] <= self.MAX_BACK_X) &
                            (lidar[:, 1] >= self.MIN_Y) & (lidar[:, 1] <= self.MAX_Y) &
                            (lidar[:, 2] >= self.MIN_Z) & (lidar[:, 2] <= self.MAX_Z))

        lidar = lidar[mask]
        lidar[:, 2] = lidar[:, 2] - self.MIN_Z

        return lidar

    def make_bev_map(self, PointCloud_):
        Height = self.BEV_HEIGHT + 1
        Width = self.BEV_WIDTH + 1

        # Discretize Feature Map
        PointCloud = np.copy(PointCloud_)
        PointCloud[:, 0] = np.int_(np.floor(PointCloud[:, 0] / self.DISCRETIZATION))
        PointCloud[:, 1] = np.int_(np.floor(PointCloud[:, 1] / self.DISCRETIZATION) + Width / 2)

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
        max_height = float(np.abs(self.MAX_Z - self.MIN_Z))
        heightMap[np.int_(PointCloud_top[:, 0]), np.int_(PointCloud_top[:, 1])] = PointCloud_top[:, 2] / max_height

        normalizedCounts = np.minimum(1.0, np.log(unique_counts + 1) / np.log(64))
        intensityMap[np.int_(PointCloud_top[:, 0]), np.int_(PointCloud_top[:, 1])] = PointCloud_top[:, 3]
        densityMap[np.int_(PointCloud_top[:, 0]), np.int_(PointCloud_top[:, 1])] = normalizedCounts

        RGB_Map = np.zeros((3, Height - 1, Width - 1))
        RGB_Map[2, :, :] = densityMap[:self.BEV_HEIGHT, :self.BEV_WIDTH]  # r_map
        RGB_Map[1, :, :] = heightMap[:self.BEV_HEIGHT, :self.BEV_WIDTH]  # g_map
        RGB_Map[0, :, :] = intensityMap[:self.BEV_HEIGHT, :self.BEV_WIDTH]  # b_map

        return RGB_Map

    def do_detection(self, bevmap, is_front):
        if not is_front:
            bevmap = torch.flip(bevmap, [1, 2])

        input_bev_maps = bevmap.unsqueeze(0).to(self.device, non_blocking=True).float()
        outputs = self.model(input_bev_maps)

        outputs['hm_cen'] = self._sigmoid(outputs['hm_cen'])
        outputs['cen_offset'] = self._sigmoid(outputs['cen_offset'])
        # detections size (batch_size, K, 10)
        detections = self.decode(outputs['hm_cen'], outputs['cen_offset'], outputs['direction'], outputs['z_coor'], outputs['dim'], self.K)
        detections = detections.cpu().numpy().astype(np.float32)
        detections = self.post_processing(detections)

        return detections[0], bevmap

    def _sigmoid(self, x):
        return torch.clamp(x.sigmoid_(), min=1e-4, max=1 - 1e-4)

    def decode(self, hm_cen, cen_offset, direction, z_coor, dim, K=40):
        batch_size, num_classes, height, width = hm_cen.size()

        hm_cen = self._nms(hm_cen)
        scores, inds, clses, ys, xs = self._topk(hm_cen, K=K)
        if cen_offset is not None:
            cen_offset = self._transpose_and_gather_feat(cen_offset, inds)
            cen_offset = cen_offset.view(batch_size, K, 2)
            xs = xs.view(batch_size, K, 1) + cen_offset[:, :, 0:1]
            ys = ys.view(batch_size, K, 1) + cen_offset[:, :, 1:2]
        else:
            xs = xs.view(batch_size, K, 1) + 0.5
            ys = ys.view(batch_size, K, 1) + 0.5

        direction = self._transpose_and_gather_feat(direction, inds)
        direction = direction.view(batch_size, K, 2)
        z_coor = self._transpose_and_gather_feat(z_coor, inds)
        z_coor = z_coor.view(batch_size, K, 1)
        dim = self._transpose_and_gather_feat(dim, inds)
        dim = dim.view(batch_size, K, 3)
        clses = clses.view(batch_size, K, 1).float()
        scores = scores.view(batch_size, K, 1)

        # (scores x 1, ys x 1, xs x 1, z_coor x 1, dim x 3, direction x 2, clses x 1)
        # (scores-0:1, ys-1:2, xs-2:3, z_coor-3:4, dim-4:7, direction-7:9, clses-9:10)
        # detections: [batch_size, K, 10]
        detections = torch.cat([scores, xs, ys, z_coor, dim, direction, clses], dim=2)

        return detections

    def _nms(self, heat, kernel=3):
        pad = (kernel - 1) // 2
        hmax = F.max_pool2d(heat, (kernel, kernel), stride=1, padding=pad)
        keep = (hmax == heat).float()

        return heat * keep

    def _topk(self, scores, K=40):
        batch, cat, height, width = scores.size()

        topk_scores, topk_inds = torch.topk(scores.view(batch, cat, -1), K)

        topk_inds = topk_inds % (height * width)
        topk_ys = (torch.floor_divide(topk_inds, width)).float()
        topk_xs = (topk_inds % width).int().float()

        topk_score, topk_ind = torch.topk(topk_scores.view(batch, -1), K)
        topk_clses = (torch.floor_divide(topk_ind, K)).int()
        topk_inds = self._gather_feat(topk_inds.view(batch, -1, 1), topk_ind).view(batch, K)
        topk_ys = self._gather_feat(topk_ys.view(batch, -1, 1), topk_ind).view(batch, K)
        topk_xs = self._gather_feat(topk_xs.view(batch, -1, 1), topk_ind).view(batch, K)

        return topk_score, topk_inds, topk_clses, topk_ys, topk_xs

    def _gather_feat(self, feat, ind, mask=None):
        dim = feat.size(2)
        ind = ind.unsqueeze(2).expand(ind.size(0), ind.size(1), dim)
        feat = feat.gather(1, ind)
        if mask is not None:
            mask = mask.unsqueeze(2).expand_as(feat)
            feat = feat[mask]
            feat = feat.view(-1, dim)
        return feat

    def _transpose_and_gather_feat(self, feat, ind):
        feat = feat.permute(0, 2, 3, 1).contiguous()
        feat = feat.view(feat.size(0), -1, feat.size(3))
        feat = self._gather_feat(feat, ind)
        return feat

    def post_processing(self,detections):
        """
        :param detections: [batch_size, K, 10]
        # (scores x 1, xs x 1, ys x 1, z_coor x 1, dim x 3, direction x 2, clses x 1)
        # (scores-0:1, xs-1:2, ys-2:3, z_coor-3:4, dim-4:7, direction-7:9, clses-9:10)
        :return:
        """
        # TODO: Need to consider rescale to the original scale: x, y

        ret = []
        for i in range(detections.shape[0]):
            top_preds = {}
            classes = detections[i, :, -1]
            for j in range(self.NUM_CLASSES):
                inds = (classes == j)
                # x, y, z, h, w, l, yaw
                top_preds[j] = np.concatenate([
                    detections[i, inds, 0:1],
                    detections[i, inds, 1:2] * self.DOWN_RATIO,
                    detections[i, inds, 2:3] * self.DOWN_RATIO,
                    detections[i, inds, 3:4],
                    detections[i, inds, 4:5],
                    detections[i, inds, 5:6] / self.BOUND_SIZE_Y * self.BEV_WIDTH,
                    detections[i, inds, 6:7] / self.BOUND_SIZE_X * self.BEV_HEIGHT,
                    self.get_yaw(detections[i, inds, 7:9]).astype(np.float32)], axis=1)
                # Filter by peak_thresh
                if len(top_preds[j]) > 0:
                    keep_inds = (top_preds[j][:, 0] > self.PEAK_THRESH)
                    top_preds[j] = top_preds[j][keep_inds]
            ret.append(top_preds)

        return ret

    def get_yaw(self, direction):
        return np.arctan2(direction[:, 0:1], direction[:, 1:2])

    def convert_det_to_real_values(self, detections, add_score=False):
        kitti_dets = []
        for cls_id in range(self.NUM_CLASSES):
            if len(detections[cls_id]) > 0:
                for det in detections[cls_id]:
                    # (scores-0:1, x-1:2, y-2:3, z-3:4, dim-4:7, yaw-7:8)
                    _score, _x, _y, _z, _h, _w, _l, _yaw = det
                    _yaw = -_yaw
                    x = _y / self.BEV_HEIGHT * self.BOUND_SIZE_X + self.MIN_FRONT_X
                    y = _x / self.BEV_WIDTH * self.BOUND_SIZE_Y + self.MIN_Y
                    z = _z + self.MIN_Z
                    w = _w / self.BEV_WIDTH * self.BOUND_SIZE_Y
                    l = _l / self.BEV_HEIGHT * self.BOUND_SIZE_X
                    if not add_score:
                        kitti_dets.append([cls_id, x, y, z, _h, w, l, _yaw])
                    else:
                        kitti_dets.append([cls_id, x, y, z, _h, w, l, _yaw, _score])

        return np.array(kitti_dets)

    def generate_autoware_objects(self, detections, header, tf_matrix, tf_rot):

        """
        Generate Autoware DetectedObject from Detections
        :param detections: SFA detections
        :param header: time stamp corresponding to poinctloud msg
        :param tf_matrix: 4x4 homogenous transformation matrix to go from lidar frame to output frame
        :param tf_rot: quaternion representing rotation to go from lidar frame to output frame
        :return: AutowareDetectedObject
        """
        detected_objects_list = []
        for object in detections:

            detected_object = DetectedObject()
            detected_object.header.frame_id = self.output_frame
            detected_object.header.stamp = header.stamp
            detected_object.label = self.class_names[object[0]]
            detected_object.color = LIGHT_BLUE
            detected_object.valid = True

            position_in_lidar = np.array([object[1], object[2], object[3], 1])
            orientation_in_lidar = quaternion_from_euler(0, 0, object[7])

            detected_object.pose = self.transform_pose(position_in_lidar, orientation_in_lidar, tf_matrix, tf_rot)
            detected_object.pose_reliable = True

            # object dimensions
            detected_object.dimensions.x = object[6]
            detected_object.dimensions.y = object[5]
            detected_object.dimensions.z = object[4]
            # Populate convex hull
            detected_object.convex_hull = self.produce_hull(detected_object.pose, detected_object.dimensions, header.stamp)

            detected_object.valid = True
            detected_object.acceleration_reliable = False
            detected_object.velocity_reliable = False

            detected_objects_list.append(detected_object)

        return detected_objects_list

    def transform_pose(self, position, orientation, tf_matrix, tf_rot):

        # create Pose object.
        transformed_pose = Pose()
        # transform input pose's position using the transformation matrix
        transformed_x, transformed_y, transformed_z, _ = np.dot(tf_matrix, position)
        transformed_pose.position = Point(transformed_x, transformed_y, transformed_z)

        # transform the objects' orientation quaternion to output frame by multiplying it with the transform quaternion.
        # Note: it's not an ordinary element-wise multiplication
        x, y, z, w = quaternion_multiply(tf_rot, orientation)
        transformed_pose.orientation = Quaternion(x, y, z, w)

        return transformed_pose

    def produce_hull(self, obj_pose, obj_dims, stamp):

        """
        Produce convex hull for an object given its pose and dimensions
        :param obj_pose: geometry_msgs/Pose. Position and orientation of object
        :param obj_dims: Vector3 - length, width and height of object
        :param vella_stamp: Time stamp at which the lidar pointcloud was created
        :return: geometry_msgs/PolygonStamped
        """
        convex_hull = PolygonStamped()
        convex_hull.header.frame_id = self.output_frame
        convex_hull.header.stamp = stamp

        # Taken from Autoware mini
        # compute heading angle from object's orientation
        _, _, heading = euler_from_quaternion(
            (obj_pose.orientation.x, obj_pose.orientation.y, obj_pose.orientation.z, obj_pose.orientation.w))

        # use cv2.boxPoints to get a rotated rectangle given the angle
        points = cv2.boxPoints((
            (obj_pose.position.x, obj_pose.position.y),
            (obj_dims.x, obj_dims.y),
            math.degrees(heading)
        ))
        convex_hull.polygon.points = [Point(x, y, obj_pose.position.z) for x, y in points]
        # Add the first polygon point to the list of points to make it 5 points in total and complete the loop
        convex_hull.polygon.points.append(convex_hull.polygon.points[0])

        return convex_hull

    @staticmethod
    def run():
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('sfa_detection', anonymous=True, log_level=rospy.INFO)
    node = SFAInference()
    node.run()
