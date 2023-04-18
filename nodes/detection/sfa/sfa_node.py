#!/usr/bin/env python3

from __future__ import print_function
from __future__ import division

import rospy
import tf
from sensor_msgs.msg import PointCloud2
from autoware_msgs.msg import DetectedObjectArray
from  ros_numpy import numpify

import warnings

warnings.filterwarnings("ignore", category=UserWarning)
import argparse
import numpy as np
from easydict import EasyDict as edict
import torch
from model_utils import get_num_parameters, get_filtered_lidar, makeBEVMap, do_detect
from sfa_utils import convert_det_to_real_values

import os

from save_model import create_model


class SFAInference:
    def __init__(self):

        # Params
        self.model_path = rospy.get_param("~model_path", './kilynuaronwa_model.pth')  # path of the trained model
        self.front_only = rospy.get_param("~front_only", True) # Enable detections for only front FOV. Setting it to false runs detections on 360 FOV

        self.boundary_min_x = rospy.get_param("~minX", 0)  # minimum distance on lidar x-axis to process points at
        self.boundary_max_x = rospy.get_param("~maxX", 50)  # maxmimum distance on lidar x-axis to process points at
        self.boundary_min_y = rospy.get_param("~minY", -25)  # maxmimum distance on lidar y-axis to process points at
        self.boundary_max_y = rospy.get_param("~maxY", 25)  # maxmimum distance on lidar y-axis to process points at
        self.boundary_min_z = rospy.get_param("~minZ", -2.73)  # maxmimum distance on lidar z-axis to process points at
        self.boundary_max_z = rospy.get_param("~maxZ", 1.27)  # maxmimum distance on lidar z-axis to process points at

        self.bev_height = rospy.get_param("~bev_height", 608)  # number of rows of the bev image
        self.bev_width = rospy.get_param("~bev_height", 608)  # number of columns of the bev image

        self.K = rospy.get_param("~K", 50)  # number of top scoring detections to process
        self.down_ratio = rospy.get_param("~down_ratio", 4)
        self.num_classes = rospy.get_param("~num_classes", 3) # number of classes to detect
        self.peak_thresh = rospy.get_param("~peak_thresh", 0.2) # score filter

        self.discretization = (self.boundary_max_x - self.boundary_min_x) / self.bev_height # 3D world discretization to 2D image: distance encoded by 1 pixel

        self.bound_size_x = self.boundary_max_x - self.boundary_min_x
        self.bound_size_y = self.boundary_max_y - self.boundary_min_y
        self.bound_size_z = self.boundary_max_z - self.boundary_min_z

        self.device = torch.device('cuda:0')
        # self.lidar_frame = rospy.get_param("~lidar_frame", 'lidar_center')  # frame_id for tracks published by vella - vella does not populate frame_id of vdk/tracks messages
        # self.output_frame = rospy.get_param("~output_frame", 'map')  # transform vella tracks from lidar frame to this frame

        self.configs = self.parse_config()
        # self.model = self.get_model(self.model_path)
        self.model = self.get_model()

        self.class_names = {0: "pedestrian",
                            1: "car",
                            2: "cyclist"}

        self.front_only= True

        # transform listener
        self.tf_listener = tf.TransformListener()

        # Autoware detected objects publisher
        self.detected_object_array_pub = rospy.Publisher('detected_objects', DetectedObjectArray, queue_size=1)
        # vella tracks subscriber
        rospy.Subscriber('/lidar_center/points_raw', PointCloud2, self.pointcloud_callback, queue_size=1,buff_size=6553600)

        rospy.loginfo(self.__class__.__name__ + " - SFA detector initialized")

    def parse_config(self):
        parser = argparse.ArgumentParser(description='Testing config for the Implementation')
        parser.add_argument('--saved_fn', type=str, default='fpn_resnet_18', metavar='FN',
                            help='The name using for saving logs, models,...')
        parser.add_argument('-a', '--arch', type=str, default='fpn_resnet_18', metavar='ARCH',
                            help='The name of the model architecture')
        parser.add_argument('--pretrained_path', type=str,
                            default='/home/zain/autoware_mini_ws/src/autoware_mini/data/sfa_models/Model_fpn_resnet_18_epoch_70.pth', metavar='PATH',
                            help='the path of the pretrained checkpoint')
        parser.add_argument('--K', type=int, default=50,
                            help='the number of top K')
        parser.add_argument('--no_cuda', action='store_true',
                            help='If true, cuda is not used.')
        parser.add_argument('--gpu_idx', default=0, type=int,
                            help='GPU index to use.')
        parser.add_argument('--num_samples', type=int, default=None,
                            help='Take a subset of the dataset to run and debug')
        parser.add_argument('--num_workers', type=int, default=1,
                            help='Number of threads for loading data')
        parser.add_argument('--batch_size', type=int, default=1,
                            help='mini-batch size (default: 4)')
        parser.add_argument('--peak_thresh', type=float, default=0.3)
        parser.add_argument('--save_test_output', action='store_true',
                            help='If true, the output image of the testing phase will be saved')
        parser.add_argument('--output_format', type=str, default='image', metavar='PATH',
                            help='the type of the test output (support image or video)')
        parser.add_argument('--output_video_fn', type=str, default='out_fpn_resnet_18', metavar='PATH',
                            help='the video filename if the output format is video')
        parser.add_argument('--output-width', type=int, default=608,
                            help='the width of showing output, the height maybe vary')

        configs = edict(vars(parser.parse_args()))
        configs.pin_memory = True
        configs.distributed = False  # For testing on 1 GPU only

        configs.input_size = (608, 608)
        configs.hm_size = (152, 152)
        configs.down_ratio = 4
        configs.max_objects = 50

        configs.imagenet_pretrained = False
        configs.head_conv = 64
        configs.num_classes = 3
        configs.num_center_offset = 2
        configs.num_z = 1
        configs.num_dim = 3
        configs.num_direction = 2  # sin, cos

        configs.heads = {
            'hm_cen': configs.num_classes,
            'cen_offset': configs.num_center_offset,
            'direction': configs.num_direction,
            'z_coor': configs.num_z,
            'dim': configs.num_dim
        }
        configs.num_input_features = 4

        ####################################################################
        ##############Dataset, Checkpoints, and results dir configs#########
        ####################################################################
        configs.root_dir = '../'
        configs.dataset_dir = os.path.join(configs.root_dir, 'dataset', 'kitti')

        return configs

    def get_model(self):
        model = create_model(self.configs)
        print('\n\n' + '-*=' * 30 + '\n\n')
        assert os.path.isfile(self.configs.pretrained_path), "No file at {}".format(self.configs.pretrained_path)
        model.load_state_dict(torch.load(self.configs.pretrained_path, map_location='cpu'))
        print('Loaded weights from {}\n'.format(self.configs.pretrained_path))

        self.configs.device = torch.device('cpu' if self.configs.no_cuda else 'cuda:{}'.format(self.configs.gpu_idx))
        model = model.to(device=self.configs.device)
        # torch.save(model, "./kilynuaronwa_model.pth")
        model.eval()

        return  model

    # def get_model(self, model_path):
    #     """
    #     model_path: path of the loaded model
    #     return: Loaded model with its trained parameters. In inference mode
    #     """
    #     model = torch.load(model_path)
    #     model.eval()
    #     rospy.loginfo(self.__class__.__name__ + " - Model Loaded with {} parameters".format(get_num_parameters(model)))
    #
    #     return model

    def pointcloud_callback(self, pointcloud):
        """
        pointcloud: raw lidar sensor data points in ros format
        return: None
        publishes Autoware DetectedObjects
        """

        # Unpack pointcloud2 msg ytpe to numpy array
        pcd_array = numpify(pointcloud)

        # Reshape the array into shape -> (num_points, 4). 4 corresponds to the fields -> x,y,z,intensity
        points = np.zeros((pcd_array.shape[0], 4))
        points[:, 0] = pcd_array['x']
        points[:, 1] = pcd_array['y']
        points[:, 2] = pcd_array['z']
        points[:, 3] = pcd_array['intensity']

        detected_objects_array = DetectedObjectArray()
        detected_objects_array.header.stamp = pointcloud.header.stamp
        detected_objects_array.header.frame_id = 'lidar_center'

        if not self.front_only:
            front_dets, back_dets = self.detect(points, front_only=self.front_only)
            detected_objects_array.objects += self.generate_detected_objects(front_dets, pointcloud.header)
            detected_objects_array.objects += self.generate_detected_objects(back_dets, pointcloud.header)
        else:
            front_dets = self.detect(points, front_only=True)
            print(front_dets)
            # detected_objects_array.objects += self.generate_detected_objects(front_dets, pointcloud.header)

        # self.detected_object_array_pub.publish(detected_objects_array)

    def detect(self, points, front_only):
        with torch.no_grad():
            front_lidar = get_filtered_lidar(points, self.boundary_min_x, self.boundary_max_x, self.boundary_min_y, self.boundary_max_y, self.boundary_min_z, self.boundary_max_z)
            front_bev_map = makeBEVMap(front_lidar, self.bev_height, self.bev_width, self.discretization, self.boundary_min_z, self.boundary_max_z)
            front_bev_map = torch.from_numpy(front_bev_map)
            front_detections, front_bevmap = do_detect(self.device, self.bound_size_x, self.bound_size_y, self.bev_width, self.bev_height, self.K, self.num_classes, self.down_ratio, self.peak_thresh, self.model, front_bev_map, is_front=True)
            print(front_detections)
            front_final_dets = convert_det_to_real_values(front_detections)
            return front_final_dets

            # if not front_only:
            #     back_lidar = get_filtered_lidar(points, cnf.boundary_back)
            #     back_bev_map = makeBEVMap(back_lidar, cnf.boundary_back)
            #     back_bev_map = torch.from_numpy(back_bev_map)
            #     back_detections, back_bevmap, _ = do_detect(self.configs, self.model, back_bev_map, is_front=False)
            #     back_final_dets = convert_det_to_real_values(back_detections)
            #     if back_final_dets.shape[0] != 0:
            #         back_final_dets[:, 1] = back_final_dets[:, 1] * -1
            #         back_final_dets[:, 2] = back_final_dets[:, 2] * -1
            #
            #     return front_final_dets , back_final_dets
            # else:
            #     return front_final_dets

    @staticmethod
    def run():
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('sfa_detection', anonymous=True, log_level=rospy.INFO)
    node = SFAInference()
    node.run()
