#!/usr/bin/env python3

import rospy
import numpy as np

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

DOWN_RATIO = 4
NUM_CLASSES = 12  # number of classes to detect
CLASS_NAMES = {
    0: 'car',
    1 : 'pedestrian',
    2: 'bicyclist',
    3: 'motorcyclist',
    4: 'bus',
    5: 'truck',
    6: 'trailer',
    7: 'animal',
    8: 'traffic_cone',
    9: 'barrier',
    10: 'stroller',
    11: 'on_road_obstacle'
}

class SFADetector:
    def __init__(self):
        # Params
        self.onnx_path = rospy.get_param("~onnx_path")  # path of the trained model
        self.load_onnx(self.onnx_path) # get onnx model
        rospy.loginfo("%s - loaded ONNX model file %s", rospy.get_name(), self.onnx_path)

        input_shape = self.model.get_inputs()[0].shape
        if input_shape[-1] == 608:
            assert tuple(input_shape[1:]) == (3, 608, 608), "Incorrect input shape for short distance model: " + str(input_shape[1:])
            self.only_front = False
            self.BEV_HEIGHT = 608  # number of rows of the bev image
            self.BEV_WIDTH = 608  # number of columns of the bev image
            self.MAX_FRONT_X = 50  # maximum distance on lidar +ve x-axis to process points at (back detections)
            self.MIN_BACK_X = -50  # maxmimum distance on lidar -ve x-axis to process points at
            self.MIN_Y = -25  # maxmimum distance on lidar -ve y-axis to process points at
            self.MAX_Y = 25  # maxmimum distance on lidar +ve y-axis to process points at
        else:
            assert tuple(input_shape[1:]) == (3, 1216, 1216), "Incorrect input shape for long distance model: " + str(input_shape[1:])
            self.only_front = True
            self.BEV_HEIGHT = 1216  # number of rows of the bev image
            self.BEV_WIDTH = 1216  # number of columns of the bev image
            self.MAX_FRONT_X = 80  # maximum distance on lidar +ve x-axis to process points at (back detections)
            self.MIN_BACK_X = -80  # maxmimum distance on lidar -ve x-axis to process points at
            self.MIN_Y = -40  # maxmimum distance on lidar -ve y-axis to process points at
            self.MAX_Y = 40  # maxmimum distance on lidar +ve y-axis to process points at

        self.MIN_FRONT_X = 0  # minimum distance on lidar +ve x-axis to process points at (front_detections)
        self.MAX_BACK_X = 0  # minimum distance on lidar -ve x-axis to process points at
        self.MIN_Z = rospy.get_param("~min_z")  # maxmimum distance on lidar -ve z-axis to process points at
        self.MAX_Z = rospy.get_param("~max_z")  # maxmimum distance on lidar +ve z-axis to process points at
        self.DISCRETIZATION = (self.MAX_FRONT_X - self.MIN_FRONT_X) / self.BEV_HEIGHT  # 3D world discretization to 2D image: distance encoded by 1 pixel
        self.BOUND_SIZE_X = self.MAX_FRONT_X - self.MIN_FRONT_X
        self.BOUND_SIZE_Y = self.MAX_Y - self.MIN_Y
        self.MAX_HEIGHT = np.abs(self.MAX_Z - self.MIN_Z)

        self.score_thresh = rospy.get_param("~score_thresh")  # score filter
        self.top_k = rospy.get_param("~top_k")  # number of top scoring detections to process
        self.output_frame = rospy.get_param("/detection/output_frame")  # transform detected objects from lidar frame to this frame
        self.transform_timeout = rospy.get_param('~transform_timeout') # transform timeout when waiting for transform to output frame

        # transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        # Subscribers and Publishers
        self.detected_object_array_pub = rospy.Publisher('detected_objects', DetectedObjectArray, queue_size=1)
        rospy.Subscriber('points_raw', PointCloud2, self.pointcloud_callback, queue_size=1, buff_size=2*1024*1024)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def load_onnx(self, onnx_path):

        self.model = onnxruntime.InferenceSession(onnx_path, providers=['CUDAExecutionProvider'])

    def do_detection(self, input_bev_maps):

        return self.model.run([], {"input":input_bev_maps})[0]

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

        if self.only_front:
            input_bev_maps = np.expand_dims(front_bev_map, axis=0).astype(np.float32)
            detections = self.do_detection(input_bev_maps)
            return self.post_processing(detections[0])
        else:
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
        pointcloud[:, 1] = np.int_(np.floor(pointcloud[:, 1] / self.DISCRETIZATION) + self.BEV_WIDTH / 2)

        # sort-3times
        sorted_indices = np.lexsort((-pointcloud[:, 2], pointcloud[:, 1], pointcloud[:, 0]))
        pointcloud = pointcloud[sorted_indices]
        _, unique_indices, unique_counts = np.unique(pointcloud[:, 0:2], axis=0, return_index=True, return_counts=True)
        pointcloud_top = pointcloud[unique_indices]

        normalized_counts = np.minimum(1.0, np.log(unique_counts + 1) / np.log(64))

        hid_map = np.zeros((3, self.BEV_HEIGHT , self.BEV_WIDTH))
        hid_map[2, np.int_(pointcloud_top[:, 0]), np.int_(pointcloud_top[:, 1])] = normalized_counts # density map
        hid_map[1, np.int_(pointcloud_top[:, 0]), np.int_(pointcloud_top[:, 1])] = pointcloud_top[:, 2] / self.MAX_HEIGHT  # height map
        hid_map[0, np.int_(pointcloud_top[:, 0]), np.int_(pointcloud_top[:, 1])] = pointcloud_top[:, 3]  # intensity map

        return hid_map

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
            x = (filtered_detections[:, 2] * DOWN_RATIO) /self.BEV_HEIGHT * self.BOUND_SIZE_X + self.MIN_FRONT_X
            y = (filtered_detections[:, 1] * DOWN_RATIO) /self.BEV_HEIGHT * self.BOUND_SIZE_Y + self.MIN_Y
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
