#!/usr/bin/env python3

import traceback
import numpy as np
import shapely
import shapely.ops
import rospy
import tf2_ros
import message_filters
from ros_numpy import numpify, msgify

from std_msgs.msg import ColorRGBA
from autoware_mini.msg import Path
from lexus_platform.msg import Float32MultiArrayStamped
from geometry_msgs.msg import Point, Pose, PoseStamped, TransformStamped
from visualization_msgs.msg import MarkerArray, Marker
from jsk_rviz_plugins.msg import OverlayText

from autoware_mini.path import PathWrapper
from autoware_mini.transform import transform_point

class LaneBoundaryMatcher:

    def __init__(self):

        # parameters
        self.enable_height_correction = rospy.get_param("~enable_height_correction")
        self.lookahead_distance = rospy.get_param("~lookahead_distance")
        self.x_correction_treshold = rospy.get_param("~x_correction_treshold")
        self.y_correction_treshold = rospy.get_param("~y_correction_treshold")
        self.probability_treshold = rospy.get_param("~probability_treshold")
        self.transform_timeout = rospy.get_param("~transform_timeout")
        self.openpilot_delay_compensation = rospy.get_param("~openpilot_delay_compensation")
        self.alpha = rospy.get_param("~alpha")
        self.no_correction_weight = rospy.get_param("~no_correction_weight")

        # variables
        self.current_pose = None
        self.global_path = None

        self.x_correction = 0
        self.y_correction = 0
        self.z_correction = 0
        self.transform_matrix = np.eye(4)
        
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.publish_base_link_correction_tf()

        # publishers
        self.current_pose_pub = rospy.Publisher('current_pose', PoseStamped, queue_size=1, tcp_nodelay=True)
        self.lane_bound_markers_pub = rospy.Publisher('lane_boundary_matcher_markers', MarkerArray, queue_size=1, tcp_nodelay=True)
        self.gnss_corrections_detailed_pub = rospy.Publisher('/dashboard/gnss_corrections_detailed', OverlayText, queue_size=1)

        # subscribers
        rospy.Subscriber('current_pose_gnss', PoseStamped, self.current_pose_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/planning/lanelet2_global_path', Path, self.global_path_callback, queue_size=1, tcp_nodelay=True)
        lane_lines_sub = message_filters.Subscriber('/openpilot/lane_lines', Float32MultiArrayStamped, queue_size=1, tcp_nodelay=True)
        lane_lines_probs_sub = message_filters.Subscriber('/openpilot/lane_line_probs', Float32MultiArrayStamped, queue_size=1, tcp_nodelay=True)

        ts = message_filters.TimeSynchronizer([lane_lines_sub, lane_lines_probs_sub], queue_size=2)
        ts.registerCallback(self.lane_line_callback)

    def calculate_updated_correction(self, new_value, old_value, weight=1):
        alpha = weight * self.alpha
        return alpha * new_value + (1 - alpha) * old_value
    
    def current_pose_callback(self, msg):
        current_pose_matrix = numpify(msg.pose)
        corrected_current_pose_matrix = self.transform_matrix @ current_pose_matrix

        msg.pose = msgify(Pose, corrected_current_pose_matrix)
        self.current_pose_pub.publish(msg)

    def lane_line_callback(self, lane_lines_msg, lane_lines_probs_msg):
        try:
            openpilot_lane_boundaries = float32_multiarray_to_numpy(lane_lines_msg)
            openpilot_lane_boundary_probs = float32_multiarray_to_numpy(lane_lines_probs_msg)[1:3]

            global_path = self.global_path
            timestamp = lane_lines_msg.header.stamp - rospy.Duration.from_sec(self.openpilot_delay_compensation)

            if global_path is None or global_path.left_boundary is None or global_path.right_boundary is None:
                self.x_correction = self.calculate_updated_correction(0, self.x_correction)
                self.y_correction = self.calculate_updated_correction(0, self.y_correction)

                self.publish_base_link_correction_tf(correction_stamp=timestamp)
                return

            # Fetch transforms
            try:
                transform_openpilot = self.tf_buffer.lookup_transform("map_gnss", "openpilot", timestamp, rospy.Duration(self.transform_timeout))
                tf_matrix_openpilot = numpify(transform_openpilot.transform)
                transform_footprint = self.tf_buffer.lookup_transform("map_gnss", "base_footprint", timestamp, rospy.Duration(self.transform_timeout))
            except (tf2_ros.TransformException, rospy.ROSTimeMovedBackwardsException) as e:
                rospy.logwarn("%s - %s", rospy.get_name(), e)
                return
            
            current_pose_openpilot = transform_point(Point(0, 0, 0), transform_openpilot)
            
            if self.enable_height_correction:
                current_pose_footprint = transform_point(Point(0, 0, 0), transform_footprint)
                current_pos_dist = global_path.linestring.project(shapely.Point(current_pose_footprint.x, current_pose_footprint.y, current_pose_footprint.z))
                self.z_correction = global_path.get_elevation_at_distance(current_pos_dist) - current_pose_footprint.z

            ##################################################################
            # Trim map lane boundaries
            ##################################################################

            # find the distance of current position
            left_cur_pos_dist = global_path.left_boundary.project(shapely.Point(current_pose_openpilot.x, current_pose_openpilot.y, current_pose_openpilot.z))
            right_cur_pos_dist = global_path.right_boundary.project(shapely.Point(current_pose_openpilot.x, current_pose_openpilot.y, current_pose_openpilot.z))
            
            # make sure that lookahead distance does not go beyond the end of global path
            left_end_dist = min(left_cur_pos_dist + self.lookahead_distance, global_path.left_boundary.length)
            right_end_dist = min(right_cur_pos_dist + self.lookahead_distance, global_path.right_boundary.length)

            # cut out the relevant sections from the global path boundaries
            map_left_lane_boundary = shapely.ops.substring(global_path.left_boundary, left_cur_pos_dist, left_end_dist)
            map_right_lane_boundary = shapely.ops.substring(global_path.right_boundary, right_cur_pos_dist, right_end_dist)

            # return if one boundary given by substring is not a LineString type
            if not isinstance(map_left_lane_boundary, shapely.LineString) or not isinstance(map_right_lane_boundary, shapely.LineString):
                return
            
            ##################################################################
            # Transform openpilot lane boundaries and create linestrings
            ##################################################################

            # transfrom openpilot predicted lane boundaries to map_gnss frame
            center_openpilot_lane_boundaries = np.transpose(openpilot_lane_boundaries, (0, 2, 1))[1:3, :, :] # transpose axis 1 and 2
            flattened_openpilot_lane_boundaries = center_openpilot_lane_boundaries.reshape(-1, 4) # flatten to (2*n_points, 4)
            flattened_openpilot_lane_boundaries[:, 3] = 1 # replace the time dimension with ones to get homogeneous points
            homogeneous_openpilot_lane_boundaries = flattened_openpilot_lane_boundaries @ tf_matrix_openpilot.T # do the transform
            openpilot_lane_boundary_points = homogeneous_openpilot_lane_boundaries[:, :3].reshape(2, -1, 3) # convert back to 3d matrix with 3d points

            # trim openpilot lane boundaries
            openpilot_left_lane_boundary, openpilot_right_lane_boundary = shapely.linestrings(openpilot_lane_boundary_points)

            # make sure that lookahead distance does not go beyond the end of global path
            openpilot_left_end_dist = min(global_path.left_boundary.length - left_cur_pos_dist,  self.lookahead_distance)
            openpilot_right_end_dist = min(global_path.right_boundary.length - right_cur_pos_dist,  self.lookahead_distance)

            openpilot_left_lane_boundary = shapely.ops.substring(openpilot_left_lane_boundary, 0, openpilot_left_end_dist)
            openpilot_right_lane_boundary = shapely.ops.substring(openpilot_right_lane_boundary, 0, openpilot_right_end_dist)

            ##################################################################
            # Perform matching
            ##################################################################

            # calculate the average difference for right and left boundaries
            x, y = self.find_average_distance(map_left_lane_boundary, map_right_lane_boundary, 
                                                openpilot_left_lane_boundary, openpilot_right_lane_boundary, openpilot_lane_boundary_probs)

            weight = np.sqrt(np.sum(openpilot_lane_boundary_probs**2))

            # if the difference between map and openpilot lane boundaries is too big then don't use the correction
            if abs(x) > self.x_correction_treshold or abs(y) > self.y_correction_treshold or weight < self.probability_treshold:
                x, y = 0, 0
                weight = self.no_correction_weight
                no_correction = True
            else:
                no_correction = False       
            
            # use exponential moving average to smooth coordinate corrections 
            self.x_correction = self.calculate_updated_correction(x, self.x_correction, weight)
            self.y_correction = self.calculate_updated_correction(y, self.y_correction, weight)

            self.publish_base_link_correction_tf(correction_stamp=timestamp)

            ##################################################################
            # Visualization
            ##################################################################

            openpilot_color = ColorRGBA(0.5, 0.8, 0.7, 1.0) if no_correction else ColorRGBA(1.0, 1.0, 0.0, 1.0)
            map_color = ColorRGBA(0.6, 0.5, 0.4, 1.0) if no_correction else ColorRGBA(1.0, 1.0, 0.0, 1.0)

            lanes_marker_array = MarkerArray()
            marker1 = self.get_lane_boundary_marker(openpilot_right_lane_boundary, "right", 0, "map_gnss", openpilot_color, timestamp)
            marker2 = self.get_lane_boundary_marker(openpilot_left_lane_boundary, "left", 1, "map_gnss", openpilot_color, timestamp)
            lanes_marker_array.markers.append(marker1)
            lanes_marker_array.markers.append(marker2)

            marker3 = self.get_lane_boundary_marker(map_right_lane_boundary, "right", 2, "map", map_color, timestamp)
            marker4 = self.get_lane_boundary_marker(map_left_lane_boundary, "left", 3, "map", map_color, timestamp)
            lanes_marker_array.markers.append(marker3)
            lanes_marker_array.markers.append(marker4)
            
            self.lane_bound_markers_pub.publish(lanes_marker_array)
            self.publish_gnss_corrections_detailed()
        except Exception as e:
            rospy.logerr_throttle(10, "%s - Exception in callback: %s", rospy.get_name(), traceback.format_exc())

    def global_path_callback(self, msg):
        if len(msg.waypoints) == 0:
            self.global_path = None
            return
        
        self.global_path = PathWrapper(msg.waypoints, boundaries=True)

    def find_average_distance(self, map_left_lane_boundary, map_right_lane_boundary, openpilot_left_lane_boundary, openpilot_right_lane_boundary, probs=(1,1)):
        sample_point_count = len(openpilot_left_lane_boundary.coords)
        dists = np.linspace(0, self.lookahead_distance, sample_point_count)

        map_left_points = map_left_lane_boundary.interpolate(dists)
        map_right_points = map_right_lane_boundary.interpolate(dists)

        openpilot_left_points = openpilot_left_lane_boundary.interpolate(dists)
        openpilot_right_points = openpilot_right_lane_boundary.interpolate(dists)

        diffs_left = np.mean(shapely.get_coordinates(map_left_points) - shapely.get_coordinates(openpilot_left_points), axis=0)
        diffs_right = np.mean(shapely.get_coordinates(map_right_points) - shapely.get_coordinates(openpilot_right_points), axis=0)
        diffs_x, diffs_y = (probs[0]*diffs_left + probs[1]*diffs_right) / (probs[0] + probs[1])

        return diffs_x, diffs_y

    def get_lane_boundary_marker(self, lane_boundary, side, marker_id, frame_id, marker_color, stamp):
        points = []
        for x, y, z in lane_boundary.coords:
            point = Point(x=x,y=y, z=z)
            points.append(point)

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = f"{side} bound"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.color = marker_color
        marker.points = points

        return marker

    def publish_base_link_correction_tf(self, correction_stamp=None):
        t = TransformStamped()

        if correction_stamp is None:
            t.header.stamp = rospy.Time.now()
        else:
            t.header.stamp = correction_stamp
        t.header.frame_id = "map"
        t.child_frame_id = "map_gnss"

        t.transform.translation.x = self.x_correction
        t.transform.translation.y = self.y_correction
        t.transform.translation.z = self.z_correction
        t.transform.rotation.w = 1
        self.transform_matrix = numpify(t.transform)

        self.tf_broadcaster.sendTransform(t)

    def publish_gnss_corrections_detailed(self):

        gnss_corrections_detailed = OverlayText()

        x_correction_text = f"x correction: <span style='color: white;'>{self.x_correction:.5f} </span>\n"
        y_correction_text = f"y correction: <span style='color: white;'>{self.y_correction:.5f} </span>\n"
        z_correction_text = f"z correction: <span style='color: white;'>{self.z_correction:.5f} </span>"

        gnss_corrections_detailed.text = "<span style='font-style: bold; color: white;'>GNSS corrections:</span>\n" + x_correction_text + y_correction_text + z_correction_text

        self.gnss_corrections_detailed_pub.publish(gnss_corrections_detailed)

    def run(self):
        rospy.spin()

def float32_multiarray_to_numpy(multiarray):
    dims = tuple(map(lambda x: x.size, multiarray.layout.dim))
    data = multiarray.data[multiarray.layout.data_offset:]
    return np.array(data, dtype=np.float32).reshape(dims)

if __name__ == '__main__':
    rospy.init_node('lane_boundary_matcher')
    node = LaneBoundaryMatcher()
    node.run()