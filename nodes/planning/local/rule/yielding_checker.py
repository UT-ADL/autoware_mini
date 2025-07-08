#!/usr/bin/env python3

import rospy
import shapely
import numpy as np
from autoware_mini.msg import Path, DetectedObjectArray
from sensor_msgs.msg import PointCloud2
from autoware_mini.collision import CollisionPoints
from autoware_mini.lanelet2 import load_lanelet2_map, get_stop_lines_using_subtype
from autoware_mini.path import PathWrapper

class YieldingChecker:

    def __init__(self):

        # parameters
        self.safety_box_width = rospy.get_param("safety_box_width")
        self.braking_safety_distance_yield = rospy.get_param("~braking_safety_distance_yield")
        self.yielding_maximum_deceleration = rospy.get_param("~yielding_maximum_deceleration")
        self.yielding_distance_limit = rospy.get_param("~yielding_distance_limit")
        self.heading_alignment_limit = rospy.get_param("~heading_alignment_limit")
        self.use_object_width = rospy.get_param("use_object_width")
        lanelet2_map_path = rospy.get_param("~lanelet2_map_path")

        # variables
        self.objects = None
        self.yield_lines_on_global_path = []

        lanelet2_map = load_lanelet2_map(lanelet2_map_path)
        yield_lines = get_stop_lines_using_subtype(lanelet2_map, subtypes=["yield", "yield_stop"])
        self.yield_lines = np.array(list(yield_lines.values()))

        # publishers
        self.local_path_collision_pub = rospy.Publisher('yielding_collision_points', PointCloud2, queue_size=1, tcp_nodelay=True)

        # subscribers
        rospy.Subscriber('/detection/predicted_objects_map', DetectedObjectArray, self.predicted_objects_callback, queue_size=1, buff_size=2**20, tcp_nodelay=True)
        rospy.Subscriber('global_path', Path, self.global_path_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('extracted_local_path', Path, self.local_path_callback, queue_size=1, tcp_nodelay=True)

    def predicted_objects_callback(self, msg):
        self.objects = msg.objects

    def global_path_callback(self, msg):
        global_path = PathWrapper(msg.waypoints)

        mask = global_path.linestring.intersects(self.yield_lines)
        self.yield_lines_on_global_path = self.yield_lines[mask]

    def local_path_callback(self, msg):

        objects = self.objects
        yield_lines_on_global_path = self.yield_lines_on_global_path

        if objects is None:
            rospy.logwarn_throttle(3, "%s - detected objects not received!", rospy.get_name())
            return

        collision_points = CollisionPoints()

        if len(msg.waypoints) > 0 and len(objects) > 0 and len(yield_lines_on_global_path) > 0:
            local_path = PathWrapper(msg.waypoints)
            local_path_buffer = local_path.linestring.buffer(self.safety_box_width / 2, cap_style="flat")
            shapely.prepare(local_path_buffer)

            mask = local_path.linestring.intersects(yield_lines_on_global_path)
            yield_lines_on_local_path = yield_lines_on_global_path[mask]
            if len(yield_lines_on_local_path) > 0:
                intersection_points = local_path.linestring.intersection(yield_lines_on_local_path)
                intersection_distances = local_path.linestring.project(intersection_points)
                # get the closest yield line
                min_distance_index = np.argmin(intersection_distances)
                yield_line_distance = intersection_distances[min_distance_index]
                yield_line_point = intersection_points[min_distance_index]

                yielding_found = False
                for obj in objects:
                    for path in obj.candidate_trajectories.paths:

                        trajectory_to_check = PathWrapper(path.waypoints).linestring

                        if self.use_object_width:
                            trajectory_to_check = trajectory_to_check.buffer(obj.dimensions.y / 2, cap_style="flat")

                        if local_path_buffer.intersects(trajectory_to_check):
                            trajectory_intersection_result = trajectory_to_check.intersection(local_path_buffer)
                            trajectory_intersection_coords = shapely.get_coordinates(trajectory_intersection_result)
                            trajectory_intersection_points = shapely.points(trajectory_intersection_coords)
                            trajectory_intersection_distance = min(local_path.linestring.project(trajectory_intersection_points))

                            # Intersection not in the reasonable range - after yield line and within 40m limits
                            if trajectory_intersection_distance < yield_line_distance or trajectory_intersection_distance - yield_line_distance > self.yielding_distance_limit:
                                continue

                            # Do not yield if object itself is on the local path
                            object_polygon = shapely.polygons([np.array(obj.convex_hull).reshape(-1, 3)])
                            if local_path_buffer.intersects(object_polygon):
                                continue

                            # Yielding to all that are left
                            collision_points.add_point(x = yield_line_point.x,
                                                    y = yield_line_point.y,
                                                    z = yield_line_point.z,
                                                    vx = 0.0,
                                                    vy = 0.0, 
                                                    vz = 0.0,
                                                    distance_to_stop = self.braking_safety_distance_yield,
                                                    deceleration_limit = self.yielding_maximum_deceleration,
                                                    category = CollisionPoints.YIELDING_TRAJECTORY)
                            # if one found then break the path and object loops
                            yielding_found = True
                            break
                    if yielding_found:
                        break

        collision_points_msg = collision_points.create_message()
        collision_points_msg.header = msg.header
        self.local_path_collision_pub.publish(collision_points_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('yielding_checker')
    node = YieldingChecker()
    node.run()