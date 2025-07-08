#!/usr/bin/env python3

import rospy
import shapely
import numpy as np
from autoware_mini.msg import Path, DetectedObjectArray
from sensor_msgs.msg import PointCloud2
from autoware_mini.geometry import get_speed_from_velocity
from autoware_mini.collision import CollisionPoints

class ObjectCollisionChecker:

    def __init__(self):

        # parameters
        self.safety_box_width = rospy.get_param("safety_box_width")
        self.stopped_speed_limit = rospy.get_param("stopped_speed_limit")
        self.braking_safety_distance_obstacle = rospy.get_param("~braking_safety_distance_obstacle")

        # variables
        self.detected_objects = None

        # publishers
        self.local_path_collision_pub = rospy.Publisher('object_collision_points', PointCloud2, queue_size=1, tcp_nodelay=True)

        # subscribers
        rospy.Subscriber('extracted_local_path', Path, self.path_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/detection/tracked_objects', DetectedObjectArray, self.detected_objects_callback, queue_size=1, buff_size=2**20, tcp_nodelay=True)

    def detected_objects_callback(self, msg):
        self.detected_objects = msg.objects

    def path_callback(self, msg):

        detected_objects = self.detected_objects
        collision_points = CollisionPoints()

        if detected_objects is None:
            rospy.logwarn_throttle(3, "%s - detected objects not received!", rospy.get_name())
            return

        if len(msg.waypoints) > 0 and len(detected_objects) > 0:
            local_path_linestring = shapely.LineString([(waypoint.position.x, waypoint.position.y) for waypoint in msg.waypoints])

            # create buffer around local path
            local_path_buffer = local_path_linestring.buffer(self.safety_box_width / 2, cap_style="flat")
            shapely.prepare(local_path_buffer)

            for obj in detected_objects:
                # get the convex hulls and store as shapely polygons
                object_polygon = shapely.polygons(np.array(obj.convex_hull).reshape(-1, 3))

                if local_path_buffer.intersects(object_polygon):
                    intersection_result = object_polygon.intersection(local_path_buffer)
                    intersection_points = shapely.get_coordinates(intersection_result)
                    object_speed = get_speed_from_velocity(obj.velocity)

                    collision_points.add_intersection_points(intersection_points,
                                                            z = obj.center.z - obj.dimensions.z / 2,
                                                            vx = obj.velocity.x,
                                                            vy = obj.velocity.y,
                                                            vz = obj.velocity.z,
                                                            distance_to_stop = self.braking_safety_distance_obstacle,
                                                            deceleration_limit = np.inf,
                                                            category = CollisionPoints.STOPPED_OBSTACLE_ON_PATH if object_speed < self.stopped_speed_limit else CollisionPoints.MOVING_OBSTACLE_ON_PATH)

        collision_points_msg = collision_points.create_message()
        collision_points_msg.header = msg.header
        self.local_path_collision_pub.publish(collision_points_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('object_collision_checker')
    node = ObjectCollisionChecker()
    node.run()