#!/usr/bin/env python3

import rospy
import tf2_ros
import pyclothoids
import shapely
from copy import deepcopy

from geometry_msgs.msg import Point, PointStamped, PoseStamped, TwistStamped
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA, Bool
from autoware_mini.msg import Path, Waypoint
from std_srvs.srv import EmptyResponse, Empty

from autoware_mini.transform import transform_point, get_origin_point
from autoware_mini.geometry import get_heading_between_two_points, get_distance_between_two_points_2d
from autoware_mini.visualization import triangulate_linestring


class WaypointAssistance:

    def __init__(self):

        # Parameters
        self.assist_goal_clear_distance = rospy.get_param('~assist_goal_clear_distance')
        self.assistance_speed = rospy.get_param('~assistance_speed') / 3.6 # convert km/h to m/s
        self.half_waypoint_size = rospy.get_param('~waypoint_size') / 2.0
        self.half_drawing_area_size = rospy.get_param('~drawing_area_size') / 2.0
        self.ego_vehicle_stopped_speed_limit = rospy.get_param('ego_vehicle_stopped_speed_limit')

        # tf buffer to convert coordinates (if needed) for clicked points
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)

        # state: 'idle' | 'stopping' | 'drawing' | 'driving'
        self.state = 'idle'

        # Keep track of paths and pose
        self.original_global_path = Path()
        self.waypoints = [] # sparse waypoints
        self.assisted_path = Path() # clothoid waypoints
        self.assisted_path_linestring = None # 2D linestring of assisted_path, for arc-length end-of-path check
        self.current_speed = 0.0
        # While ignore-object pick is armed, skip waypoint placement so ignored_objects_filter handles the click
        self.ignore_object_enabled = False

        # Publisher: forward incoming lane change paths to the assistance topic
        self.assisted_path_pub = rospy.Publisher('assisted_path', Path, queue_size=10, latch=True, tcp_nodelay=True)
        # Marker publisher for visualizing temporary waypoints and path
        self.markers_pub = rospy.Publisher('assisted_path_markers', MarkerArray, queue_size=10, latch=True, tcp_nodelay=True)
        # Assistance state publisher so other nodes (e.g., obstacle_simulation, drivemode_visualizer) can react
        self.assistance_state_pub = rospy.Publisher('assistance_enabled', Bool, queue_size=1, latch=True, tcp_nodelay=True)

        # Services for action buttons
        self.srv_start = rospy.Service('service_start_assistance', Empty, self.service_start_assistance)
        self.srv_proceed = rospy.Service('service_proceed_driving', Empty, self.service_proceed_driving)
        self.srv_finish = rospy.Service('service_finish_assistance', Empty, self.service_finish_assistance)

        # Subscriber: listen to lane change planner output and forward it
        rospy.Subscriber('lane_change_global_path', Path, self.global_path_callback, tcp_nodelay=True)
        # Subscribe to clicked points in RViz to draw waypoints
        rospy.Subscriber('/clicked_point', PointStamped, self.clicked_point_callback, tcp_nodelay=True)
        # Subscribe to the vehicle's pose and speed
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/localization/current_velocity', TwistStamped, self.current_velocity_callback, queue_size=1, tcp_nodelay=True)
        # Suppress waypoint placement while ignore-object pick is armed
        rospy.Subscriber('/perception/ignore_object_enabled', Bool, self.ignore_object_state_callback, queue_size=1, tcp_nodelay=True)


    def global_path_callback(self, msg):
        self.original_global_path = msg
        if self.state == 'idle':
            self.assisted_path_pub.publish(msg)


    def service_start_assistance(self, req=None):

        if self.current_speed >= self.ego_vehicle_stopped_speed_limit and self.state != 'stopping':
            if self.state == 'idle':
                # Make copy of the original global path
                self.assisted_path = deepcopy(self.original_global_path)

            # Make the car stop, either on global path or assisted path
            for wp in self.assisted_path.waypoints:
                wp.speed = 0.0

            self.assisted_path_pub.publish(self.assisted_path)
            self.state = 'stopping'
            self.assistance_state_pub.publish(Bool(True))

            return EmptyResponse()

        self.waypoints = []
        self.assisted_path = Path()
        self.assisted_path.header.frame_id = 'map'
        self.assisted_path.header.stamp = rospy.Time.now()
        self.assisted_path_linestring = None

        # Clear the global path and existing markers and notify other nodes that assistance is enabled
        self.assisted_path_pub.publish(self.assisted_path)
        self.state = 'drawing'
        self.publish_assistance_markers()
        self.assistance_state_pub.publish(Bool(True))

        return EmptyResponse()


    def service_proceed_driving(self, req=None):
        if self.state == 'driving':
            rospy.logwarn("Already in driving mode.")
            return EmptyResponse()

        if self.state != 'drawing':
            rospy.logerr("Cannot proceed to driving mode when not in drawing mode.")
            return EmptyResponse()

        if not self.waypoints:
            rospy.logerr("Cannot proceed to driving mode with an empty assisted path.")
            return EmptyResponse()

        self.assisted_path_pub.publish(self.assisted_path)
        self.state = 'driving'

        return EmptyResponse()


    def service_finish_assistance(self, req=None):
        if self.state == 'idle':
            rospy.logerr("Cannot finish assistance when not in assistance mode.")
            return EmptyResponse()

        # Clear temporary path and markers
        self.waypoints = []
        self.assisted_path = Path()
        self.assisted_path_linestring = None

        # Restore original global path
        self.assisted_path_pub.publish(self.original_global_path)
        self.state = 'idle'
        self.publish_assistance_markers()  # will clear markers
        self.assistance_state_pub.publish(Bool(False))

        return EmptyResponse()

    def current_velocity_callback(self, msg):
        self.current_speed = msg.twist.linear.x
        if self.current_speed < self.ego_vehicle_stopped_speed_limit and self.state == 'stopping':
            self.service_start_assistance()

    def current_pose_callback(self, msg):
        if self.state == 'driving' and self.assisted_path_linestring is not None:
            car_point = shapely.points(msg.pose.position.x, msg.pose.position.y)
            if self.assisted_path_linestring.length - self.assisted_path_linestring.project(car_point) <= self.assist_goal_clear_distance:
                self.service_start_assistance()

    def ignore_object_state_callback(self, msg):
        self.ignore_object_enabled = msg.data

    def clicked_point_callback(self, msg):
        # Called when operator clicks Publish Point in RViz
        # Ignore-object pick is armed: let ignored_objects_filter consume the click
        if self.ignore_object_enabled:
            return

        if self.state not in ('drawing', 'driving'):
            return

        # Ensure point is in 'map' frame
        if msg.header.frame_id != 'map':
            transform = self.tf_buffer.lookup_transform('map', msg.header.frame_id, msg.header.stamp, rospy.Duration(0.06))
            msg.point = transform_point(transform, msg.point)

        for i, wp in enumerate(self.waypoints):
            if abs(msg.point.x - wp.position.x) <= self.half_waypoint_size and abs(msg.point.y - wp.position.y) <= self.half_waypoint_size:
                # When clicking on the first two waypoints or if there are 3 or fewer waypoints, reset the assistance
                if i <= 1 or len(self.waypoints) <= 3:
                    self.service_start_assistance()
                    return
                # Remove the clicked waypoint
                self.waypoints.pop(i)
                break
        else:
            # If the clicked point is not close to any existing waypoints, add it as a new waypoint
            base_footprint_point = get_origin_point(self.tf_buffer, 'map', 'base_footprint')

            # If this is the first point, prepend current vehicle position as the first waypoint
            if not self.waypoints:
                base_wp = Waypoint()
                base_wp.position = base_footprint_point
                base_wp.speed = self.assistance_speed
                base_wp.lanechange_state = 0
                base_wp.turn_signal = Waypoint.TURN_STRAIGHT
                self.waypoints.append(base_wp)

                front_wp = Waypoint()
                front_wp.position = get_origin_point(self.tf_buffer, 'map', 'car_front')
                front_wp.position.z = base_footprint_point.z
                front_wp.speed = self.assistance_speed
                front_wp.lanechange_state = 0
                front_wp.turn_signal = Waypoint.TURN_STRAIGHT
                self.waypoints.append(front_wp)

            # Create waypoint from clicked point and append
            wp = Waypoint()
            wp.position.x = msg.point.x
            wp.position.y = msg.point.y
            wp.position.z = msg.point.z if msg.point.z != 0.0 else base_footprint_point.z
            wp.speed = self.assistance_speed
            wp.lanechange_state = 0
            wp.turn_signal = Waypoint.TURN_STRAIGHT

            self.waypoints.append(wp)

        # Update assisted path preview and markers
        self.recompute_assisted_path()

        # Update markers to visualize waypoints and temporary path
        self.publish_assistance_markers()

        if self.state == 'driving':
            self.assisted_path_pub.publish(self.assisted_path)


    def publish_assistance_markers(self):
        # Publish MarkerArray representing the temporary waypoints and path visualization
        marker_array = MarkerArray()

        # Clear previously-published markers; subsequent ADDs below reconstitute the current view.
        delete_all_marker = Marker()
        delete_all_marker.header.frame_id = 'map'
        delete_all_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_all_marker)

        # Add transparent square marker to make the surroundings of the vehicle clickable
        if self.state in ('drawing', 'driving'):
            square_marker = Marker()
            square_marker.header.frame_id = 'base_footprint'
            square_marker.header.stamp = rospy.Time(0)
            square_marker.ns = 'Drawing Area'
            square_marker.id = 0
            square_marker.type = Marker.TRIANGLE_LIST
            square_marker.action = Marker.ADD
            square_marker.frame_locked = True
            square_marker.scale.x = 1.0
            square_marker.scale.y = 1.0
            square_marker.scale.z = 1.0
            square_marker.color = ColorRGBA(0.2, 0.6, 1.0, 0.0) # invisible marker for RViz
            square_marker.points = [
                Point(x=-self.half_drawing_area_size, y=-self.half_drawing_area_size, z=0.0), Point(x= self.half_drawing_area_size, y=-self.half_drawing_area_size, z=0.0), Point(x= self.half_drawing_area_size, y= self.half_drawing_area_size, z=0.0),
                Point(x=-self.half_drawing_area_size, y=-self.half_drawing_area_size, z=0.0), Point(x= self.half_drawing_area_size, y= self.half_drawing_area_size, z=0.0), Point(x=-self.half_drawing_area_size, y= self.half_drawing_area_size, z=0.0)
            ]
            marker_array.markers.append(square_marker)

        # Create waypoint markers
        for i, waypoint in enumerate(self.waypoints):
            z = waypoint.position.z + 0.1
            half = self.half_waypoint_size
            box_marker = Marker()
            box_marker.header.frame_id = self.assisted_path.header.frame_id
            box_marker.header.stamp = rospy.Time.now()
            box_marker.ns = 'Border'
            box_marker.id = i
            box_marker.type = Marker.LINE_STRIP
            box_marker.action = Marker.ADD
            box_marker.scale.x = 0.2 # line width in meters
            box_marker.pose.orientation.w = 1.0
            box_marker.color = ColorRGBA(1.0, 1.0, 0.0, 0.9)
            box_marker.points = [
                Point(x=waypoint.position.x - half, y=waypoint.position.y - half, z=z),
                Point(x=waypoint.position.x - half, y=waypoint.position.y + half, z=z),
                Point(x=waypoint.position.x + half, y=waypoint.position.y + half, z=z),
                Point(x=waypoint.position.x + half, y=waypoint.position.y - half, z=z),
                Point(x=waypoint.position.x - half, y=waypoint.position.y - half, z=z),
            ]
            marker_array.markers.append(box_marker)

        # Path lines
        if len(self.waypoints) >= 3:
            linestring = shapely.linestrings([(wp.position.x, wp.position.y, wp.position.z) for wp in self.assisted_path.waypoints])
            linestring = linestring.simplify(0.05)
            triangle_points = triangulate_linestring(linestring, 1.5, z_offset=0.05)

            line_marker = Marker()
            line_marker.header.frame_id = self.assisted_path.header.frame_id
            line_marker.header.stamp = rospy.Time.now()
            line_marker.ns = 'Path'
            line_marker.id = 0
            line_marker.type = line_marker.TRIANGLE_LIST
            line_marker.action = line_marker.ADD
            line_marker.scale.x = 1.0
            line_marker.scale.y = 1.0
            line_marker.scale.z = 1.0
            line_marker.pose.orientation.w = 1.0
            line_marker.color = ColorRGBA(0.7, 0.7, 0.6, 0.7)
            line_marker.colors = [line_marker.color] * len(triangle_points)
            line_marker.points = triangle_points
            marker_array.markers.append(line_marker)

        self.markers_pub.publish(marker_array)


    def recompute_assisted_path(self):
        self.assisted_path.waypoints = []
        heading_start = get_heading_between_two_points(self.waypoints[0].position, self.waypoints[1].position)
        for i in range(len(self.waypoints) - 1):
            if i < len(self.waypoints) - 2:
                heading_next = get_heading_between_two_points(self.waypoints[i+1].position, self.waypoints[i+2].position)
            else:
                heading_next = heading_start
            dist = get_distance_between_two_points_2d(self.waypoints[i].position, self.waypoints[i+1].position)

            # Number of samples is the distance in meters, minimum 2
            n_samples = max(int(dist), 2)
            clothoid = pyclothoids.Clothoid.G1Hermite(
                self.waypoints[i].position.x,
                self.waypoints[i].position.y,
                heading_start,
                self.waypoints[i+1].position.x,
                self.waypoints[i+1].position.y,
                heading_next
            )
            xs, ys = clothoid.SampleXY(n_samples)
            for x, y in zip(xs, ys):
                wp = Waypoint()
                wp.position.x = x
                wp.position.y = y
                wp.position.z = self.waypoints[i].position.z
                wp.speed = self.waypoints[i].speed
                wp.lanechange_state = self.waypoints[i].lanechange_state
                wp.turn_signal = self.waypoints[i].turn_signal
                self.assisted_path.waypoints.append(wp)
            heading_start = heading_next

        self.assisted_path_linestring = shapely.linestrings([(wp.position.x, wp.position.y) for wp in self.assisted_path.waypoints])


    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('waypoint_assistance')
    node = WaypointAssistance()
    node.run()
