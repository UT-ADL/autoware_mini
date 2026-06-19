#!/usr/bin/env python3
"""
Propagate the ego pose to Carla and optionally follow with the spectator camera.

In VIL mode, offsets current_pose by the static base_link -> ego_vehicle
transform to teleport the Carla actor and broadcast map -> ego_vehicle.
When spectator following is enabled, also computes and publishes the
spectator camera pose from the same callback.
"""

import math

import rospy
import tf2_ros

from tf.transformations import quaternion_from_euler, euler_from_quaternion

from geometry_msgs.msg import Pose, Point, PoseStamped, Quaternion

from autoware_mini.transform import transform_pose, transform_to_pose, pose_to_transform


class CarlaPoseRelay:
    def __init__(self):

        self.use_vil = rospy.get_param("~use_vil")
        self.spectator_follow_ego = rospy.get_param("~spectator_follow_ego")
        if self.spectator_follow_ego:
            # static ego -> spectator offset: behind and above the ego, pitched down toward it
            distance_behind = rospy.get_param("~spectator_distance_behind")
            height_above = rospy.get_param("~spectator_height_above")
            pitch = rospy.get_param("~spectator_pitch")
            qx, qy, qz, qw = quaternion_from_euler(0.0, math.radians(-pitch), 0.0)
            self.ego_to_spectator = Pose(position=Point(x=-distance_behind, z=height_above),
                                         orientation=Quaternion(x=qx, y=qy, z=qz, w=qw))

        # stamp of the last broadcast map -> ego_vehicle, to skip redundant TFs (see callback)
        self.last_tf_stamp = None

        tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(tf_buffer)

        # publishers
        if self.use_vil:
            self.br = tf2_ros.TransformBroadcaster()
            self.ego_teleport_pub = rospy.Publisher('/carla/ego_vehicle/control/set_transform', Pose, queue_size=1, tcp_nodelay=True)
        if self.spectator_follow_ego:
            self.spectator_pub = rospy.Publisher('/carla/spectator/set_transform', Pose, queue_size=1, tcp_nodelay=True)

        # static offset of the Carla ego actor (vehicle center) from base_link, taken from Carla's frames
        transform = tf_buffer.lookup_transform("base_link", "ego_vehicle", rospy.Time(0), rospy.Duration(100))
        self.base_link_to_ego = transform_to_pose(transform)

        # subscribers
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1, tcp_nodelay=True)
        rospy.loginfo("%s - initialized", rospy.get_name())

    def current_pose_callback(self, msg):
        # map -> ego_vehicle = (map -> base_link from current_pose) * (base_link -> ego_vehicle)
        ego_pose = transform_pose(pose_to_transform(msg.pose), self.base_link_to_ego)

        if self.use_vil:
            # teleport the Carla ego to the ego pose
            self.ego_teleport_pub.publish(ego_pose)

            # broadcast map -> ego_vehicle to root Carla's sensor frames in map. During bag playback
            # the clock doesn't advance while waiting to start, so multiple messages can share a stamp;
            # broadcast only once per stamp to avoid TF_REPEATED_DATA
            if msg.header.stamp != self.last_tf_stamp:
                self.last_tf_stamp = msg.header.stamp
                t = pose_to_transform(ego_pose)
                t.header.stamp = msg.header.stamp
                t.header.frame_id = "map"
                t.child_frame_id = "ego_vehicle"
                self.br.sendTransform(t)

        if self.spectator_follow_ego:
            # follow the ego heading and pitch but keep the camera level (zero roll)
            o = ego_pose.orientation
            _, pitch, yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])
            o.x, o.y, o.z, o.w = quaternion_from_euler(0.0, pitch, yaw)
            # map -> spectator = (map -> ego_vehicle, leveled) * (ego -> spectator)
            self.spectator_pub.publish(transform_pose(pose_to_transform(ego_pose), self.ego_to_spectator))

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('carla_pose_relay', log_level=rospy.INFO)
    node = CarlaPoseRelay()
    node.run()
