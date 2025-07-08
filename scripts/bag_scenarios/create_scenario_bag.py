#!/usr/bin/env python

import rospy
import rosbag
import argparse
from geometry_msgs.msg import PoseWithCovarianceStamped

def process_rosbags(args):
    first_pose = None
    last_pose = None
    first_velocity = None
    first_pose_time = None
    goal_time = None

    # Open the input bag for reading
    with rosbag.Bag(args.input_bag, 'r') as in_bag:
        # Determine the start time of the bag
        bag_start_time = in_bag.get_start_time()
        start_time = rospy.Time(bag_start_time + args.start_time) if args.start_time is not None else None
        end_time = rospy.Time(bag_start_time + args.end_time) if args.end_time is not None else None

        for topic, msg, t in in_bag.read_messages(topics=['/localization/current_pose', '/localization/current_velocity']):
            if start_time and t < start_time:
                continue
            if end_time and t > end_time:
                break

            if topic == '/localization/current_pose':
                if first_pose is None:
                    first_pose = msg
                    first_pose_time = t
                    goal_time = t + rospy.Duration(args.goal_delay)
                last_pose = msg
            elif topic == '/localization/current_velocity':
                if first_velocity is None:
                    first_velocity = msg

        if first_pose is None or first_velocity is None:
            print("No localization messages found in the bag.")
            return

        # Open the output bag for writing
        with rosbag.Bag(args.output_bag, 'w', rosbag.Compression.BZ2) as out_bag:
            if first_pose:
                initialpose = PoseWithCovarianceStamped()
                initialpose.header = first_pose.header
                initialpose.pose.pose = first_pose.pose
                out_bag.write('/initialpose', initialpose, first_pose_time)

            if first_velocity:
                out_bag.write('/initialvelocity', first_velocity, first_pose_time)

            # Copy relevant topics from the input bag to the output bag
            for topic, msg, t in in_bag.read_messages(topics=[args.detected_objects_topic, args.traffic_light_status_topic, '/tf']):
                if start_time and t < start_time:
                    continue
                if end_time and t > end_time:
                    break

                if last_pose and t >= goal_time:
                    out_bag.write('/move_base_simple/goal', last_pose, t)
                    last_pose = None

                if topic == args.detected_objects_topic:
                    topic = '/detection/detected_objects'
                elif topic == args.traffic_light_status_topic:
                    topic = '/detection/traffic_light_status'
                elif topic == '/tf':
                    for transform in msg.transforms:
                        if transform.child_frame_id == 'base_link':
                            transform.child_frame_id = 'lexus_model'

                out_bag.write(topic, msg, t)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Process ROS bag files.")
    parser.add_argument("input_bag", type=str, help="Input bag file path")
    parser.add_argument("output_bag", type=str, help="Output bag file path")
    parser.add_argument("--start_time", type=float, help="Start time in seconds (optional)")
    parser.add_argument("--end_time", type=float, help="End time in seconds (optional)")
    parser.add_argument("--goal_delay", type=float, default=0.1, help="Delay goal from start time (default: 0.1)")
    parser.add_argument("--detected_objects_topic", default="/detection/detected_objects")
    parser.add_argument("--traffic_light_status_topic", default="/detection/traffic_light_status")
    
    args = parser.parse_args()
    process_rosbags(args)
