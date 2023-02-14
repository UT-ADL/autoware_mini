#!/usr/bin/env python

import math

import rospy
import threading
import message_filters

from std_msgs.msg import Bool, Header
from autoware_msgs.msg import VehicleCmd, VehicleStatus, Gear
from automotive_platform_msgs.msg import SpeedMode, SteerMode, TurnSignalCommand, GearCommand,\
     CurvatureFeedback, ThrottleFeedback, BrakeFeedback, GearFeedback, SteeringFeedback, VelocityAccelCov
from automotive_navigation_msgs.msg import ModuleState

LOW_SPEED_THRESH = 0.01

class SSCInterface:
    def __init__(self):
        self.publish_rate = rospy.get_param('publish_rate', 50)
        self.use_adaptive_gear_ratio = rospy.get_param('use_adaptive_gear_ratio', False)
        self.enable_reverse_motion = rospy.get_param('enable_reverse_motion', False)
        self.command_timeout = rospy.get_param('command_timeout', 200)
        self.wheel_base = rospy.get_param('wheel_base', 2.789)
        self.ssc_gear_ratio = rospy.get_param('ssc_gear_ratio', 16.135)
        self.acceleration_limit = rospy.get_param('acceleration_limit', 3.0)
        self.deceleration_limit = rospy.get_param('deceleration_limit', -3.0)
        self.max_curvature_rate = rospy.get_param('max_curvature_rate', 0.15)
        self.agr_coef_a = rospy.get_param('agr_coef_a', 15.713)
        self.agr_coef_b = rospy.get_param('agr_coef_b', 0.053)
        self.agr_coef_c = rospy.get_param('agr_coef_c', 0.042)

        self.desired_mode = 0
        self.desired_speed = 0.0
        self.desired_acceleration = 0.0
        self.desired_steering_angle = 0.0
        self.desired_curvature = 0.0
        self.desired_turn_signal = TurnSignalCommand.NONE
        self.desired_gear = Gear.NONE
        self.last_command_time = 0.0
        
        self.engage_sub = rospy.Subscriber('engage', Bool, self.engage_callback, queue_size=1)
        self.vehicle_cmd_sub = rospy.Subscriber('vehicle_cmd', VehicleCmd, self.vehicle_cmd_callback, queue_size=1)

        self.module_states_sub = rospy.Subscriber('ssc/module_states', ModuleState, self.module_states_callback, queue_size=1)
        self.curvature_feedback_sub = message_filters.Subscriber('ssc/curvature_feedback', CurvatureFeedback)
        self.throttle_feedback_sub = message_filters.Subscriber('ssc/throttle_feedback', ThrottleFeedback)
        self.brake_feedback_sub = message_filters.Subscriber('ssc/brake_feedback', BrakeFeedback)
        self.gear_feedback_sub = message_filters.Subscriber('ssc/gear_feedback', GearFeedback)
        self.steering_wheel_sub = message_filters.Subscriber('ssc/steering_feedback', SteeringFeedback)
        self.velocity_accel_sub = message_filters.Subscriber('ssc/velocity_accel_cov', VelocityAccelCov)
        self.ssc_feedbacks_sub = message_filters.ApproximateTimeSynchronizer([self.curvature_feedback_sub, self.throttle_feedback_sub,\
            self.brake_feedback_sub, self.gear_feedback_sub, self.steering_wheel_sub, self.velocity_accel_sub], queue_size=10, slop=0.04)
        self.ssc_feedbacks_sub.registerCallback(self.ssc_feedbacks_callback)

        self.speed_mode_pub = rospy.Publisher('ssc/arbitrated_speed_commands', SpeedMode, queue_size=1)
        self.steer_mode_pub = rospy.Publisher('ssc/arbitrated_steering_commands', SteerMode, queue_size=1)
        self.turn_signal_pub = rospy.Publisher('ssc/turn_signal_command', TurnSignalCommand, queue_size=1)
        self.gear_pub = rospy.Publisher('ssc/gear_select', GearCommand, queue_size=1)

    def engage_callback(self, msg):
        self.desired_mode = int(msg.data)

    def vehicle_cmd_callback(self, msg):
        self.last_command_time = rospy.get_time()

        is_valid_cmd = (msg.gear_cmd.gear in [Gear.DRIVE, Gear.LOW] and msg.ctrl_cmd.linear_velocity >= 0.0) or \
                       (msg.gear_cmd.gear == Gear.REVERSE and msg.ctrl_cmd.linear_velocity <= 0.0) or \
                       (msg.gear_cmd.gear == Gear.PARK and -LOW_SPEED_THRESH <= msg.ctrl_cmd.linear_velocity <= LOW_SPEED_THRESH) or \
                        msg.gear_cmd.gear == Gear.NONE
        if is_valid_cmd:
            self.desired_speed = abs(msg.ctrl_cmd.linear_velocity)
            self.desired_acceleration = abs(max(min(msg.ctrl_cmd.linear_acceleration, self.acceleration_limit), self.deceleration_limit))
        else:
            rospy.logwarn("Invalid vehicle command: gear = %d, velocity = %lf", msg.gear_cmd.gear, msg.ctrl_cmd.linear_velocity)
            rospy.logwarn("Disengaging autonomy")
            self.desired_mode = 0
            self.desired_speed = 0.0
            self.desired_acceleration = 0.0

        if self.use_adaptive_gear_ratio:
            self.desired_steering_angle = msg.ctrl_cmd.steering_angle * self.ssc_gear_ratio / self.adaptive_gear_ratio
        else:
            self.desired_steering_angle = msg.ctrl_cmd.steering_angle

        self.desired_curvature = math.tan(self.desired_steering_angle) / self.wheel_base

        self.desired_turn_signal = TurnSignalCommand.NONE
        if msg.lamp_cmd.l == 0 and msg.lamp_cmd.r == 0:
            self.desired_turn_signal = TurnSignalCommand.NONE
        elif msg.lamp_cmd.l == 1 and msg.lamp_cmd.r == 0:
            self.desired_turn_signal = TurnSignalCommand.LEFT
        elif msg.lamp_cmd.l == 0 and msg.lamp_cmd.r == 1:
            self.desired_turn_signal = TurnSignalCommand.RIGHT
        elif msg.lamp_cmd.l == 1 and msg.lamp_cmd.r == 1:
            # HAZARD signal cannot be ised in TurnSignalCommand
            pass

        if msg.emergency == 1:
            rospy.logerr("Emergency stopping, speed overridden to 0")
            self.desired_speed = 0.0

    def module_states_callback(self, msg):
        if 'veh_controller' in msg.name:
            if msg.state == 'active':
                self.current_mode = VehicleStatus.MODE_AUTO
            else:
                self.current_mode = VehicleStatus.MODE_MANUAL
            
            if msg.state in ('failure', 'fatal', 'not_ready'):
                self.desired_mode = 0

    def ssc_feedbacks_callback(self, curvature_msg, throttle_msg, brake_msg, gear_msg, steering_msg, velocity_accel_msg):
        self.adaptive_gear_ratio = self.agr_coef_a + self.agr_coef_b * velocity_accel_msg.velocity**2 - self.agr_coef_c * steering_msg.steering_wheel_angle
        print("ssc feedbacks callback")

    def publish_speed_command(self, header):
        msg = SpeedMode(header = header)
        msg.mode = self.desired_mode
        msg.speed = self.desired_speed
        msg.acceleration_limit = self.desired_acceleration
        msg.deceleration_limit = self.desired_acceleration
        self.speed_mode_pub.publish(msg)

    def publish_steer_command(self, header):
        msg = SteerMode(header = header)
        msg.mode = self.desired_mode
        msg.curvature = self.desired_curvature
        msg.max_curvature_rate = self.max_curvature_rate
        self.steer_mode_pub.publish(msg)

    def publish_turn_command(self, header):
        msg = TurnSignalCommand(header = header)
        msg.mode = self.desired_mode
        msg.turn_signal = self.desired_turn_signal
        self.turn_signal_pub.publish(msg)

    def publish_gear_command(self, header):
        msg = GearCommand(header = header)
        msg.command.gear = self.desired_gear
        self.gear_pub.publish(msg)

    def publisher(self):
        # publish commands at fixed rate
        rate = rospy.Rate(self.publish_rate)

        while not rospy.is_shutdown():
            # turn off autonomy when haven't received vehicle command some time
            time_from_last_command = (rospy.get_time() - self.last_command_time) * 1000
            if self.desired_mode == 1 and time_from_last_command > self.command_timeout:
                rospy.logerr("Did not receive any commands for %d ms", self.command_timeout)
                rospy.logerr("Disengaging autonomy until re-enabled")
                self.desired_mode = 0

            # publish command messages at fixed rate to keep SSC alive
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'base_link'
            self.publish_speed_command(header)
            self.publish_steer_command(header)
            self.publish_turn_command(header)
            self.publish_gear_command(header)

            rate.sleep()

    def run(self):
        # start separate thread for publishing
        threading.Thread(target=self.publisher).start()
        # keep the main thread for subscribers
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('ssc_interface', log_level=rospy.INFO)
    node = SSCInterface()
    node.run()
