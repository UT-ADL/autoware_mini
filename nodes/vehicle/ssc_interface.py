#!/usr/bin/env python

import math
import sys

import rospy
import message_filters

from std_msgs.msg import Bool, Header
from autoware_msgs.msg import VehicleCmd, VehicleStatus, Gear
from automotive_platform_msgs.msg import SpeedMode, SteerMode, TurnSignalCommand, GearCommand,\
     CurvatureFeedback, ThrottleFeedback, BrakeFeedback, GearFeedback, SteeringFeedback, VelocityAccelCov
from automotive_navigation_msgs.msg import ModuleState
from pacmod_msgs.msg import SystemRptInt

LOW_SPEED_THRESH = 0.01

TURN_RPT_TO_VEHICLE_STATUS_LAMP_MAP = {
    SystemRptInt.TURN_NONE: 0,
    SystemRptInt.TURN_LEFT: VehicleStatus.LAMP_LEFT,
    SystemRptInt.TURN_RIGHT: VehicleStatus.LAMP_RIGHT,
    SystemRptInt.TURN_HAZARDS: VehicleStatus.LAMP_HAZARD
}

class SSCInterface:
    def __init__(self):
        # get parameters
        self.use_adaptive_gear_ratio = rospy.get_param('use_adaptive_gear_ratio', True)
        self.enable_reverse_motion = rospy.get_param('enable_reverse_motion', False)
        self.command_timeout = rospy.get_param('command_timeout', 200)
        self.wheel_base = rospy.get_param('wheel_base', 2.789)
        self.ssc_gear_ratio = rospy.get_param('ssc_gear_ratio', 16.135)
        self.acceleration_limit = rospy.get_param('acceleration_limit', 3.0)
        self.deceleration_limit = rospy.get_param('deceleration_limit', 3.0)
        self.max_curvature_rate = rospy.get_param('max_curvature_rate', 0.15)
        self.agr_coef_a = rospy.get_param('agr_coef_a', 15.713)
        self.agr_coef_b = rospy.get_param('agr_coef_b', 0.053)
        self.agr_coef_c = rospy.get_param('agr_coef_c', 0.042)

        # initialize variables
        self.engage = False
        self.dbw_enabled = False
        self.adaptive_gear_ratio = self.ssc_gear_ratio
        
        # initialize command subscribers
        self.engage_sub = rospy.Subscriber('engage', Bool, self.engage_callback, queue_size=1)
        self.vehicle_cmd_sub = rospy.Subscriber('vehicle_cmd', VehicleCmd, self.vehicle_cmd_callback, queue_size=1)

        # initialize SSC feedback subscribers
        self.module_states_sub = rospy.Subscriber('/ssc/module_states', ModuleState, self.module_states_callback, queue_size=1)
        self.curvature_feedback_sub = message_filters.Subscriber('/ssc/curvature_feedback', CurvatureFeedback)
        self.throttle_feedback_sub = message_filters.Subscriber('/ssc/throttle_feedback', ThrottleFeedback)
        self.brake_feedback_sub = message_filters.Subscriber('/ssc/brake_feedback', BrakeFeedback)
        self.gear_feedback_sub = message_filters.Subscriber('/ssc/gear_feedback', GearFeedback)
        self.steering_wheel_sub = message_filters.Subscriber('/ssc/steering_feedback', SteeringFeedback)
        self.velocity_accel_sub = message_filters.Subscriber('/ssc/velocity_accel_cov', VelocityAccelCov)
        # take turn signal info from Pacmod, because it is not available from SSC
        self.turn_rpt_sub = message_filters.Subscriber('/pacmod/parsed_tx/turn_rpt', SystemRptInt)
        self.ssc_feedbacks_sub = message_filters.ApproximateTimeSynchronizer([self.curvature_feedback_sub, self.throttle_feedback_sub,\
            self.brake_feedback_sub, self.gear_feedback_sub, self.steering_wheel_sub, self.velocity_accel_sub, self.turn_rpt_sub], 
            queue_size=10, slop=0.04)
        self.ssc_feedbacks_sub.registerCallback(self.ssc_feedbacks_callback)

        # initialize SSC command publishers
        self.speed_mode_pub = rospy.Publisher('/ssc/arbitrated_speed_commands', SpeedMode, queue_size=1)
        self.steer_mode_pub = rospy.Publisher('/ssc/arbitrated_steering_commands', SteerMode, queue_size=1)
        self.turn_signal_pub = rospy.Publisher('/ssc/turn_signal_command', TurnSignalCommand, queue_size=1)
        self.gear_pub = rospy.Publisher('/ssc/gear_select', GearCommand, queue_size=1)

        # initialize vehicle status publisher
        self.vehicle_status_pub = rospy.Publisher('vehicle_status', VehicleStatus, queue_size=1)

        # initialize timeout timer
        self.alive = False
        self.timeout_timer = rospy.Timer(rospy.Duration(self.command_timeout / 1000.0), self.timeout_callback)

    def engage_callback(self, msg):
        # record engagement command
        self.engage = msg.data

    def vehicle_cmd_callback(self, msg):
        # check for valid combinations of gear and velocity
        is_valid_cmd = (msg.gear_cmd.gear in [Gear.DRIVE, Gear.LOW] and msg.ctrl_cmd.linear_velocity >= 0.0) or \
                       (msg.gear_cmd.gear == Gear.REVERSE and msg.ctrl_cmd.linear_velocity <= 0.0) or \
                       (msg.gear_cmd.gear == Gear.PARK and -LOW_SPEED_THRESH <= msg.ctrl_cmd.linear_velocity <= LOW_SPEED_THRESH) or \
                        msg.gear_cmd.gear == Gear.NONE
        if is_valid_cmd:
            # if valid command, set speed and engage
            desired_mode = int(self.engage)
            desired_speed = abs(msg.ctrl_cmd.linear_velocity)
        else:
            rospy.logwarn("Invalid vehicle command: gear = %d, velocity = %lf", msg.gear_cmd.gear, msg.ctrl_cmd.linear_velocity)
            rospy.logwarn("Disengaging autonomy")
            # if not valid command then disengage
            desired_mode = 0
            desired_speed = 0.0

        # calculate desired steering angle
        if self.use_adaptive_gear_ratio:
            desired_steering_angle = msg.ctrl_cmd.steering_angle * self.ssc_gear_ratio / self.adaptive_gear_ratio
        else:
            desired_steering_angle = msg.ctrl_cmd.steering_angle

        # calculate desired curvature for SSC
        desired_curvature = math.tan(desired_steering_angle) / self.wheel_base

        # set desired gear only when valid
        desired_gear = Gear.NONE
        if self.engage and is_valid_cmd:
            desired_gear = msg.gear_cmd.gear
            # refuse REVERSE gear when not enabled
            if desired_gear == Gear.REVERSE and not self.enable_reverse_motion:
                rospy.logerr("Reverse gear ignored, reverse motion not enabled")
                desired_gear = Gear.NONE

        # calculate desired turn signal for SSC
        desired_turn_signal = TurnSignalCommand.NONE
        if msg.lamp_cmd.l == 0 and msg.lamp_cmd.r == 0:
            desired_turn_signal = TurnSignalCommand.NONE
        elif msg.lamp_cmd.l == 1 and msg.lamp_cmd.r == 0:
            desired_turn_signal = TurnSignalCommand.LEFT
        elif msg.lamp_cmd.l == 0 and msg.lamp_cmd.r == 1:
            desired_turn_signal = TurnSignalCommand.RIGHT
        elif msg.lamp_cmd.l == 1 and msg.lamp_cmd.r == 1:
            # HAZARD signal cannot be ised in TurnSignalCommand
            pass

        # emergency mode stops the car
        if msg.emergency == 1:
            rospy.logerr("Emergency stopping, speed overridden to 0")
            desired_speed = 0.0

        # publish command messages
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = 'base_link'
        self.publish_speed_command(header, desired_mode, desired_speed, self.acceleration_limit, self.deceleration_limit)
        self.publish_steer_command(header, desired_mode, desired_curvature, self.max_curvature_rate)
        self.publish_turn_command(header, desired_mode, desired_turn_signal)
        self.publish_gear_command(header, desired_gear)

        # mark alive
        self.alive = True

    def timeout_callback(self, event=None):
        if not self.alive and self.engage:
            rospy.logerr("Did not receive any commands for %d ms", self.command_timeout)
            rospy.logerr("Disengaging autonomy until re-enabled")
            self.engage = False
        self.alive = False
        # TODO: publish commands

    def module_states_callback(self, msg):
        if 'veh_controller' in msg.name:
            # report current AUTO vs MANUAL mode
            if msg.state == 'active':
                self.dbw_enabled = True
            else:
                self.dbw_enabled = False

            # in case of SSC failure disengage
            if msg.state in ('failure', 'fatal', 'not_ready'):
                self.engage = False

    def ssc_feedbacks_callback(self, curvature_msg, throttle_msg, brake_msg, gear_msg, steering_msg, velocity_accel_msg, turn_rpt_msg):
        # calculate adaptive gear ratio, guard against division by zero later
        self.adaptive_gear_ratio = max(self.agr_coef_a + self.agr_coef_b * velocity_accel_msg.velocity**2 - self.agr_coef_c * steering_msg.steering_wheel_angle, sys.float_info.min)

        # current steering curvature
        if self.use_adaptive_gear_ratio:
            curvature = math.tan(steering_msg.steering_wheel_angle / self.adaptive_gear_ratio) / self.wheel_base
        else:
            curvature = curvature_msg.curvature

        vehicle_status = VehicleStatus()
        vehicle_status.header.frame_id = 'base_link'
        vehicle_status.header.stamp = rospy.Time.now()

        # current drive and steering mode
        if self.dbw_enabled:
            vehicle_status.drivemode = VehicleStatus.MODE_AUTO
        else:
            vehicle_status.drivemode = VehicleStatus.MODE_MANUAL
        vehicle_status.steeringmode = vehicle_status.drivemode

        # current speed km/h
        vehicle_status.speed = velocity_accel_msg.velocity * 3.6
        
        # current pedal positions [0,1000]
        vehicle_status.drivepedal = int(1000 * throttle_msg.throttle_pedal)
        vehicle_status.brakepedal = int(1000 * brake_msg.brake_pedal)

        # steering angle in radians
        vehicle_status.angle = math.atan(curvature * self.wheel_base)

        # current gear
        vehicle_status.current_gear.gear = gear_msg.current_gear.gear

        # turn signals
        vehicle_status.lamp = TURN_RPT_TO_VEHICLE_STATUS_LAMP_MAP[turn_rpt_msg.output]

        # publish the status message
        self.vehicle_status_pub.publish(vehicle_status)

    def publish_speed_command(self, header, desired_mode, desired_speed, acceleration_limit, deceleration_limit):
        # publish speed command
        msg = SpeedMode(header = header)
        msg.mode = desired_mode
        msg.speed = desired_speed
        msg.acceleration_limit = acceleration_limit
        msg.deceleration_limit = deceleration_limit
        self.speed_mode_pub.publish(msg)

    def publish_steer_command(self, header, desired_mode, desired_curvature, max_curvature_rate):
        # publish steering command
        msg = SteerMode(header = header)
        msg.mode = desired_mode
        msg.curvature = desired_curvature
        msg.max_curvature_rate = max_curvature_rate
        self.steer_mode_pub.publish(msg)

    def publish_turn_command(self, header, desired_mode, desired_turn_signal):
        # publish turn signal command
        msg = TurnSignalCommand(header = header)
        msg.mode = desired_mode
        msg.turn_signal = desired_turn_signal
        self.turn_signal_pub.publish(msg)

    def publish_gear_command(self, header, desired_gear):
        # publish gear command
        msg = GearCommand(header = header)
        msg.command.gear = desired_gear
        self.gear_pub.publish(msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('ssc_interface', log_level=rospy.INFO)
    node = SSCInterface()
    node.run()
