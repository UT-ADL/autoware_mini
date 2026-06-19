#!/usr/bin/env python3

import math
import sys
import traceback

import rospy
import message_filters

from std_msgs.msg import Bool, Header
from autoware_mini.msg import VehicleCommand, VehicleStatus
from automotive_platform_msgs.msg import SpeedMode, SteerMode, TurnSignalCommand, GearCommand,\
     CurvatureFeedback, ThrottleFeedback, BrakeFeedback, GearFeedback, SteeringFeedback, VelocityAccelCov
from automotive_navigation_msgs.msg import ModuleState
# from pacmod3_msgs.msg import SystemRptInt

LOW_SPEED_THRESH = 0.01

TURN_SIGNAL_TO_SSC = {
    VehicleCommand.TURN_STRAIGHT: TurnSignalCommand.NONE,
    VehicleCommand.TURN_LEFT: TurnSignalCommand.LEFT,
    VehicleCommand.TURN_RIGHT: TurnSignalCommand.RIGHT,
}

class SSCInterface:
    def __init__(self):
        # get parameters
        self.use_adaptive_gear_ratio = rospy.get_param('~use_adaptive_gear_ratio')
        self.enable_reverse_motion = rospy.get_param('~enable_reverse_motion')
        self.command_timeout = rospy.get_param('~command_timeout')
        self.wheel_base = rospy.get_param('wheel_base')
        self.ssc_gear_ratio = rospy.get_param('steer_ratio')
        self.acceleration_limit = rospy.get_param('acceleration_limit')
        self.deceleration_limit = rospy.get_param('deceleration_limit')
        self.default_acceleration = rospy.get_param('/planning/default_acceleration')
        self.default_deceleration = rospy.get_param('/planning/default_deceleration')
        self.max_curvature_rate = rospy.get_param('~max_curvature_rate')
        self.agr_coef_a = rospy.get_param('~agr_coef_a')
        self.agr_coef_b = rospy.get_param('~agr_coef_b')
        self.agr_coef_c = rospy.get_param('~agr_coef_c')
        self.max_speed = rospy.get_param('~max_speed')
        self.enable_emergency_braking = rospy.get_param('~enable_emergency_braking')
        # self.turn_signals = SystemRptInt.TURN_NONE

        # initialize variables
        self.engage = False
        self.dbw_enabled = False
        self.adaptive_gear_ratio = self.ssc_gear_ratio

        # initialize SSC command publishers
        self.speed_mode_pub = rospy.Publisher('/ssc/arbitrated_speed_commands', SpeedMode, queue_size=1, tcp_nodelay=True)
        self.steer_mode_pub = rospy.Publisher('/ssc/arbitrated_steering_commands', SteerMode, queue_size=1, tcp_nodelay=True)
        self.turn_signal_pub = rospy.Publisher('/ssc/turn_signal_command', TurnSignalCommand, queue_size=1, tcp_nodelay=True)
        self.gear_pub = rospy.Publisher('/ssc/gear_select', GearCommand, queue_size=1, tcp_nodelay=True)

        # initialize vehicle status publisher
        self.vehicle_status_pub = rospy.Publisher('vehicle_status', VehicleStatus, queue_size=1, tcp_nodelay=True)

        # initialize command subscribers
        rospy.Subscriber('engage', Bool, self.engage_callback, queue_size=None, tcp_nodelay=True)
        rospy.Subscriber('/control/vehicle_cmd', VehicleCommand, self.vehicle_cmd_callback, queue_size=1, tcp_nodelay=True)

        # initialize SSC feedback subscribers
        rospy.Subscriber('/ssc/module_states', ModuleState, self.module_states_callback, queue_size=1, tcp_nodelay=True)
        message_filters.ApproximateTimeSynchronizer([
                message_filters.Subscriber('/ssc/curvature_feedback', CurvatureFeedback, queue_size=1, tcp_nodelay=True),
                message_filters.Subscriber('/ssc/throttle_feedback', ThrottleFeedback, queue_size=1, tcp_nodelay=True),
                message_filters.Subscriber('/ssc/brake_feedback', BrakeFeedback, queue_size=1, tcp_nodelay=True),
                message_filters.Subscriber('/ssc/gear_feedback', GearFeedback, queue_size=1, tcp_nodelay=True),
                message_filters.Subscriber('/ssc/steering_feedback', SteeringFeedback, queue_size=1, tcp_nodelay=True),
                message_filters.Subscriber('/ssc/velocity_accel_cov', VelocityAccelCov, queue_size=1, tcp_nodelay=True)
            ], queue_size=2, slop=1.0/30.0).registerCallback(self.ssc_feedbacks_callback)
        # # take turn signal info from Pacmod, because it is not available from SSC (not used for now, to remove pacmod3_msgs dependency)
        # rospy.Subscriber('/pacmod/turn_rpt', SystemRptInt, self.turn_rpt_callback, queue_size=1, tcp_nodelay=True)

        # initialize timeout timer
        self.alive = False
        self.timeout_timer = rospy.Timer(rospy.Duration(self.command_timeout / 1000.0), self.timeout_callback)

    def engage_callback(self, msg):
        # record engagement command
        self.engage = msg.data

    def vehicle_cmd_callback(self, msg):
        # check for valid combinations of gear and velocity
        is_valid_cmd = (msg.gear in [VehicleCommand.GEAR_DRIVE, VehicleCommand.GEAR_LOW] and msg.speed >= 0.0) or \
                       (msg.gear == VehicleCommand.GEAR_REVERSE and msg.speed <= 0.0) or \
                       (msg.gear == VehicleCommand.GEAR_PARK and -LOW_SPEED_THRESH <= msg.speed <= LOW_SPEED_THRESH) or \
                        msg.gear == VehicleCommand.GEAR_NONE
        if is_valid_cmd:
            # if valid command, set speed and engage
            desired_mode = int(self.engage)
            desired_speed = min(self.max_speed / 3.6, abs(msg.speed))
        else:
            rospy.logwarn("%s - invalid vehicle command: gear = %d, speed = %lf", rospy.get_name(), msg.gear, msg.speed)
            rospy.logwarn("%s - disengaging autonomy", rospy.get_name())
            # if not valid command then disengage
            desired_mode = 0
            desired_speed = 0.0

        # calculate desired steering angle
        if self.use_adaptive_gear_ratio:
            desired_steering_angle = msg.steering_angle * self.ssc_gear_ratio / self.adaptive_gear_ratio
        else:
            desired_steering_angle = msg.steering_angle

        # calculate desired curvature for SSC
        desired_curvature = math.tan(desired_steering_angle) / self.wheel_base

        # set desired gear only when valid
        desired_gear = VehicleCommand.GEAR_NONE
        if self.engage and is_valid_cmd:
            desired_gear = msg.gear
            # refuse REVERSE gear when not enabled
            if desired_gear == VehicleCommand.GEAR_REVERSE and not self.enable_reverse_motion:
                rospy.logerr("%s - reverse gear ignored, reverse motion not enabled", rospy.get_name())
                desired_gear = VehicleCommand.GEAR_NONE

        # calculate desired turn signal for SSC
        desired_turn_signal = TURN_SIGNAL_TO_SSC.get(msg.turn_signal, TurnSignalCommand.NONE)

        # emergency mode stops the car
        if msg.emergency == 1 and self.enable_emergency_braking:
            rospy.logwarn_throttle(10, "%s - emergency stopping", rospy.get_name())
            acceleration_limit = 0.0
            deceleration_limit = 0.0
            desired_speed = 0.0
        else:
            # calculate acceleration and deceleration limits
            if msg.acceleration == 0.0:
                acceleration_limit = self.default_acceleration
                deceleration_limit = self.default_deceleration
            elif msg.acceleration > 0.0:
                acceleration_limit = min(msg.acceleration, self.acceleration_limit)
                deceleration_limit = self.default_deceleration
            elif msg.acceleration < 0.0:
                acceleration_limit = self.default_acceleration
                deceleration_limit = min(-msg.acceleration, self.deceleration_limit)

        # publish command messages
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = 'base_link'
        self.publish_speed_command(header, desired_mode, desired_speed, acceleration_limit, deceleration_limit)
        self.publish_steer_command(header, desired_mode, desired_curvature)
        self.publish_turn_command(header, desired_mode, desired_turn_signal)
        self.publish_gear_command(header, desired_gear)

        # mark alive
        self.alive = True

    def timeout_callback(self, event=None):
        if not self.alive and self.engage:
            rospy.logerr("%s - did not receive any commands for at least %d ms", rospy.get_name(), self.command_timeout)
            rospy.logerr("%s - disengaging autonomy until re-enabled", rospy.get_name())
            self.engage = False

            # send dummy commands to keep SSC alive
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'base_link'
            self.publish_speed_command(header, 0, 0.0)
            self.publish_steer_command(header, 0, 0.0)
            self.publish_turn_command(header, 0, TurnSignalCommand.NONE)
            self.publish_gear_command(header, VehicleCommand.GEAR_NONE)

        self.alive = False

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

    def ssc_feedbacks_callback(self, curvature_msg, throttle_msg, brake_msg, gear_msg, steering_msg, velocity_accel_msg):
        try:

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

            # current speed m/s
            vehicle_status.speed = velocity_accel_msg.velocity

            # current pedal positions [0,1]
            vehicle_status.drivepedal = throttle_msg.throttle_pedal
            vehicle_status.brakepedal = brake_msg.brake_pedal

            # steering angle in radians
            vehicle_status.angle = math.atan(curvature * self.wheel_base)

            # current gear
            vehicle_status.gear = gear_msg.current_gear.gear

            # turn signals
            vehicle_status.turn_signal = VehicleStatus.TURN_STRAIGHT

            # publish the status message
            self.vehicle_status_pub.publish(vehicle_status)

        except Exception as e:
            rospy.logerr_throttle(10, "%s - Exception in callback: %s", rospy.get_name(), traceback.format_exc())

    # def turn_rpt_callback(self, turn_rpt_msg):
    #     self.turn_signals = turn_rpt_msg.output


    def publish_speed_command(self, header, desired_mode, desired_speed, acceleration_limit=0.0, deceleration_limit=0.0):
        # publish speed command
        msg = SpeedMode(header = header)
        msg.mode = desired_mode
        msg.speed = desired_speed
        msg.acceleration_limit = acceleration_limit
        msg.deceleration_limit = deceleration_limit
        self.speed_mode_pub.publish(msg)

    def publish_steer_command(self, header, desired_mode, desired_curvature):
        # publish steering command
        msg = SteerMode(header = header)
        msg.mode = desired_mode
        msg.curvature = desired_curvature
        msg.max_curvature_rate = self.max_curvature_rate
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
