# Control focus on webcam using printer's XY coordinates,
# and/or control other webcam settings using gcode macros.
#
# Copyright (C) 2022 Colin MacKenzie <colin@flyingeinstein.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math
import logging
import numpy as np

UPDATE_TIME  = 1.0

def clamp(value: int or float, min_range: int or float, max_range: int or float):
    if value < min_range:
        return min_range
    elif value > max_range:
        return max_range
    else:
        return value


######################################################################
# Velocity Mode
######################################################################

class VelocityMode:
    cmd_velocity_help = 'Set settings on your webcam'
    cmd_set_vel_help = 'Set toolhead to velocity'
    cmd_clear_vel_help = 'Disable toolhead velocity mode'

    dt = 0.2

    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.gcode   = self.printer.lookup_object('gcode')
        #self.printer.load_object(config, "display_status")

        #self.video_dev = config.get('device', '/dev/video0')
        #self.min_focus = config.getint('min_focus', 0)
        #self.max_focus = config.getint('max_focus', 255)

        # read camera position from config and ensure it is 4 values (x,y,z)
        #self.camera_position = ensure_xyz(config.getintlist('camera_position'))

        # todo: the length of these arrays should be determined by number of kinematic dimensions (i.e. usually 3 for xyz)
        self.velocity = [0.0, 0.0]              # commanded user velocity
        self.target_position = [0.0, 0.0]       # the last position we sent to the toolhead

        # todo: load this from a printer object and/or config
        self.min_limits = [0.0, 0.0, 0.0]
        self.max_limits = [300.0, 300.0, 100.0]

        self.stepperTimer     = None
        self.printer.register_event_handler('klippy:ready', self._handle_ready)
        self.gcode.register_command('SET_VEL',
                                    self.cmd_set_vel,
                                    desc=self.cmd_set_vel_help)
        self.gcode.register_command('CLEAR_VEL',
                                    self.cmd_clear_vel,
                                    desc=self.cmd_clear_vel_help)
        self.shutdown = False
        self.enable_velocity_control = True     # user control of velocity mode
        self.allow_velocity_mode = False        # False if the machine is not homed or velocity mode cannot be enabled


    def _handle_ready(self):
        self.shutdown = False
        self.reactor = self.printer.get_reactor()
        self.printer.register_event_handler('klippy:shutdown', 
                                            self._handle_shutdown)
        self.printer.register_event_handler("homing:home_rails_end",
                                            self.handle_home_rails_end)

        # machine position from kinematics driver
        self.position = [0.0, 0.0, 0.0]
        self.toolhead = self.printer.lookup_object('toolhead')
        self.kin = self.toolhead.get_kinematics()

        # register update timers
        #self.displayStatus = self.printer.lookup_object('display_status')
        self.stepperTimer = self.reactor.register_timer(
                                self._pollStepper,
                                self.reactor.NOW)

    def _handle_shutdown(self):
        self.shutdown = True
        pass

    def handle_home_rails_end(self, homing_state, rails):
        # an axis was homed, try to enable focus control
        if not self.enable_velocity_control:
            self.try_allow_velocity_control()

    def home_status(self):
        # note: we only care about axis that are involved in the focal_axis setting
        curtime = self.printer.get_reactor().monotonic()
        kin_status = self.toolhead.get_kinematics().get_status(curtime)
        home_status = kin_status['homed_axes']
        x_ready = (self.focal_axis[0] == 0.0) or ('x' in home_status)
        y_ready = (self.focal_axis[1] == 0.0) or ('y' in home_status)
        z_ready = (self.focal_axis[2] == 0.0) or ('z' in home_status)
        return x_ready and y_ready and z_ready

    def try_allow_velocity_control(self):
        if self.allow_focus_control:
            # already enabled
            return True

        if self.home_status():
            # we have the components we need for position derived auto-focus
            self.allow_focus_control = True
            self.gcode.respond_raw('allowing velocity control mode')
            return True
        else:
            return False

    def disable_velocity_control(self):
        if self.allow_focus_control:
            self.allow_focus_control = False
            self.gcode.respond_raw('disabled velocity mode')


    def _pollStepper(self, eventtime):
        # get raw stepper counts (target position)
        kin_spos = {s.get_name(): s.get_commanded_position()
                    for s in self.kin.get_steppers()}
       
        # covert stepper counts to machine coordinates
        self.position = self.kin.calc_position(kin_spos)

        # todo: I think we can just add a point dT ahead based on the current position,
        #       this will keep the lookahead full and the toolhead will always only be
        #       dT time behind instead of a target_position variable that keeps getting
        #       too far ahead of the toolhead's speed capabilities.
        # note: if you set the velocity too far high over capabilities (inc acc/const/dec phases)
        #       then we'd be adding position points that the machine must still go through and yet
        #       it will take more time than requested velocity would take if it used velocity control.
        #       If velocity is too low, then its slow??
        #       so maybe better to choose smaller dT increments.
		# solution: maybe the next timer should be scheduled based on current toolhead velocity so
        #       we keep the max plan ahead distance to a minimum.

        # todo: check how far the current position is from the commanded position,
        #       then calculate how much time to get there based on current velocity,
        #       then only add more velocity if we are less than a time threshold.
        # add position increment based on velocity
        square_velocity = 0.0
        move_to = [None] * len(self.velocity)
        for i,v in enumerate(self.velocity):
            if v is not None and v != 0.0:
                new_pos = clamp(self.position[i] + v * self.dt, self.min_limits[i], self.max_limits[i])
                if new_pos != self.position[i]:
                    move_to[i] = new_pos
                    square_velocity += v * v   # since this axis contributes to overall velocity

            # send new target position to toolhead if we still have velocity after limits/etc
            if square_velocity > 0:
                vel = math.sqrt(square_velocity)
                self.toolhead.manual_move(move_to, vel)
        
        return eventtime + self.dt / 2

    def cmd_set_vel(self, gcmd):
        X=gcmd.get_float('X', None)
        Y=gcmd.get_float('Y', None)
        if X is not None or Y is not None:
            # dx/dy mode
            self.velocity[0] = X if X is not None else 0.0
            self.velocity[1] = Y if Y is not None else 0.0
        else:
            A=gcmd.get_float('A', None)
            V=gcmd.get_float('V', None)
            if A is None or V is None:
                self.gcode.respond_raw('SET_VEL requires either X,Y (dx/dy mode) or A,V (angle, velocity) mode arguments')
            else:
                # angle/velocity mode
                self.velocity[0] = V * math.cos(A)
                self.velocity[1] = V * math.sin(A)


    def cmd_clear_vel(self, gcmd):
        self.velocity = [0.0 for v in self.velocity]



def load_config(config):
    return VelocityMode(config)


#def load_config_prefix(config):
#    return WebcamFocus(config)
