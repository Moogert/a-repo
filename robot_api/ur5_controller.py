#!/usr/bin/python27

#ur5_controller.py

import socket
import logging
import time
#import klampt.math.vectorops
import math
from PyUniversalRobot.network import rtde
import threading
import signal
import serial
import sys
import binascii
import ur5_config as config

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

SETPOINT_HALT     = 0
SETPOINT_POSITION = 1
SETPOINT_VELOCITY = 2


_CONTROLLER_PROGRAM = '''
stop program
set unlock protective stop

def rtde_control_loop():

    # constants
    SETPOINT_TIMEOUT  = 20
    SETPOINT_HALT     = 0
    SETPOINT_POSITION = 1
    SETPOINT_VELOCITY = 2
    CONTROL_PERIOD = 0.008
    RTDE_WATCHDOG_FREQUENCY = 1

    # integer registers
    REG_SETPOINT = 0
    REG_TYPE = 1

    # double registers
    REG_TARGET = 0
    REG_VELOCITY = 6
    REG_ACCELERATION = 7
    REG_LOOKAHEAD = 8
    REG_GAIN = 9

    # I/O configuration
    set_standard_analog_input_domain(0, 1)
    set_standard_analog_input_domain(1, 1)
    set_tool_analog_input_domain(0, 1)
    set_tool_analog_input_domain(1, 1)
    set_analog_outputdomain(0, 0)
    set_analog_outputdomain(1, 0)
    set_tool_voltage(0)
    set_input_actions_to_default()

    # tool configuration
    set_tcp(p[{tcp[0]}, {tcp[1]}, {tcp[2]}, {tcp[3]}, {tcp[4]}, {tcp[5]}])
    set_payload({payload})
    set_gravity([{gravity[0]}, {gravity[1]}, {gravity[2]}])

    setpoint_number = read_input_integer_register(REG_SETPOINT)
    last_setpoint_number = setpoint_number
    missed_setpoints = 0

    rtde_set_watchdog("input_int_register_0", RTDE_WATCHDOG_FREQUENCY, "stop")

    while True:

	write_output_integer_register(7, 5)

        setpoint_number = read_input_integer_register(REG_SETPOINT)
        if setpoint_number == last_setpoint_number:
            missed_setpoints = missed_setpoints + 1
        else:
            missed_setpoints = 0
        end
        last_setpoint_number = setpoint_number

        if missed_setpoints >= SETPOINT_TIMEOUT:
            popup("setpoint timeout", title="PyUniversalRobot", error=True)
            halt
        end

        # update the setpoint
        write_output_integer_register(0, setpoint_number)

        target = [0, 0, 0, 0, 0, 0]
        target[0] = read_input_float_register(REG_TARGET + 0)
        target[1] = read_input_float_register(REG_TARGET + 1)
        target[2] = read_input_float_register(REG_TARGET + 2)
        target[3] = read_input_float_register(REG_TARGET + 3)
        target[4] = read_input_float_register(REG_TARGET + 4)
        target[5] = read_input_float_register(REG_TARGET + 5)

        type = read_input_integer_register(REG_TYPE)
        if type == SETPOINT_HALT:
            # issue command
            popup("halt command issued", title="PyUniversalRobot", error=True)
            halt
        elif type == SETPOINT_POSITION:
            # read lookahead and gain parameters
            lookahead = read_input_float_register(REG_LOOKAHEAD)
            gain = read_input_float_register(REG_GAIN)

            # issue command
            # NOTE: acceleration and velocity arguments are ignored
            servoj(target, 0, 0, CONTROL_PERIOD, lookahead, gain)
        elif type == SETPOINT_VELOCITY:
            # read acceleration parameter
            acceleration = read_input_float_register(REG_ACCELERATION)

            # issue command
            speedj(target, acceleration, CONTROL_PERIOD)
        else:
            # alert and quit
            popup("unknown setpoint type received", title="PyUniversalRobot", error=True)
            halt
        end
    end
end
'''



class UR5Controller(object):
    def __init__(self, host, **kwargs):
        self._quit = True

        self._robot_host = host
        self._rtde_port = kwargs.pop('rtde_port', 30004)
        self._command_port = kwargs.pop('command_port', 30002)

        self._servo_halt = None
        self._servo_position = None
        self._servo_velocity = None

        self._gain = 300
        self._lookahead = 0.1
        self._acceleration = 10
        self._velocity = 0
        self._speed_scale = None

        self._tcp = kwargs.pop('tcp', [0,0,0.04,0,0,0])
        self._payload = kwargs.pop('payload', 0.9)
        self._gravity = kwargs.pop('gravity', [0, 0, 9.81])

        self._version = None

        self._filters = kwargs.pop('filters', [])
        self._start_time = None
        self._last_t = 0

        #stuff needed for threading
        self._conn = None
        self._max_speed_scale = None
        self._sock = None
        self._quit = False

    def start(self):
        # initialize RTDE
        self._conn = rtde.RTDE(self._robot_host, self._rtde_port)
        self._conn.connect()
        self._version = self._conn.get_controller_version()

        # configure outputs (URControl -> Python)
        self._conn.send_output_setup(
            ['timestamp', 'target_q', 'actual_q', 'target_qd', 'actual_qd', 'target_qdd', 'target_speed_fraction'],
            ['DOUBLE', 'VECTOR6D', 'VECTOR6D', 'VECTOR6D', 'VECTOR6D', 'VECTOR6D', 'DOUBLE']
        )

        # configure inputs (Python -> URControl)
        target = self._conn.send_input_setup(['input_double_register_{}'.format(i) for i in range(6)], ['DOUBLE']*6)
        setpoint_id = self._conn.send_input_setup(['input_int_register_0'], ['INT32'])
        target_type = self._conn.send_input_setup(['input_int_register_1'], ['INT32'])
        velocity = self._conn.send_input_setup(['input_double_register_6'], ['DOUBLE'])
        acceleration = self._conn.send_input_setup(['input_double_register_7'], ['DOUBLE'])
        lookahead = self._conn.send_input_setup(['input_double_register_8'], ['DOUBLE'])
        gain = self._conn.send_input_setup(['input_double_register_9'], ['DOUBLE'])
        speed_slider = self._conn.send_input_setup(['speed_slider_mask', 'speed_slider_fraction'], ['UINT32', 'DOUBLE'])

        # start RTDE
        self._conn.send_start()

        # start the controller program
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.connect((self._robot_host, self._command_port))
        program = _CONTROLLER_PROGRAM.format(tcp=self._tcp, payload=self._payload, gravity=self._gravity)
        logger.info('controller program:\n{}'.format(program))
        self._sock.sendall(program.encode('ascii') + b'\n')

        self._max_speed_scale = None

        self._quit = False
        controlThread = threading.Thread(target = self.controlLoop, args=[[target, setpoint_id, target_type, velocity, acceleration, lookahead, gain, speed_slider]])
        controlThread.start()

    def controlLoop(self, args):
        if not len(args) == 8:
            print("Error in thread configuration, should have 8 inputs")
            self._sock.sendall(b'stop program\n')
            self._conn.send_pause()
            self._conn.disconnect()
            #disconnect and exit
            return

        target = args[0]
        setpoint_id = args[1]
        target_type = args[2]
        velocity = args[3]
        acceleration = args[4]
        lookahead = args[5]
        gain = args[6]
        speed_slider = args[7]


        def r2l(reg, base=0):
            list = []
            for i in range(6):
                list.append(reg.__dict__['input_double_register_{}'.format(base + i)])
            return list

        def l2r(list, reg, base=0):
            for i in range (6):
                reg.__dict__['input_double_register_{}'.format(base + i)] = list[i]
            return reg

        setpoint_number = 0
        #could set this outside the thread, but it seems reasonable for when the thread starts the setpoint should be 1

        while not self._quit:
            # receive the current update_state

            state = self._conn.receive()
            if state is None:
                logger.warn('lost RTDE connection -> stopping')
                break
            # honor speed scaling set when program started
            if self._max_speed_scale is None:
                self._max_speed_scale = state.target_speed_fraction
                self._speed_scale = state.target_speed_fraction

            # invoke update
            self._update(state)

            # run installed filters
            for f in self._filters:
                f(state)

            # determine target type
            if self._servo_halt:
                target_type.input_int_register_1 = SETPOINT_HALT
                self._conn.send(target_type)
                self._servo_halt = None
            elif self._servo_position:
                target_type.input_int_register_1 = SETPOINT_POSITION
                self._conn.send(target_type)
                self._conn.send(l2r(self._servo_position, target, 0))
                self._servo_position = None
            elif self._servo_velocity:
                target_type.input_int_register_1 = SETPOINT_VELOCITY
                self._conn.send(target_type)
                self._conn.send(l2r(self._servo_velocity, target, 0))
                self._servo_velocity = None
            else:
                logger.warn('missing setpoint -> stopping')
                break

            # kick watchdog
            setpoint_number += 1
            setpoint_id.input_int_register_0 = setpoint_number
            self._conn.send(setpoint_id)

            # clamp speed scale
            self._speed_scale = max(min(self._speed_scale, self._max_speed_scale), 0)

            # send parameters
            velocity.input_double_register_6 = self._velocity
            self._conn.send(velocity)
            acceleration.input_double_register_7 = self._acceleration
            self._conn.send(acceleration)
            lookahead.input_double_register_8 = self._lookahead
            self._conn.send(lookahead)
            gain.input_double_register_9 = self._gain
            self._conn.send(gain)
            speed_slider.speed_slider_mask = 1
            speed_slider.speed_slider_fraction = self._speed_scale
            self._conn.send(speed_slider)

        self._quit = True
        print("ending control loop")
        #if this loop exits, disconnect
        self._sock.sendall(b'stop program\n')

        self._conn.send_pause()
        self._conn.disconnect()
        print("disconnecting")


    def servo(self, halt=None, q=None, qd=None, qd_max=None, qdd_max=None, lookahead=None, gain=None):
        if sum([ x is not None for x in [halt, q, qd]]) > 1:
            raise RuntimeError('cannot specifiy more than one of halt, q, and qd')
        elif halt:
            self._servo_halt = True
            self._servo_position = None
            self._servo_velocity = None
        elif q:
            self._servo_halt = None
            self._servo_position = q
            self._servo_velocity = None
        elif qd:
            self._servo_halt = None
            self._servo_position = None
            self._servo_velocity = qd
        else:
            raise RuntimeError('missing halt, q, or qd')

        if qd_max is not None:
            self._velocity = qd_max
        if qdd_max is not None:
            self._acceleration = qdd_max
        if lookahead is not None:
            self._lookahead = lookahead
        if gain is not None:
            self._gain = gain

    #this function is not used
    #speed_scale set on teaching pendant directly 
    def speed_scale(self, s=None):
        if s is not None:
            self._speed_scale = s

        return self._speed_scale

    def _update(self, state):
        """Updated every control loop. Can be filled out by subclasses"""
    	pass

    def stop(self):
        self._quit = True

    def inLimits(self, q, min_limits=None, max_limits=None):
        for i in range(0, len(q)):
            if(min_limits and max_limits):
                if not(q[i] <= max_limits[i] and q[i] >= min_limits[i]):
                    return False
            else:
                print "warning, joint limits not set"
                return False
        return True

    def isFormatted(self, val):
        #do formatting
        if val:
            if len(val) == config.ROBOT_CONFIG_LEN:
                return True
        else:
            print("Error, val: ", val, " is not formatted correctly")
        return False

    @property
    def version(self):
        return self._version
