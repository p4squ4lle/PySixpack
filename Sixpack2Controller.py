#!/usr/bin/env python

from serial import Serial
from collections import OrderedDict
from Sixpack2Motor import Sixpack2Motor
from constants import *


class SIXpack2Controller(list):
    # =========================================================================
    # Initialize Sixpack Controller
    # =========================================================================

    def __init__(self, port='/dev/ttyUSB3',
                 baudrate=19200, timeout=None,
                 sixpack_addr='00', resp_addr='00', num_motors=1):

        list.__init__(self)

        self._port = port
        self._ser = Serial(self._port)
        self._ser.baudrate = baudrate
        self._ser.timeout = timeout

        self._sixpack_addr = sixpack_addr
        self._resp_addr = resp_addr
        self.num_motors = num_motors

        self._create_motors()

        if self.num_motors != len(self):
            raise UserWarning('number of specified motors (={0})'
                              .format(self.num_motors),
                              '!= number of initialized motors (={1})'
                              .format(len(self)))

        self.last_command = ''
        self.last_request = ''
        self.status_dict = {'motor{}'.format(i): {'action': None,
                                                  'position': None,
                                                  'velocity': None}
                            for i in range(self.num_motors)}

        self._reply_dict = OrderedDict.fromkeys(['addr', 'cmd', 'p0',
                                                 'p1', 'p2', 'p3',
                                                 'p4', 'p5', 'p6'])

    # ========================================================================
    # Initialize Sixpack Motors
    # ========================================================================

    def _create_motors(self):
        for motno in range(self.num_motors):
            motor = Sixpack2Motor(self, motno)
            self.append(motor)

    # ========================================================================
    # Send command and request reply
    # ========================================================================

    def __send_command(self, command):
        """
        Encodes and sends command to the PACK.
        """

        command = self._sixpack_addr + command
        self.last_command = command
        command_bytes = bytes.fromhex(command)

        self._ser.reset_output_buffer()
        self._ser.write(command_bytes)

        return None

    def _send_request(self, request):
        """
        Encodes and sends request to the PACK.
        Receives reply bytes, transcodes them into integer numbers
        and writes them into reply_dict dictonary.
        """

        request = self._sixpack_addr + request
        self.last_request = request
        request_bytes = bytes.fromhex(request)

        self._ser.reset_output_buffer()
        self._ser.write(request_bytes)

        self._ser.reset_input_buffer()
        reply_bytes = self._ser.read(9)
        reply_hex = reply_bytes.hex()

        if reply_hex[2:4] != request[2:4]:
            raise UserWarning('Warning: Response command nr ({0}) does not'
                              'match requested command nr ({1})'
                              .format(reply_hex[2:4], request[2:4]))

        for i, k in enumerate(self.reply_dict):
            self.reply_dict[k] = int(reply_hex[2*i:2*i+2], 16)
        self.reply_dict['cmd'] = reply_hex[2:4]

        return self.reply_dict

    # ========================================================================
    # Get Unit Information
    # ========================================================================

    def get_unit_info(self):
        """
        Reads out firmware revision, reset-flag, temperature and serial number
        """

        request = '43{0}'.format(self._resp_addr) + 6*'00'
        reply = self._send_request(request)

        firmware = reply['p0']
        firmware = '.'.join(str(firmware))

        reset_flag = reply['p1']

        pack_temp = _decode_param([reply['p2']])

        serial_n = list(reply.values())[5:9]
        serial_n = _decode_param(serial_n, signed=False)

        # serial_n = _decode_param([reply['p3'], reply['p4']
        #                            reply['p5'], reply['p6']],
        #                            signed=False)

        return firmware, reset_flag, pack_temp, serial_n

    # ========================================================================
    # Moving the motors
    # ========================================================================

    def query_all(self, mask='00'):
        """
        Queries current activity of all motors. The mask specifies the motors
        for which a delayed response is wanted, i.e. the PACK will not send the
        response before all concerned motors are inactive.
        (mask for delayed response: bit 0 = motor 0, ..., bit 5 = motor 5;
         0: motor masked, 1: delayed respond; default: all motors masked)

        The response is written in the status_dict dictonary.
        """

        mask = _encode_mask(mask)

        request = '28{0}{1}'.format(self._resp_addr, mask) + 5 * '00'
        reply = self._send_request(request)

        for i in range(self.num_motors):
            act = reply['p{}'.format(i)]
            if act in ACTION_DICT:
                self.status_dict[
                                'motor{}'.format(i)
                                ]['action'] = ACTION_DICT[act]
            elif 20 <= act <= 29:
                self.status_dict[
                                'motor{}'.format(i)
                                ]['action'] = 'reference switch search'
            else:
                raise UserWarning('reply action ({0}) for motor {1} seems to'
                                  .format(act, i), 'be incorrect and was not'
                                  'found in ACTION_DICT')

        return self.status_dict

    def start_parallel_ramp(self, mask):
        """
        Starts coordinated movement, by starting multiple motors at the same
        time. The target position has to be programmed previously.
        (Sixpack2Motor.set_targetpos(targetpos))
        (mask for parallel ramp: bit 0 = motor 0, ..., bit 5 = motor 5;
         0: motor masked, 1: start motor)
        """

        mask = _encode_mask(mask)

        command = '29{}'.format(mask) + 6 * '00'
        self._send_command(command)

        return None

    def stop_motors(self, mask='111111'):
        """
        Stop multiple motors at the same time. This command sets the target
        position of each concerned motor equal to its actual position. However
        motors driving with velocity beyond vstart will overshoot and return
        driving another ramp back to the point their new target position
        was set using this command.
        (mask for deceleration: bit 0 = motor 0, ..., bit 5 = motor 5;
         0: motor masked, 1: set target position to actual position)
        """

        mask = _encode_mask(mask)

        command = '2A{0}'.format(mask) + 6 * '00'
        self._send_command(command)

        return None

    # ========================================================================
    # Setting motor parameters
    # ========================================================================

    def set_velocity(self, clkdiv=5):

        clkdiv = _encode_param(clkdiv, 'clkdiv', num_bytes=1)
        command = '12{}'.format(clkdiv) + 6 * '00'
        self._send_command(command)

        return None

    def write_motor_char_table(self, pointer, entrylist):
        # nochmal vestehen was die funktion genau macht

        if type(entrylist) != list:
            raise TypeError('given entry list has to be of type list',
                            '([ entry0, entry1, entry2, entry3];',
                            'type of given entry list: {}'
                            .format(type(entrylist)))

        else:
            pointer = _encode_param(pointer, 'table_pointer', num_bytes=1)

        for i, v in enumerate(entrylist):
            entrylist[i] = _encode_param(v, 'table_entry', num_bytes=1)

        cmd = '17{0}{1}{2}{3}{4}'.format(pointer, *entrylist) + 2 * '00'
        self._send_command(cmd)

        return None

    # =============================================================================
    # Additional Inputs/Outputs
    # =============================================================================

    def read_input_channels(self, channelno):

        channelno = _encode_param(channelno, 'channelno', num_bytes=1)

        request = '30{0}{1}'.format(channelno, self._resp_addr) + 5 * '00'
        reply = self._send_request(request)

        channelno = reply['p0']

        analogue_value = reply['p1']

        ref_input = reply['p3']

        all_ref_inputs = reply['p4']
        # um die Bedeutung herauszubekommen müsste man hier
        # wieder in binär umrechnen

        logic_state_TTLIO1 = reply['p5']

        return (reply, channelno, analogue_value, ref_input,
                all_ref_inputs, logic_state_TTLIO1)

    def set_limits_stop_func(self, channelno,
                             min_value_left=0,
                             max_value_right=1023):
        channelno = _encode_param(channelno, 'channelno', num_bytes=1)

        min_value_left = _encode_param(min_value_left, 'stop_func_limits',
                                       num_bytes=2)
        max_value_right = _encode_param(max_value_right, 'stop_func_limits',
                                        num_bytes=2)

        cmd = '31{0}{1}{2}'.format(channelno, min_value_left, max_value_right)
        cmd += 2 * '00'
        self._send_command(cmd)

        return None

    def set_add_outputs(self, logic_state_TTLOUT1, TTLIO1,
                        logic_state_TTLIO1, TTLOUT1_ready):

        cmd = '320{0}{1}{2}{3}'.format(logic_state_TTLOUT1, TTLIO1,
                                       logic_state_TTLIO1, TTLOUT1_ready)
        cmd += 3 * '00'

        self._send_command(cmd)

        return None

    def set_ready_output_func(self, motormask, refsearchmask):

        motormask = _encode_mask(motormask)
        refsearchmask = _encode_mask(refsearchmask)

        cmd = '33{0}{1}'.format(motormask, refsearchmask) + 5 * '00'
        self._send_command(cmd)

        return None

    # =============================================================================
    # Other Settings
    # =============================================================================

    def adjust_baudrate(self, baudratedivisor, transmitter_delay=3):

        baudratedivisor = _encode_param(baudratedivisor, 'baudratedivisor',
                                        num_bytes=2)
        transmitter_delay = _encode_param(transmitter_delay,
                                          'transmitter_delay',
                                          num_bytes=2)

        cmd = '40{0}{1}'.format(baudratedivisor, transmitter_delay) + 3 * '00'
        self._send_command(cmd)

        return None

    def set_abort_timeout(self, abort_timeout):

        abort_timeout = _encode_param(abort_timeout, 'abort_timeout',
                                      num_bytes=2)

        cmd = '41{0}'.format(abort_timeout) + 5 * '00'
        self._send_command(cmd)

        return None

    def change_unit_address(self, unit_address):

        unit_address = _encode_param(unit_address, 'unit_address', num_bytes=1)

        cmd = '42{0}'.format(unit_address) + 6 * '00'
        self._send_command(cmd)
        self._sixpack_addr = unit_address

        return None

    def complete_hwreset(self):

        cmd = 'CC' + 7 * '00'
        self._send_command(cmd)

        return None

    # =============================================================================
    # Multi-dimensional movement
    # =============================================================================

    def start_multi_movement(self, motormask):

        motormask = _encode_mask(motormask)

        cmd = '50{0}'.format(motormask) + 6 * '00'
        self._send_command(cmd)

        return None

    # =============================================================================
    # Closing Serial Port
    # =============================================================================

    def __del__(self):
        self._ser.close()
