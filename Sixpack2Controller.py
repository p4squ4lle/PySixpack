#!/usr/bin/env python

from serial import Serial
from collections import OrderedDict
from Sixpack2Motor import Sixpack2Motor
import constants as c


class SIXpack2Controller(list):
    # =========================================================================
    # Initialize Sixpack Controller
    # =========================================================================

    def __init__(self, port='/dev/ttyUSB4',
                 baudrate=19200, timeout=None,
                 sixpack_addr='00', resp_addr='00', num_motors=4):

        list.__init__(self)

        self._port = port
        self._ser = Serial(self._port)
        self._ser.baudrate = baudrate
        self._ser.timeout = timeout

        self._sixpack_addr = sixpack_addr
        self._resp_addr = resp_addr
        # sixpack & resp addr müssen als string im hex format angegeben werden
        # (z.B. '00')
        self.num_motors = num_motors

        self._create_motors()

        if self.num_motors != len(self):
            raise UserWarning('number of specified motors (={0})'
                              .format(self.num_motors),
                              '!= number of initialized motors (={1})'
                              .format(len(self)))

        self.last_command = ''
        self.last_request = ''
        self.status_dict = {'motor{}'.format(i): 'N.a.'
                            for i in range(self.num_motors)}

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

    def send_command(self, command):
        """
        Encodes and sends command to the PACK.
        """

        command = self._sixpack_addr + command
        self.last_command = command
        command_bytes = bytes.fromhex(command)

        self._ser.reset_output_buffer()
        self._ser.write(command_bytes)

        return None

    def send_request(self, request):
        """
        Encodes and sends request to the PACK.
        Receive reply bytes, transcode them into integer numbers
        and write them into reply_dict dictonary
        """

        request = self._sixpack_addr + request
        self.last_request = request
        request_bytes = bytes.fromhex(request)

        self._ser.reset_output_buffer()
        self._ser.write(request_bytes)

        self._ser.reset_input_buffer()
        reply_bytes = self._ser.read(9)
        reply_hex = reply_bytes.hex()

        reply_dict = OrderedDict.fromkeys(['addr', 'cmd', 'p0',
                                           'p1', 'p2', 'p3',
                                           'p4', 'p5', 'p6'])

        if reply_hex[2:4] != request[2:4]:
            print('Error: Response command nr ({0}) does not match requested'
                  'command nr ({1})'.format(reply_dict['cmd'], request[2:4]))

        for i, k in enumerate(reply_dict):
            reply_dict[k] = int(reply_hex[2*i:2*i+2], 16)
        reply_dict['cmd'] = reply_hex[2:4]

        return reply_dict

    # ========================================================================
    # Get Unit Information
    # ========================================================================

    def get_unit_info(self):
        """
        Reads out firmware revision, reset-flag, temperature and serial number
        """

        request = '43{0}'.format(self._resp_addr) + 6*'00'
        reply = self.send_request(request)

        firmware = reply['p0']
        firmware = '.'.join(str(firmware))

        reset_flag = reply['p1']

        pack_temp = c.decode_param([reply['p2']])

        serial_n = list(reply.values())[5:9]
        serial_n = c.decode_param(serial_n, signed=False)

        # serial_n = c.decode_param([reply['p3'], reply['p4']
        #                            reply['p5'], reply['p6']],
        #                            signed=False)

        return firmware, reset_flag, pack_temp, serial_n

    # ========================================================================
    # Moving the motors
    # ========================================================================

    def query_all_motor_activities(self, mask=0):
        """
        Queries current activity of all motors. The mask specifies the motors
        for which a delayed response is wanted, i.e. the PACK will not send the
        response before all concerned motors are inactive.
        (mask for delayed response: bit 0 = motor 0, ..., bit 5 = motor 5;
         0: motor masked, 1: delayed respond; default: all motors masked)

        The response is written in the status_dict dictonary.
        """

        request = '28{0}{1}'.format(self._resp_addr, mask) + 5 * '00'
        reply = self.send_request(request)

        for i in range(self.num_motors):
            act = reply['p{}'.format(i)]
            self.status_dict['motor{}'.format(i)] = c.ACTION_DICT[act]

        return reply

    def start_parallel_ramp(self, mask):
        """
        Starts coordinated movement, by starting multiple motors at the same
        time. The target position has to be programmed previously.
        (Sixpack2Motor.set_targetpos(targetpos))
        (mask for parallel ramp: bit 0 = motor 0, ..., bit 5 = motor 5;
         0: motor masked, 1: start motor)
        """

        command = '29{}'.format(mask) + 6 * '00'
        self.send_command(command)

        return None

    def stop_motors(self, mask):
        """
        Stop multiple motors at the same time. This command sets the target
        position of each concerned motor equal to its actual position. However
        motors driving beyond vstart will overshoot and return driving another
        ramp back to the point their new target position was set using this
        command.
        (mask for deceleration: bit 0 = motor 0, ..., bit 5 = motor 5;
         0: motor masked, 1: set target position to actual position)
        """

        command = '2A{0}'.format(mask) + 6 * '00'
        # mask muss richtig parametrisiert werden
        self.send_command(command)

        return None

    # ========================================================================
    # Setting motor parameters
    # ========================================================================

    def set_velocity(self, clkdiv=5):
        # check formatting of encode_param if num_bytes=1, 2 ...

        clkdiv = c.encode_param(clkdiv, num_bytes=1)
        command = '12{}'.format(clkdiv) + 6 * '00'
        self.send_command(command)

        return None

    def write_motor_char_table(self, pointer, entries):
        # nochmal vestehen was die funktion genau macht

        if pointer % 4 != 0:
            raise ValueError('Parameter pointer not allowed.'
                             'Allowed values: 0, 4, 8, 12. (value given: {})'
                             .format(pointer))
        else:
            pointer = c.encode_param(pointer, num_bytes=1)

        for i, v in enumerate(entries):
            entries[i] = c.encode_param(v, num_bytes=1)

        cmd = '17{0}{1}{2}{3}{4}'.format(pointer, *entries) + 2 * '00'
        # weiß nich ob das so funktioniert
        self.send_command(cmd)

        return None

    # =============================================================================
    # Additional Inputs/Outputs
    # =============================================================================

    def read_input_channels(self, channelno):

        request = '300{0}{1}'.format(channelno, self._resp_addr) + 5 * '00'
        reply = self.send_request(request)

        channelno = reply['p0']

        analogue_value = reply['p1']

        ref_input = reply['p3']

        all_ref_inputs = reply['p4']
        # um die Bedeutung herauszubekommen müsste man hier
        # wieder in binär umrechnen

        logic_state_TTLIO1 = reply['p5']

        return(reply, channelno, analogue_value,
               ref_input, all_ref_inputs, logic_state_TTLIO1)

    def set_limits_stop_func(self, channelno,
                             min_value_left=0,
                             max_value_right=1023):

        min_value_left = c.encode_param(min_value_left, num_bytes=2)
        max_value_right = c.encode_param(max_value_right, num_bytes=2)

        cmd = '310{0}{1}{2}'.format(channelno, min_value_left, max_value_right)
        cmd += 2 * '00'
        self.send_command(cmd)

        return None

    def set_add_outputs(self, logic_state_TTLOUT1, TTLIO1,
                        logic_state_TTLIO1, TTLOUT1_ready):

        cmd = '320{0}{1}{2}{3}'.format(logic_state_TTLOUT1, TTLIO1,
                                       logic_state_TTLIO1, TTLOUT1_ready)
        cmd += 3 * '00'

        self.send_command(cmd)

        return None

    def set_ready_output_func(self, motormask, refsearchmask):

        cmd = '33{0}{1}'.format(motormask, refsearchmask) + 5 * '00'
        self.send_command(cmd)

        return None

    # =============================================================================
    # Other Settings
    # =============================================================================

    def adjust_baudrate(self, baudratedivisor, transmitter_delay=3):

        baudratedivisor = c.encode_param(baudratedivisor, num_bytes=2)
        transmitter_delay = c.encode_param(transmitter_delay, num_bytes=2)

        cmd = '40{0}{1}'.format(baudratedivisor, transmitter_delay) + 3 * '00'
        self.send_command(cmd)

        return None

    def set_abort_timeout(self, abort_timeout):

        abort_timeout = c.encode_param(abort_timeout, num_bytes=2)

        cmd = '41{0}'.format(abort_timeout) + 5 * '00'
        self.send_command(cmd)

        return None

    def change_unit_address(self, unit_address):

        unit_address = c.encode_param(unit_address, num_bytes=1)

        cmd = '42{0}'.format(unit_address) + 6 * '00'
        self.send_command(cmd)

        return None

    def complete_hwreset(self):

        cmd = 'CC' + 7 * '00'
        self.send_command(cmd)

        return None

    # =============================================================================
    # Multi-dimensional movement
    # =============================================================================

    def start_multi_movement(self, motormask):

        cmd = '50{0}'.format(motormask) + 6 * '00'
        self.send_command(cmd)

        return None

    # =============================================================================
    # Closing Serial Port
    # =============================================================================

    def __del__(self):
        self._ser.close()
