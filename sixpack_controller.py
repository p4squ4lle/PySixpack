#!/usr/bin/env python

from serial import Serial
from collections import OrderedDict


class SIXpack2():
    # =============================================================================
    # Initialize Sixpack Controller
    # =============================================================================

    def __init__(self, port='/dev/ttyUSB4',
                 baudrate=19200, timeout=None,
                 sixpack_addr='00', resp_addr='00', num_motors=4):

        self._port = port
        self._ser = Serial(port)
        self._ser.baudrate = baudrate
        self._ser.timeout = timeout

        self._sixpack_addr = sixpack_addr
        self._resp_addr = resp_addr
        self.num_motors = num_motors

        self._last_command = ''
        self._last_request = ''

        self.status_dict = {'motor{}'.format(i+1): None
                            for i in range(self.num_motors)}

# =============================================================================
# Send command and request reply
# =====================================================================

    def send_command(self, command):
        """
        Encode and send command
        """

        command = self._sixpack_addr + command

        command_bytes = bytes.fromhex(command)

        self._ser.write(command_bytes)

        self._last_command = command

        return None

    def send_request(self, request):
        """
        Encode and send request.
        Receive reply bytes, transcode them into integer numbers
        and write them into reply_string dictonary
        """

        request = self._sixpack_addr + request
        request_bytes = bytes.fromhex(request)

        self._ser.write(request_bytes)
        self._last_request = request

        reply_bytes = self._ser.read(9)          # bugging list
        reply_hex = reply_bytes.hex()

        reply_dict = OrderedDict.fromkeys(['addr', 'cmd', 'p0',
                                           'p1', 'p2', 'p3',
                                           'p4', 'p5', 'p6'])

        for i, k in enumerate(reply_dict):
            reply_dict[k] = int(reply_hex[2*i:2*i+2], 16)

        if reply_dict['cmd'] != request[2:4]:
            print('''Error: Response command nr ({0}) does not match
                  requested command nr ({1})
                  '''.format(reply_dict['cmd'], request[2:4]))

        return reply_dict


# =============================================================================
# Encode and decode parameter
# =============================================================================

def encode_param(self, param, num_bytes=4):
    # check whether each given param results in 4/8 (!) digit hex string

    parameter = ''

    max_value = 1 << (8*num_bytes)

    # do this better!
    if param < 0 and 2 * abs(param) < max_value:
        param += max_value
    elif param > max_value:
        raise ValueError('parameter out of range')

    param_hex = '{:x}'.format(param)

    checksum = 2 * num_bytes - len(param_hex)
    if checksum != 0:
        param_hex = checksum * '0' + param_hex

    for i in reversed(range(num_bytes)):
        parameter += param_hex[2*i:2*i+2]

    return parameter


def decode_param(self, param, signed=True):

    max_value = 1 << (8 * len(param))

    value = sum(b << i*8 for i, b in enumerate(param))

    if signed and value >= max_value/2:
        value -= max_value

    return value


# =============================================================================
# Get Unit Information
# =============================================================================

    def get_unit_info(self):
        """
        Allows to read out firmware revision, reset-flag, temperature
        and serail number
        """

        request = '43{0:x}'.format(self._resp_addr) + 6*'00'
        reply = self.send_request(request)

        firmware = reply['p0']
        firmware = '.'.join(str(firmware))

        reset_flag = reply['p1']

        pack_temp = self.decode_param([reply['p2']])

        serial_n = self.decode_param([reply['p3'], reply['p4']], signed=False)

        return firmware, reset_flag, pack_temp, serial_n

    def get_pos(self, motno):
        """
        Queries position and activity of given motor
        (motno: 0...num_motors)
        """

        request = '200{0}{1:x}'.format(motno, self._resp_addr) + 5 * '00'
        reply = self.send_request(request)

        motno = reply['p0']

        posact = self.decode_param([reply['p1'], reply['p2'],
                                    reply['p3'], reply['p4']])

        action = reply['p5']
        self.status_dict['motor{}'.format(motno)] = reply['p5']

        stop_status = reply['p6']

        return motno, posact, action, stop_status

    def get_vel(self, motno):
        """
        Queries velocity and activity of given motor
        (motno: 0...num_motors)
        """

        request = '210{0}{1:x}'.format(motno, self._resp_addr) + 5 * '00'
        reply = self.send_request(request)

        motno = reply['p0']

        velact = self.decode_param([reply['p1'], reply['p2']], signed=False)
        # im windows programm nachschauen ob es auch
        # negative geschwindigkeiten gibt

        action = reply['p3']
        self.status_dict['motor{}'.format(motno)] = reply['p3']

        return motno, velact, action


# =============================================================================
#
# =============================================================================

    def start_ref_search(self, motno):

        """
        Starts search of reference switch
        """

        command = '220{}'.format(motno) + 6 * '00'
        self.send_command(command)

        return None

    def start_ramp(self, motno, targetpos):

        targetpos = self.encode_param(targetpos)

        command = '230{0}{1}'.format(motno, targetpos) + 2 * '00'
        self.send_command(command)

        return None

    def activate_PI_on_targetpos(self, motno, targetpos):

        targetpos = self.encode_param(targetpos)

        command = '240{0}{1}'.format(motno, targetpos) + 2 * '00'
        self.send_command(command)

        return None

    def rotate(self, motno, rotvel):

        rotvel = self.encode_param(rotvel, num_bytes=2)

        command = '250{0}{1}'.format(motno, rotvel) + 4 * '00'
        self.send_command(command)

        return None

    def set_targetpos(self, motno, targetpos):

        targetpos = self.encode_param(targetpos)

        command = '260{0}{1}'.format(motno, targetpos) + 2 * '00'
        self.send_command(command)

        return None

    def set_actualpos(self, motno, posact):

        posact = self.encode_param(posact)

        command = '270{0}{1}'.format(motno, posact) + 2 * '00'
        self.send_command(command)

        return None

    def query_all_motor_activities(self, mask):

        request = '28{0}{1}'.format(self._resp_addr, mask) + 5 * '00'
        reply = self.send_request(request)

        for i in range(self.num_motors):
            self.status_dict['motor{}'.format(i+1)] = reply['p{}'.format(i)]

        return reply

    def start_parallel_ramp(self, mask):

        command = '29{}'.format(mask) + 6 * '00'
        self.send_command(command)

        return None

    def stop_motors(self, mask):

        command = '2A{}'.format(mask) + 6 * '00'
        self.send_command(command)

        return None

    def abort_ref_search(self, motno):
        """
        Aborts reference search of the specified motor
        """

        command = '2B0{}'.format(motno) + 6 * '00'
        self.send_command(command)

        return None


# =============================================================================
# Setting motor parameters
# =============================================================================

    def set_peak_current(self, motno, value):

        if not 0 <= value <= 255:
            raise ValueError('''Value has to be between 0 and 255
                             (value given: {})
                             '''.format(int(value)))

        command = '100{0}{1:x}'.format(motno, value) + 5 * '00'
        self.send_command(command)

        return None

    def control_current(self, motno, T0, *I):
        # check wether I can leave it as *I or get more specific
        # maybe "I=(0, 0, 0, 0)" is better
        # check command formatting
        # for the conversion of I we need a dictonary in constants file

        T0 = self.encode_param(T0, num_bytes=2)
        command = '110%s0%s0%s0%s0%s0%s%s' % (motno, *I, T0)
        self.send_command(command)

        return None

    def set_velocity(self, clkdiv=5):
        # check formatting of encode_param if num_bytes=1, 2 ...

        clkdiv = self.encode_param(clkdiv, num_bytes=1)
        command = '12{}'.format(clkdiv) + 6 * '00'
        self.send_command(command)

        return None

    def set_startvel(self, motno, vmin, vstart, divi):

        vmin = self.encode_param(vmin, num_bytes=2)
        vstart = self.encode_param(vstart, num_bytes=2)

        command = '130{0}{1}{2}0{3}00'.format(motno, vmin, vstart, divi)
        self.send_command(command)

        return None

    def set_velacc(self, motno, amax, vmax):

        amax = self.encode_param(amax, num_bytes=2)
        vmax = self.encode_param(amax, num_bytes=2)

        command = '140{0}{1}{2}0000'.format(motno, amax, vmax)
        self.send_command(command)

        return None

    def set_motparams(self, motno, poslimit, mottype, other):

        poslimit = self.encode_param(poslimit, num_bytes=4)

        cmd = '150{0}{1}{2}{3}'.format(motno, poslimit, mottype, other)
        self.send_command(cmd)

        return None


# =============================================================================
# Closing Serial Port
# =============================================================================

    def __del__(self):
        self._ser.close()
