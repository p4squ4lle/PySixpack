#!/usr/bin/env python

# sachen zum testen: apply_settings(dict) beim initialisieren von 'ser'
#                    ser.read(9)
#                    ser.readlines()
#                    ser.flush()
#                    ser.in_waiting()
#                    ser.out_waiting()

from serial import Serial
from collections import OrderedDict

ser = Serial(port='/dev/ttyUSB3',
             baudrate=19200,
             timeout=5)

x = ser.isOpen()
if x is True:
    print('USB port is open')
else:
    ser.open()
    print('USB port opened')

sixpack_addr = '00'
resp_addr = '00'
num_motors = 4

last_command = ''
last_request = ''

status_dict = {'motor{}'.format(i): None
               for i in range(num_motors)}

action_dict = {0: 'inactive', 5: 'ramping', 10: 'PI-controller',
               15: 'rotation', (20, 29): 'reference switch search',
               30: 'mechanical reference'}


def send_command(command):
    """
    Encode and send command
    """
    print('Sending command..')
    print('----------------------------------')

    command = sixpack_addr + command

    command_bytes = bytes.fromhex(command)


    x = ser.out_waiting()
    print('bytes waiting in output buffer before writing anything: {}'.format(x))
    y = ser.in_waiting()
    print('bytes waiting in input buffer before writing anything: {}'.format(y))

    print('----------------------------------')
    print('Now really sending the command...')

    ser.write(command_bytes)

    print('----------------------------------')
    x = ser.out_waiting()
    print('bytes waiting in output buffer after writing cmd: {}'.format(x))
    y = ser.in_waiting()
    print('bytes waiting in input buffer after writing cmd: {}'.format(y))


    ser.reset_output_buffer()
    ser.write(command_bytes)


    last_command = command

    return last_command


def send_request(request):
    """
    Encode and send request.
    Receive reply bytes, transcode them into integer numbers
    and write them into reply_string dictonary
    """

    print('Sending request..')
    print('----------------------------------')

    request = sixpack_addr + request
    request_bytes = bytes.fromhex(request)


    y = ser.inWaiting()

    print('# of bytes waiting in input buffer before writing anything: {}'.format(y))

    print('Now really sending the request...')
    print('----------------------------------')

    ser.reset_output_buffer()
    ser.write(request_bytes)
    last_request = request

    y = ser.inWaiting()
    print('# of bytes waiting in input buffer after writing request: {}'.format(y))

    reply_bytes = ser.read(9)          # bugging list
    reply_hex = reply_bytes.hex()
    print('----------------------------------')
    y = ser.inWaiting()

    print('# of bytes waiting in input buffer after reading reply: {}'.format(y))

    reply_dict = OrderedDict.fromkeys(['addr', 'cmd', 'p0',
                                       'p1', 'p2', 'p3',
                                       'p4', 'p5', 'p6'])
    
    for i, k in enumerate(reply_dict):
        reply_dict[k] = int(reply_hex[2*i:2*i+2], 16)

    if reply_dict['cmd'] != int(request[2:4], 16):
        print('''Error: Response command nr ({0}) does not match
              requested command nr ({1})
              '''.format(reply_dict['cmd'], request[2:4]))

    return reply_dict


# =============================================================================
# Encode and decode parameter
# =============================================================================

def encode_param(param, num_bytes=4):
    # check whether each given param results in 4/8 (!) digit hex string

    print('Encoding parameter...')
    print('----------------------------------')

    parameter = ''

    max_value = 1 << (8*num_bytes)

    param = int(param)
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


def decode_param(param, signed=True):

    print('Decoding parameter...')
    print('----------------------------------')

    max_value = 1 << (8 * len(param))

    value = sum(b << i*8 for i, b in enumerate(param))

    if signed and value >= max_value/2:
        value -= max_value

    return value


# =============================================================================
# Get Unit Information
# =============================================================================

def get_unit_info():
    """
    Allows to read out firmware revision, reset-flag, temperature
    and serail number
    """

    print('Getting unit info...')
    print('----------------------------------')


    request = '43{0}'.format(resp_addr) + 6*'00'

    reply = send_request(request)

    firmware = reply['p0']
    firmware = '.'.join(str(firmware))

    reset_flag = reply['p1']

    pack_temp = decode_param([reply['p2']])

    serial_n = list(reply.values())[5:9]
    serial_n = decode_param(serial_n, signed=False)

    return firmware, reset_flag, pack_temp, serial_n


def get_pos(motno):
    """
    Queries position and activity of given motor
    (motno: 0...num_motors)
    """

    print('getting position of specified motor....')

    request = '200{0}{1}'.format(motno, resp_addr) + 5 * '00'
    reply = send_request(request)

    motno = reply['p0']
    print('Motornumber: {}'.format(motno))

    posact = decode_param([reply['p1'], reply['p2'],
                           reply['p3'], reply['p4']])
    print('Position of motor {0}: {1}'.format(motno, posact))

    act = reply['p5']
    if act in action_dict:
        action = action_dict[act]
    else:
        action = 'reference switch search'

    print('Action of motor {0}: {1}'.format(motno, action))

    status_dict['motor{}'.format(motno)] = reply['p5']

    stop_status = reply['p6']
    print('Stop status: {}'.format(stop_status))
    return motno, posact, action, stop_status


def get_vel(motno):
        """
        Queries velocity and activity of given motor
        (motno: 0...num_motors)
        """

        request = '210{0}{1}'.format(motno, resp_addr) + 5 * '00'
        reply = send_request(request)

        motno = reply['p0']

        velact = decode_param([reply['p1'], reply['p2']], signed=True)
        # im windows programm nachschauen ob es auch
        # negative geschwindigkeiten gibt

        action = reply['p3']
        if action in action_dict:
            action = action_dict[action]
        else:
            action  = 'reference switch search'

        status_dict['motor{}'.format(motno)] = reply['p3']

        return motno, velact, action


# =============================================================================
# Move Motor
# =============================================================================

def start_ref_search(motno):

    """
    Starts search of reference switch
    """

    command = '220{}'.format(motno) + 6 * '00'
    send_command(command)

    return None


def start_ramp(motno, targetpos):

    targetpos = encode_param(targetpos)

    command = '230{0}{1}'.format(motno, targetpos) + 2 * '00'
    send_command(command)

    return None


def rotate(motno, rotvel):

    rotvel = encode_param(rotvel, num_bytes=2)

    command = '250{0}{1}'.format(motno, rotvel) + 4 * '00'
    send_command(command)

    return None


def stop_motors(mask):

    command = '2A{:02X}'.format(mask) + 6 * '00'
    send_command(command)

    return None


def abort_ref_search(motno):
    """
    Aborts reference search of the specified motor
    """

    command = '2B0{}'.format(motno) + 6 * '00'
    send_command(command)

    return None


# =============================================================================
# Additional Inputs/Outputs
# =============================================================================

def read_input_channels(channelno):

    request = '300{0}{1}'.format(channelno, resp_addr) + 5 * '00'
    reply = send_request(request)

    channelno = reply['p0']

    analogue_value = reply['p1']

    ref_input = reply['p3']

    all_ref_inputs = reply['p4']    # um die Bedeutung herauszubekommen müsste man hier wieder in binär umrechnen

    logic_state_TTLIO1 = reply['p5']

    return reply, channelno, analogue_value, ref_input, all_ref_inputs, logic_state_TTLIO1


def query_all_motor_activities(mask='00'):
        """
        Queries current activity of all motors. The mask specifies the motors
        for which a delayed response is wanted, i.e. the PACK will not send the
        response before all concerned motors are inactive.
        (mask for delayed response: bit 0 = motor 0, ..., bit 5 = motor 5;
         0: motor masked, 1: delayed respond; default: all motors masked)

        The response is written in the status_dict dictonary.
        """

        request = '28{0}{1}'.format(resp_addr, mask) + 5 * '00'
        reply = send_request(request)

        for i in range(num_motors):
            act = reply['p{}'.format(i)]
            if act in action_dict:
                status_dict['motor{}'.format(i)] = action_dict[act]
            elif 20 <= act <= 29:
                    status_dict['motor{}'.format(i)] = 'reference switch'
                    'search'
            else:
                raise UserWarning('reply action ({0}) for motor {1} seems to'
                                  .format(act, i), 'be incorrect and was not'
                                  'found in ACTION_DICT')

        return reply