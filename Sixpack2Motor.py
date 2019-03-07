#!/usr/bin/env python

from weakref import ref
from constants import *


class Sixpack2Motor(object):

    def __init__(self, ctrl, motno):
        self._motno = motno
        r = ref(ctrl)
        self._ctrl = r()

    def get_pos(self):
        """
        Queries position and activity of given motor
        (motno: 0...num_motors)
        """

        request = '200{0}{1}'.format(self._motno, self._ctrl._resp_addr)
        request += 5 * '00'

        reply = self._ctrl._send_request(request)

        if self._motno != reply['p0']:
            raise UserWarning('specified motornumber != response motornumber')

        posact = _decode_param([reply['p1'], reply['p2'],
                                reply['p3'], reply['p4']])

        act = reply['p5']
        if act in ACTION_DICT:
            action = ACTION_DICT[act]
        elif 20 <= act <= 29:
            action = 'reference switch search'
        else:
            raise ValueError('reply action ({0}) seems to be incorrect'
                             .format(act),
                             'and was not found in ACTION_DICT')

        self._ctrl.status_dict['motor{}'.format(self._motno)] = action

        stop_status = reply['p6']

        return posact, action, stop_status

    def get_vel(self):
        """
        Queries velocity and activity of given motor
        (motno: 0...num_motors)
        """

        request = '210{0}{1}'.format(self._motno, self._ctrl._resp_addr)
        request += 5 * '00'
        reply = self._ctrl._send_request(request)

        velact = _decode_param([reply['p1'], reply['p2']], signed=True)

        act = reply['p3']
        if act in ACTION_DICT:
            action = ACTION_DICT[act]
        elif 20 <= act <= 29:
            action = 'reference switch search'
        else:
            raise ValueError('reply action ({0}) seems to be incorrect'
                             .format(act),
                             'and was not found in ACTION_DICT')

        self._ctrl.status_dict['motor{}'.format(self._motno)] = action

        return self._motno, velact, action

    # =============================================================================
    # Moving the motor
    # =============================================================================

    def start_ref_search(self):

        """
        Starts search of reference switch
        """

        cmd = '220{}'.format(self._motno) + 6 * '00'
        self._ctrl._send_command(cmd)

        return None

    def start_ramp(self, targetpos):

        targetpos = _encode_param(targetpos, 'targetpos', num_bytes=4)

        cmd = '230{0}{1}'.format(self._motno, targetpos) + 2 * '00'
        self._ctrl._send_command(cmd)

        return None

    def activate_PI_on_targetpos(self, targetpos):

        targetpos = _encode_param(targetpos, 'targetpos', num_bytes=4)

        cmd = '240{0}{1}'.format(self._motno, targetpos) + 2 * '00'
        self._ctrl._send_command(cmd)

        return None

    def rotate(self, rotvel):

        rotvel = _encode_param(rotvel, 'rotvel', num_bytes=2)

        cmd = '250{0}{1}'.format(self._motno, rotvel) + 4 * '00'
        self._ctrl._send_command(cmd)

        return None

    def set_targetpos(self, targetpos):

        targetpos = _encode_param(targetpos, 'targetpos', num_bytes=4)

        cmd = '260{0}{1}'.format(self._motno, targetpos) + 2 * '00'
        self._ctrl._send_command(cmd)

        return None

    def set_actualpos(self, posact):

        posact = _encode_param(posact, 'posact', num_bytes=4)

        cmd = '270{0}{1}'.format(self._motno, posact) + 2 * '00'
        self._ctrl._send_command(cmd)

        return None

    def abort_ref_search(self):
        """
        Aborts reference search of the specified motor
        """

        cmd = '2B0{}'.format(self._motno) + 6 * '00'
        self._ctrl._send_command(cmd)

        return None

    # =========================================================================
    # Setting motor parameters
    # =========================================================================

    def set_peak_current(self, peak_current):

        peak_current = _encode_param(peak_current, 'peak_current', num_bytes=1)

        command = '100{0}{1}'.format(self._motno, peak_current) + 5 * '00'
        self._send_command(command)

        return None

    def control_current(self, T0, currentlist=[0, 50, 75, 100]):

        I_list = []

        if not type(currentlist) == list:
            raise TypeError('given current list is not a list')
        if len(currentlist) != 4:
            raise ValueError('current list has to be of length 4.',
                             'Current list given: {}'.format(currentlist))

        for i in currentlist:
            try:
                I_list.append(I_DICT[i])
            except KeyError as e:
                print('given percentage of current ({}%) is not in allowed',
                      'values'.format(i),
                      '(allowed values: {}; given current list: {})'
                      .format(I_DICT.keys(), currentlist),
                      '(error msg: {})'.format(e))

        T0 = _encode_param(T0, 'T0', num_bytes=2)
        command = '110{0}0{1}0{2}0{3}0{4}{5}' % (self._motno, *I_list, T0)
        self._send_command(command)

        return None

    def set_startvel(self, vmin, vstart, divi):

        vmin = _encode_param(vmin, 'vmin', num_bytes=2)
        vstart = _encode_param(vstart, 'vstart', num_bytes=2)
        divi = _encode_param(divi, 'divi', num_bytes=1)

        command = '130{0}{1}{2}{3}00'.format(self._motno, vmin, vstart, divi)
        self._send_command(command)

        return None

    def set_velacc(self, amax, vmax):

        amax = _encode_param(amax, 'amax', num_bytes=2)
        vmax = _encode_param(vmax, 'vmax', num_bytes=2)

        command = '140{0}{1}{2}0000'.format(self._motno, amax, vmax)
        self._send_command(command)

        return None

    def set_motparams(self, poslimit, mottype, other):

        poslimit = _encode_param(poslimit, 'poslimit', num_bytes=4)

        cmd = '150{0}{1}{2}{3}'.format(self._motno, poslimit, mottype, other)
        self._send_command(cmd)

        return None

    def ref_search_params(self, vrefmax, debounce, stop_after=0):
        """
        change parameters for fast reference search
        (change only with motors standing still)
        """

        wait = True
        counter = 0
        while wait:
            self._ctrl.query_all_motor_activities()
            checksum = sum(i for i in self._ctrl.status_dict.values())
            if checksum == 0:
                wait = False
            else:
                counter += 1
                print('waiting for all motors to become inactive'
                      + counter * '.')

        vrefmax = _encode_param(vrefmax, 'vrefmax', num_bytes=2)
        # 511 >= vmax >= vrefmax >= vstart

        debounce = _encode_debounce(debounce)

        cmd = '160{0}{1}{2}0{3}'.format(self._motno, vrefmax,
                                        debounce, stop_after)
        self._send_command(cmd)

        return None

    def set_nulloffset_nullrange(self, nulloffset, nullrange):

        nulloffset = _encode_param(nulloffset, 'nulloffset', num_bytes=4)
        nullrange = _encode_param(nullrange, 'nullrange', num_bytes=2)

        cmd = '180{0}{1}{2}'.format(self._motno, nulloffset, nullrange)
        self._send_command(cmd)

        return None

    def set_PI_parameter(self, propdiv, intdiv, intclip, intinpclip):

        propdiv = _encode_param(propdiv, 'propdiv', num_bytes=1)
        intdiv = _encode_param(intdiv, 'intdiv', num_bytes=2)
        intclip = _encode_param(intclip, 'intclip', num_bytes=2)
        intinpclip = _encode_param(intinpclip, 'intinpclip', num_bytes=1)

        cmd = '190{0}{1}{2}{3}{4}'.format(self._motno, propdiv, intdiv,
                                          intclip, intinpclip)
        self._send_command(cmd)

        return None
