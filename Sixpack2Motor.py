#!/usr/bin/env python

from weakref import ref
import constants as c


class Sixpack2Motor(object):

    def __init__(self, ctrl, motno=0):
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

        reply = self._ctrl.send_request(request)

        if self._motno != reply['p0']:
            raise UserWarning('specified motornumber != response motornumber')

        posact = c.decode_param([reply['p1'], reply['p2'],
                                 reply['p3'], reply['p4']])

        act = reply['p5']
        if act in c.ACTION_DICT:
            action = c.ACTION_DICT[act]
        elif 20 <= act <= 29:
            action = 'reference switch search'
        else:
            raise ValueError('reply action seems to be incorrect')

        self._ctrl.status_dict['motor{}'.format(self._motno)] = action

        stop_status = reply['p6']

        return posact, action, stop_status

    def get_vel(self):
        """
        Queries velocity and activity of given motor
        (motno: 0...num_motors)
        """

        request = '210{0}{1}'.format(self._motno, self._resp_addr) + 5 * '00'
        reply = self._ctrl.send_request(request)

        motno = reply['p0']

        velact = c.decode_param([reply['p1'], reply['p2']], signed=True)

        act = reply['p3']
        if act in c.ACTION_DICT:
            action = c.ACTION_DICT[act]
        elif 20 <= act <= 29:
            action = 'reference switch search'
        else:
            raise ValueError('reply action seems to be incorrect')

        self._ctrl.status_dict['motor{}'.format(motno)] = action

        return motno, velact, action

    # =============================================================================
    # Moving the motor
    # =============================================================================

    def start_ref_search(self):

        """
        Starts search of reference switch
        """

        cmd = '220{}'.format(self._motno) + 6 * '00'
        self._ctrl.send_command(cmd)

        return None

    def start_ramp(self, targetpos):

        targetpos = c.encode_param(targetpos)

        cmd = '230{0}{1}'.format(self._motno, targetpos) + 2 * '00'
        self._ctrl.send_command(cmd)

        return None

    def activate_PI_on_targetpos(self, targetpos):

        targetpos = c.encode_param(targetpos)

        cmd = '240{0}{1}'.format(self._motno, targetpos) + 2 * '00'
        self._ctrl.send_command(cmd)

        return None

    def rotate(self, rotvel):

        rotvel = c.encode_param(rotvel, num_bytes=2)

        cmd = '250{0}{1}'.format(self._motno, rotvel) + 4 * '00'
        self._ctrl.send_command(cmd)

        return None

    def set_targetpos(self, targetpos):

        targetpos = c.encode_param(targetpos)

        cmd = '260{0}{1}'.format(self._motno, targetpos) + 2 * '00'
        self._ctrl.send_command(cmd)

        return None

    def set_actualpos(self, posact):

        posact = c.encode_param(posact)

        cmd = '270{0}{1}'.format(self._motno, posact) + 2 * '00'
        self._ctrl.send_command(cmd)

        return None

    def abort_ref_search(self):
        """
        Aborts reference search of the specified motor
        """

        cmd = '2B0{}'.format(self._motno) + 6 * '00'
        self._ctrl.send_command(cmd)

        return None

    # =============================================================================
    # Setting motor parameters
    # =============================================================================

    def set_peak_current(self, motno, value):

        if not 0 <= value <= 255:
            raise ValueError('''Value has to be between 0 and 255
                             (value given: {})
                             '''.format(int(value)))

        command = '100{0}{1:02X}'.format(motno, value) + 5 * '00'
        # das hier evtl anders lösen (vlt. value umwandeln mit encode_param)
        self.send_command(command)

        return None

    def control_current(self, motno, T0, *I):
        # check wether I can leave it as *I or get more specific
        # maybe "I=(0, 0, 0, 0)" is better
        # check command formatting
        # for the conversion of I we need a dictonary in constants file

        T0 = c.encode_param(T0, num_bytes=2)
        command = '110%s0%s0%s0%s0%s0%s%s' % (motno, *I, T0)
        self.send_command(command)

        return None

    def set_startvel(self, motno, vmin, vstart, divi):

        vmin = c.encode_param(vmin, num_bytes=2)
        vstart = c.encode_param(vstart, num_bytes=2)

        command = '130{0}{1}{2}0{3}00'.format(motno, vmin, vstart, divi)
        self.send_command(command)

        return None

    def set_velacc(self, motno, amax, vmax):

        amax = c.encode_param(amax, num_bytes=2)
        vmax = c.encode_param(amax, num_bytes=2)

        command = '140{0}{1}{2}0000'.format(motno, amax, vmax)
        self.send_command(command)

        return None

    def set_motparams(self, motno, poslimit, mottype, other):

        poslimit = c.encode_param(poslimit, num_bytes=4)

        cmd = '150{0}{1}{2}{3}'.format(motno, poslimit, mottype, other)
        self.send_command(cmd)

        return None

    def ref_search_params(self, motno, vrefmax, debounce, stop_after=0):
        """
        change parameters for fast reference search
        (change only with motors standing still)
        """

        vrefmax = c.encode_param(vrefmax, num_bytes=2)
        # 511 >= vmax >= vrefmax >= vstart

        debounce = c.encode_param(debounce, num_bytes=2)
        # muss noch ordentlich parametrisiert werden

        cmd = '160{0}{1}{2}0{3}'.format(motno, vrefmax, debounce, stop_after)
        self.send_command(cmd)

        return None

    def set_nulloffset_nullrange(self, motno, nulloffset, nullrange):

        nulloffset = c.encode_param(nulloffset, num_bytes=4)
        nullrange = c.encode_param(nullrange, num_bytes=2)

        cmd = '180{0}{1}{2}'.format(motno, nulloffset, nullrange)
        self.send_command(cmd)

        return None

    def set_PI_parameter(self, motno, propdiv, intdiv, intclip, intinclip):

        propdiv = c.encode_param(propdiv, num_bytes=1)
        intdiv = c.encode_param(intdiv, num_bytes=2)
        intclip = c.encode_param(intclip, num_bytes=2)
        intinclip = c.encode_param(intinclip, num_bytes=1)

        cmd = '190{0}{1}{2}{3}{4}'.format(motno, propdiv, intdiv,
                                          intclip, intinclip)
        self.send_command(cmd)

        return None
