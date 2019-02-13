
NUM_MOTORS = 4
STATUS_DICT = {'motor{}'.format(i+1): None
               for i in range(NUM_MOTORS)}

ACTION_DICT = {0: 'inactive', 5: 'ramping', 10: 'PI-controller',
               15: 'rotation', (20, 29): 'reference switch search',
               30: 'mechanical reference'}


# =============================================================================
# Check parameter range, encode and decode parameter
# =============================================================================


def check_paramrange(self, parameter_number, value, prefix):
    """Check if value is valid for given parameter_number"""
    pn = int(parameter_number)
    v = int(value)
    DICT = AXIS_PARAMETER if type(pn) == int else GLOBAL_PARAMETER
    if not pn in DICT:
        raise TMCLKeyError(prefix, "parameter number", pn, DICT)
    name, ranges, _ = DICT[parameter_number]
    NOTINRANGE = False
    for (l, h) in ranges:
        if not (l <= v < h):
            NOTINRANGE = True
    if NOTINRANGE:
        raise TMCLMissingElement(prefix, "parameter", repr(name),
                                  " + ".join(["range({}, {})".format(l, h)
                                  for l, h in ranges]))
    return pn, v


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
