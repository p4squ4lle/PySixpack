# =============================================================================
# Dictonary for decoding action into human readable string
# =============================================================================

ACTION_DICT = {0: 'inactive', 5: 'ramping', 10: 'PI-controller',
               15: 'rotation', (20, 29): 'reference switch search',
               30: 'mechanical reference'}

# =============================================================================
# Current control
# =============================================================================

# key = current in %, value = code to be given

I_DICT = {0: 8,
          9: 7,
          13: 6,
          19: 5,
          25: 4,
          38: 3,
          50: 2,
          75: 1,
          100: 0}

# =============================================================================
# Ranges
# =============================================================================

R_5u = (0, 2**5)
R_8u = (0, 2**8)
R_8u1 = (1, 2**8)
R_9u = (0, 2**9)
R_9u1 = (1, 2**9)
R_10s = (-2**9+1, 2**9)
R_10u = (0, 2**10)
R_15u1 = (1, 2**15)
R_16u = (0, 2**16)
R_16u1 = (1, 2**16)
R_31u = (0, 2**31)
R_32s = (-2**31+1, 2**31)   # +1 or not?

# =============================================================================
# Define ranges for axis and global parameter
# =============================================================================

PARAMETER_RANGES = {'peak_current': R_8u,
                    'T0': R_16u1,
                    'clkdiv': R_5u,
                    'vmin': R_9u,
                    'vstart': R_9u,
                    'divi': (0, 4),
                    'amax': R_15u1,
                    'vmax': R_9u1,
                    'poslimit': R_31u,
                    'vrefmax': R_9u1,
                    'nulloffset': R_32s,
                    'nullrange': R_16u,
                    'propdiv': R_8u1,
                    'intdiv': R_15u1,
                    'intclip': R_15u1,
                    'intinpclip': R_8u,
                    'targetpos': R_32s,
                    'posact': R_32s,
                    'rotvel': R_10s,
                    'analogue_value': R_10u,
                    'channelno': (0, 8),
                    'table_entry': R_8u,
                    'table_pointer': (0, 4, 8, 12),
                    'stop_func_limits': R_10u,
                    'transmitter_delay': (1, 1001),
                    'baudratedivisor': R_16u1,
                    'abort_timeout': (2, 2**16),
                    'unit_address': R_8u,
                    'debounce': R_16u
                    }


# =============================================================================
# Check parameter range, encode and decode parameter
# =============================================================================


def _check_paramrange(value, parameter):
    """
    Check if value lies within corresponding range
    for given parameter_number
    """

    if parameter not in PARAMETER_RANGES.keys():
        raise ValueError('parameter {} not found in dictonary {}'
                         .format(parameter, PARAMETER_RANGES))
    ranges = PARAMETER_RANGES[parameter]
    lo = ranges[0]
    hi = ranges[1]
    INRANGE = True
    if not (lo <= value < hi):
            INRANGE = False

    return INRANGE, lo, hi


def _encode_param(value, paramstr, num_bytes=4):

    inrange, lo, hi = _check_paramrange(value, paramstr)
    if not inrange:
        raise ValueError('parameter {} not in range ({}, {})'
                         .format(paramstr, lo, hi))

    max_value = 1 << (8*num_bytes)

    v = int(value)
    if v < 0:
        v += max_value

    param_hex = '{:X}'.format(v)

    checksum = 2 * num_bytes - len(param_hex)
    if checksum != 0:
        param_hex = checksum * '0' + param_hex

    parameter = ''
    for i in reversed(range(num_bytes)):
        parameter += param_hex[2*i:2*i+2]

    return parameter


def _decode_param(param, signed=True):

    max_value = 1 << (8 * len(param))

    value = sum(b << i*8 for i, b in enumerate(param))

    if signed and value >= max_value/2:
        value -= max_value

    return value


def _encode_mask(mask):

    try:
        maskint = int(mask, 2)
        if maskint >= 64:
            raise ValueError('given mask is ambiguous ({})'.format(mask))
        return '{:02X}'.format(maskint)
    except TypeError as error:
        print('the given mask has to be of type string (error: {})'
              .format(repr(error)))


def _encode_debounce(debounce):

    debounce = int(debounce)
    if debounce not in range(0, 32, 2):
        raise ValueError('Error: reference switch de-bouncing time is',
                         'not in range ( 0, 32, 2)')

    i = int(debounce/2 + 1)
    debounce = 0
    for j in range(i):
        debounce += 2**j

    debounce = _encode_param(debounce, 'debounce', num_bytes=2)

    return debounce
