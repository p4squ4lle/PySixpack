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
R_15u1 = (1, 2**15)
R_16u = (0, 2**16)
R_31u = (0, 2**31)
R_32s = (-2**31+1, 2**31)   # +1 or not?

# =============================================================================
# Define ranges for axis and global parameter
# =============================================================================

AXIS_PARAMETER = {'peak_current': R_8u,
                  'clkdiv': R_5u,
                  'vmin': R_9u,
                  'vstart': R_9u,
                  'divi': (0, 4),
                  'amax': R_15u1,
                  'vmax': R_9u1,
                  'poslimit': R_31u,
                  'vrefmax': R_9u1,
                  'nulloffset': R_32s,
                  'testnull': R_16u,
                  'propdiv': R_8u1,
                  'intdiv': R_15u1,
                  'intclip': R_15u1,
                  'intinpclip': R_8u,
                  'targetpos': R_32s,
                  'posact': R_32s,
                  'rotvel': R_10s,
                  }

GLOBAL_PARAMETER = {}

# =============================================================================
# Check parameter range, encode and decode parameter
# =============================================================================


def check_paramrange(parameter_number, value, prefix):
    """Check if value is valid for given parameter_number"""
    pn = int(parameter_number)
    v = int(value)
    DICT = AXIS_PARAMETER if type(pn) == int else GLOBAL_PARAMETER
    if pn not in DICT:
        raise ValueError(prefix, "parameter number", pn, DICT)
    name, ranges, _ = DICT[parameter_number]
    NOTINRANGE = False
    for (lo, hi) in ranges:
        if not (lo <= v < hi):
            NOTINRANGE = True
    if NOTINRANGE:
        raise ValueError(prefix, "parameter", repr(name),
                                 " + ".join(["range({}, {})".format(lo, hi)
                                             for lo, hi in ranges])
                         )

    return pn, v


def encode_param(param, num_bytes=4):
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


def decode_param(param, signed=True):

    max_value = 1 << (8 * len(param))

    value = sum(b << i*8 for i, b in enumerate(param))

    if signed and value >= max_value/2:
        value -= max_value

    return value
