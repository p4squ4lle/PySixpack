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


def check_paramrange(parameter, value):
    """Check if value is valid for given parameter_number"""
    p = str(parameter)
    v = int(value)
    DICT = AXIS_PARAMETER if type(p) == str else GLOBAL_PARAMETER
    if p not in DICT:
        raise ValueError("parameter", p, DICT)
    ranges = DICT[p]
    lo = ranges[0]
    hi = ranges[1]
    NOTINRANGE = False
    if not (lo <= v < hi):
            NOTINRANGE = True
    if NOTINRANGE:
        raise ValueError("parameter", repr(p), 'range({}, {})'.format(lo, hi))

    return p, v


def encode_param(param, num_bytes=4):

    max_value = 1 << (8*num_bytes)

    param = int(param)
    if param < 0:
        param += max_value

    param_hex = '{:x}'.format(param)

    checksum = 2 * num_bytes - len(param_hex)
    if checksum != 0:
        param_hex = checksum * '0' + param_hex

    parameter = ''
    for i in reversed(range(num_bytes)):
        parameter += param_hex[2*i:2*i+2]

    return parameter


def decode_param(param, signed=True):

    max_value = 1 << (8 * len(param))

    value = sum(b << i*8 for i, b in enumerate(param))

    if signed and value >= max_value/2:
        value -= max_value

    return value


def encode_mask(mask):
    # if len(mask) > 8:
        # raise Inputerror
    try:
        mask = int(mask, 2)
        return '{:02x}'.format(mask)
    except TypeError as error:
        print('the given mask has to be of type string ({})'
              .format(repr(error)))
