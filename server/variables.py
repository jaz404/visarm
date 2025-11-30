import numpy as np
import sympy as sp

ll = [9.6, 12, 9.6, 5.5, 2.5, 10.0]
theta_symbols = sp.symbols('θ1 θ2 θ3 θ4 θ5')
initial_offset = [0, 0, 0]
dh_params = [
    {'a': 0, 'alpha': np.pi/2, 'd': ll[0], 'theta': theta_symbols[0]},
    {'a': ll[1], 'alpha': 0, 'd': 0,
        'theta': theta_symbols[1] + np.pi/2},
    {'a': ll[2], 'alpha': 0, 'd': 0, 'theta': -theta_symbols[2]},
    {'a': ll[4], 'alpha': np.pi/2, 'd': 0,
        'theta': -theta_symbols[3] + np.pi/2},
    {'a': 0, 'alpha': 0, 'd': ll[3] +
        ll[5], 'theta': theta_symbols[4]},
]
joint_limits_deg = [(-80, 90), (-80, 80),
                    (-90, 90), (-90, 90), (-90, 90)]
joint_limits_rad = [(np.radians(lo), np.radians(hi))
                    for (lo, hi) in joint_limits_deg]
HOME = [0.0, 0.0, 0.0, 0.0, 0.0]
POS_1 = [-6, 29, 77, 90, -10]
POS_2 = [-6, 0, 45, 65, -10]

GRIPPER_OPEN = 0
GRIPPER_CLOSE = 35.0

HSV_DARK_GREEN = {
    "lower": np.array([30, 71, 0]),
    "upper": np.array([54, 255, 110])
}
HSV_BLUE = {
    "lower": np.array([83, 184, 0]),
    "upper": np.array([179, 255, 108])
}
HSV_GREEN = {
    "lower": np.array([46, 200, 0]),
    "upper": np.array([95, 255, 255])
}
HSV_YELLOW = {
    "lower": np.array([16, 160, 115]),
    "upper": np.array([66, 255, 255])
}
# HSV_COPPER = {
#     "lower": np.array([0, 105, 55]),
#     "upper": np.array([14, 255, 180])
# }

BINS = {
    # "HSV_DARK_GREEN": np.array([0, 26.0, 4.0]),
    "HSV_BLUE": np.array([16.0, 26.0, 5.0]),
    "HSV_GREEN": [0, 26.0, 4.0],
    "HSV_YELLOW": [5.0, -26.0, 5.0],
    # "HSV_COPPER": [16.0, -26.0, 5.0],
}

min_x, max_x = -37, 36
min_y, max_y = -37, 36
min_z, max_z = 0, 46.7
