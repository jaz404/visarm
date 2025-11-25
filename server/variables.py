import numpy as np
import sympy as sp

ll = [9.0, 10.5, 9.6, 5.5, 2.5, 10.0]
theta_symbols = sp.symbols('θ1 θ2 θ3 θ4 θ5')
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
