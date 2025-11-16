import sympy as sp

# 2DOF:
ll = [11.5, 7.0]
theta_symbols = sp.symbols('θ1 θ2')
initial_offset = [0, 0, 0]
dh_params = [
    {'a': ll[0], 'alpha': 0, 'd': 0, 'theta': theta_symbols[0]},
    {'a': ll[1], 'alpha': 0, 'd': 0, 'theta': theta_symbols[1]},
]
joint_limits = [
    (-80, 65),      # Joint 1
    (-140, 160),    # Joint 2
]

# 5DOF:
ll = [2.0, 10.3, 9.6, 4.0, 2.5, 5.0]
theta_symbols = sp.symbols('θ1 θ2 θ3 θ4 θ5')
initial_offset = [0, 0, 9.5, 1]
dh_params = [
    {'a': 0, 'alpha': 90, 'd': ll[0], 'theta': theta_symbols[0]},
    {'a': ll[1], 'alpha': 0, 'd': 0, 'theta': theta_symbols[1]},
    {'a': ll[2], 'alpha': 0, 'd': 0, 'theta': theta_symbols[2]},
    {'a': ll[3], 'alpha': 90, 'd': ll[4], 'theta': theta_symbols[3]},
    {'a': ll[5], 'alpha': 0, 'd': 0, 'theta': theta_symbols[4]},
]
joint_limits = [
    (-90, 75),      # Base Rotation (Sweep) - Joint 1
    (-90, 90),      # Shoulder - Joint 2
    (-90, 80),      # Elbow - Joint 3
    (-90, 90),      # Wrist Pitch - Joint 4
    (-90, 90),      # Wrist Roll - Joint 5
    # (65, 90)        # Gripper - Joint 6
]

ll = [2.0, 10.3, 9.6, 4.0, 2.5, 5.0]
theta_symbols = sp.symbols('θ1 θ2 θ3 θ4 θ5')
initial_offset = [0, 0, 9.5]
dh_params = [
    {'a': 0, 'alpha': 90, 'd': ll[0], 'theta': theta_symbols[0]},
    {'a': ll[1], 'alpha': 0, 'd': 0, 'theta': theta_symbols[1]},
    {'a': ll[2], 'alpha': 0, 'd': 0, 'theta': theta_symbols[2]},
    {'a': ll[4], 'alpha': 90, 'd': 0, 'theta': theta_symbols[3]},
    {'a': 0, 'alpha': 0, 'd': ll[3] + ll[5], 'theta': theta_symbols[4]},
]
joint_limits = [(-80, 95), (-80, 80), (-90, 80), (-90, 90), (-90, 90)]


def inverse_kinematics(self, joints_init, target_pos, joint_lims=True, tol=1e-3) -> np.ndarray | tuple[np.ndarray, list]:
    joints = joints_init
    Lambda = 12
    Alpha = 1
    xr_desired = target_pos[0:3, 0:3]
    xt_desired = target_pos[0:3, 3]

    x_dot_prev = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    iters = 0
    print("Starting IK loop")

    final_xt = 0
    while (True):
        jac = self.jacobian(joints)
        trans_EF_cur = self.forward_kinematics(joints)

        xr_cur = trans_EF_cur[0:3, 0:3]
        xt_cur = trans_EF_cur[0:3, 3]

        final_xt = xt_cur
        xt_dot = xt_desired - xt_cur

        # Find error rotation matrix
        R = xr_desired @ xr_cur.T

        # convert to desired angular velocity
        v = np.arccos((R[0, 0] + R[1, 1] + R[2, 2] - 1)/2)
        r = (0.5 * np.sin(v)) * np.array([[R[2, 1]-R[1, 2]],
                                          [R[0, 2]-R[2, 0]],
                                          [R[1, 0]-R[0, 1]]])

        # The large constant just tells us how much to prioritize rotation
        xr_dot = 200 * r * np.sin(v)
        xt_dot = xt_dot.reshape((3, 1))
        x_dot = np.vstack((xt_dot, xr_dot))
        x_dot_norm = np.linalg.norm(x_dot)
        if (x_dot_norm > 25):
            x_dot /= (x_dot_norm/25)

        x_dot_change = np.linalg.norm(x_dot - x_dot_prev)
        if (x_dot_change < tol):
            break

        x_dot_prev = x_dot
        joint_change = Alpha * \
            np.linalg.inv(jac.T@jac + Lambda**2 *
                          np.eye(len(self.dh_params))) @ jac.T @ x_dot
        joints += joint_change.flatten()
        if (joint_lims):
            joints = self.calculate_joint_limits(joints)
        iters += 1

    print("Done in {} iterations".format(iters))
    print("Final position is:")
    print(final_xt)

    return joints


def jacobian(self, joint_angles) -> np.ndarray:
    # Compute Jacobian matrix numerically for given joint angles (in degrees)
    angle_subs = {self.θ[i]: np.radians(
        joint_angles[i]) for i in range(len(joint_angles))}

    # Build symbolic Jacobian if not cached
    if self.J is None:
        transforms = self.compute_dh_matrix(symbolic=True)

        # full end-effector transform
        trans_EF = sp.eye(4)
        for mat in transforms:
            trans_EF = trans_EF @ mat
        pos_EF = trans_EF[0:3, 3]

        J = sp.zeros(6, len(self.dh_params))
        for joint in range(len(self.dh_params)):
            trans_joint = sp.eye(4)
            for mat in transforms[: joint + 1]:
                trans_joint = trans_joint @ mat
            z_axis = trans_joint[0:3, 2]
            pos_joint = trans_joint[0:3, 3]

            # Cross product for linear velocity
            Jv = sp.Matrix([
                z_axis[1]*(pos_EF[2] - pos_joint[2]) -
                z_axis[2]*(pos_EF[1] - pos_joint[1]),
                z_axis[2]*(pos_EF[0] - pos_joint[0]) -
                z_axis[0]*(pos_EF[2] - pos_joint[2]),
                z_axis[0]*(pos_EF[1] - pos_joint[1]) -
                z_axis[1]*(pos_EF[0] - pos_joint[0])
            ])
            Jw = z_axis

            J[0:3, joint] = Jv
            J[3:6, joint] = Jw

        self.J = sp.simplify(J)

    # Substitute joint angles and convert to numeric
    J_numeric = self.J.subs(angle_subs)
    return np.array(J_numeric).astype(np.float64)
