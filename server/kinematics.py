import numpy as np
import sympy as sp

# Generalized Kinematics class for a robotic arm
# Uses Denavit-Hartenberg parameters for transformations


class Kinematics:
    # (✔)
    def __init__(self,
                 ll: list = None,
                 dh_params: list = None,
                 joint_limits: list = None,
                 variables: sp.symbols = None,
                 initial_offset: list = None):
        # Initialize kinematics parameters here
        self.ll = ll
        self.θ = variables
        self.initial_offset = initial_offset
        self.dh_params = dh_params
        self.joint_limits = joint_limits
        self.speed = 5.0  # Default speed in units/sec
        self.J = None  # Cache for Jacobian matrix

    # (✔)
    def homogeneous_transform(self, params, symbolic=True):
        a = params['a']
        alpha = params['alpha']
        d = params['d']
        theta = params['theta']

        if symbolic:
            # Use SymPy for symbolic computation
            alpha_rad = (alpha * sp.pi / 180) if isinstance(alpha,
                                                            (int, float)) else alpha
            theta_rad = theta  # theta will be a symbol; substitution later will be numeric radians

            return sp.Matrix([
                [sp.cos(theta_rad), -sp.sin(theta_rad)*sp.cos(alpha_rad),
                 sp.sin(theta_rad)*sp.sin(alpha_rad), a*sp.cos(theta_rad)],
                [sp.sin(theta_rad),  sp.cos(theta_rad)*sp.cos(alpha_rad), -
                 sp.cos(theta_rad)*sp.sin(alpha_rad), a*sp.sin(theta_rad)],
                [0,                  sp.sin(alpha_rad),                     sp.cos(
                    alpha_rad),                    d],
                [0,                  0,
                    0,                                    1]
            ])
        else:
            # Use NumPy for numeric computation
            alpha_rad = np.radians(alpha) if isinstance(
                alpha, (int, float)) else alpha
            theta_rad = np.radians(theta) if isinstance(
                theta, (int, float)) else theta

            return np.array([
                [np.cos(theta_rad), -np.sin(theta_rad)*np.cos(alpha_rad),
                 np.sin(theta_rad)*np.sin(alpha_rad), a*np.cos(theta_rad)],
                [np.sin(theta_rad),  np.cos(theta_rad)*np.cos(alpha_rad), -
                 np.cos(theta_rad)*np.sin(alpha_rad), a*np.sin(theta_rad)],
                [0,                  np.sin(alpha_rad),                     np.cos(
                    alpha_rad),                    d],
                [0,                  0,
                    0,                                    1]
            ])

    # (✔)
    def compute_dh_matrix(self, symbolic=True):
        transforms = []
        # Initial offset transform (base height offset)
        if symbolic:
            t0 = sp.eye(4)
            t0[0, 3] = self.initial_offset[0]
            t0[1, 3] = self.initial_offset[1]
            t0[2, 3] = self.initial_offset[2]
        else:
            t0 = np.eye(4)
            t0[0, 3] = self.initial_offset[0]
            t0[1, 3] = self.initial_offset[1]
            t0[2, 3] = self.initial_offset[2]

        transforms.append(t0)
        for param in self.dh_params:
            transforms.append(self.homogeneous_transform(
                param, symbolic=symbolic))
        return transforms

    # (✔)
    def forward_kinematics(self, joint_angles) -> np.ndarray:
        # Build substitution map (convert degrees to radians)
        angle_subs = {self.θ[i]: np.radians(
            joint_angles[i]) for i in range(len(joint_angles))}

        # Multiply symbolic transforms, substitute angles once, then convert to numeric
        transforms = self.compute_dh_matrix(symbolic=True)
        trans = sp.eye(4)
        for mat in range(len(joint_angles) + 1):
            trans = trans @ transforms[mat]

        trans = trans.subs(angle_subs)
        return np.array(trans).astype(np.float64)

    # (✔)
    def calculate_joint_limits(self, joints: np.ndarray | list) -> np.ndarray:
        if isinstance(joints, list):
            joints = np.array(joints)

        for i, angle in enumerate(joints):
            if not (self.joint_limits[i][0] <= angle <= self.joint_limits[i][1]):
                joints[i] = max(min(angle, self.joint_limits[i]
                                [1]), self.joint_limits[i][0])

        joints = joints.astype(np.float64)
        return joints

    def inverse_kinematics_analytic(self, target_pos: np.ndarray) -> np.ndarray:
        """
        Analytical IK for 5-DOF arm using geometric decoupling.
        Strategy:
        1. Compute wrist center (origin of joint 4) by backing out from end-effector
        2. Solve for joints 1-3 using geometric approach (position problem)
        3. Solve for joints 4-5 using orientation (R0_3^T * R_target)
        """
        pos = target_pos[:3, 3]
        rot = target_pos[:3, :3]
        n = len(self.θ)
        joints = np.zeros(n)

        d5 = self.ll[3] + self.ll[5]  # 9.0 cm
        a4 = self.ll[4]  # 2.5 cm
        wrist_center = pos - d5 * rot[:, 2] - a4 * rot[:, 0]
        # Joint 1 rotates about z0, so we look at x-y projection
        joints[0] = np.arctan2(wrist_center[1], wrist_center[0])

        r = np.sqrt(wrist_center[0]**2 + wrist_center[1]**2)

        base_height = self.ll[0] + self.initial_offset[2]
        s = wrist_center[2] - base_height
        L1 = self.ll[1]  # shoulder to elbow (a2)
        L2 = self.ll[2]  # elbow to wrist (a3)

        D_squared = r**2 + s**2
        D = np.sqrt(D_squared)

        # Check reachability
        if D > (L1 + L2) or D < abs(L1 - L2):
            D = np.clip(D, abs(L1 - L2) + 0.01, L1 + L2 - 0.01)
            D_squared = D**2

        cos_q3 = (D_squared - L1**2 - L2**2) / (2 * L1 * L2)
        cos_q3 = np.clip(cos_q3, -1.0, 1.0)
        joints[2] = np.arccos(cos_q3)  # Elbow up configuration

        alpha = np.arctan2(s, r)  # Angle from horizontal to wrist
        beta = np.arctan2(L2 * np.sin(joints[2]), L1 + L2 * np.cos(joints[2]))
        joints[1] = alpha - beta

        transforms = self.compute_dh_matrix(symbolic=True)
        angle_subs = {self.θ[i]: joints[i] for i in range(3)}
        T0_3 = sp.eye(4)
        for i in range(4):  # 0 (base offset) + joints 1,2,3
            T0_3 = T0_3 @ transforms[i]
        T0_3 = np.array(T0_3.subs(angle_subs)).astype(np.float64)
        R0_3 = T0_3[:3, :3]
        R3_5 = R0_3.T @ rot
        joints[3] = np.arctan2(R3_5[0, 2], -R3_5[1, 2]) - np.pi/2
        joints[4] = np.arctan2(R3_5[2, 0], R3_5[2, 1])

        # Convert to degrees and apply joint limits
        joints = np.degrees(joints)
        return self.calculate_joint_limits(joints)


def round_trip_test(kin, n=100):
    ok = True
    for _ in range(n):
        print(f"Test {_+1}/{n}")
        # random within joint limits (radians)
        q_rand = []
        for (lo, hi) in kin.joint_limits:
            q_rand.append(np.radians(np.random.uniform(lo, hi)))
        q_rand = np.array(q_rand)
        # if your forward expects degrees adjust
        T = kin.forward_kinematics(np.degrees(q_rand))
        q_ik = kin.inverse_kinematics_analytic(T)
        T2 = kin.forward_kinematics(q_ik)
        pos_err = np.linalg.norm(T[:3, 3] - T2[:3, 3])
        # orientation error via rotation matrices
        R_err = T[:3, :3] @ T2[:3, :3].T
        angle_err = np.arccos(np.clip((np.trace(R_err)-1)/2, -1, 1))
        if pos_err > 15 or np.degrees(angle_err) > 15:
            print("Failed sample:", pos_err, np.degrees(angle_err))
            ok = False
            break
    print("Round-trip tests passed?", ok)


def main():
    ll = [2.0, 10.3, 9.6, 4.0, 2.5, 5.0]
    theta_symbols = sp.symbols('θ1 θ2 θ3 θ4 θ5')
    initial_offset = [0, 0, 9.5]
    dh_params = [
        {'a': 0, 'alpha': 90, 'd': ll[0], 'theta': theta_symbols[0]},
        {'a': ll[1], 'alpha': 0, 'd': 0,
            'theta': theta_symbols[1] + np.pi/2},
        {'a': ll[2], 'alpha': 0, 'd': 0, 'theta': -theta_symbols[2]},
        {'a': ll[4], 'alpha': 90, 'd': 0,
            'theta': -theta_symbols[3] + np.pi/2},
        {'a': 0, 'alpha': 0, 'd': ll[3] +
            ll[5], 'theta': theta_symbols[4]},
    ]
    joint_limits = [(-80, 90), (-80, 80), (-90, 90), (-90, 90), (-90, 90)]

    kin = Kinematics(
        ll=ll,
        dh_params=dh_params,
        joint_limits=joint_limits,
        variables=theta_symbols,
        initial_offset=initial_offset
    )

    transforms = kin.compute_dh_matrix(symbolic=True)
    t = sp.eye(4)
    print("DH Transformation Matrices (cumulative):")
    for i, mat in enumerate(transforms):
        t = t @ mat            # use mat, not the loop variable named t
        t = sp.simplify(t)     # assign the simplified result
        # print(f"Transform {i}:")
        # sp.pprint(t)

    # sp.pprint(t)

    # round_trip_test(kin, n=100)
    angles = [0,0,0,0,0]
    end_eff_pos = kin.forward_kinematics(angles)
    print(f"End Effector Position for joints {angles}:")
    print(end_eff_pos)

    # joints=kin.inverse_kinematics_analytic(end_eff_pos)
    # print("Inverse Kinematics Result for the above position:")
    # for i in range(len(joints)):
    #     print(f"Joint {i+1}: {joints[i]:.2f} - {angles[i]} = {joints[i]-angles[i]:.2f} degrees")


if __name__ == "__main__":
    main()
