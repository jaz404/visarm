import numpy as np
import sympy as sp


class VisualServoing:
    # (✔)
    def __init__(self,
                 ll: list = None,
                 dh_params: list = None,
                 joint_limits: list = None,
                 variables: sp.symbols = None,
                 initial_offset: list = None):
        self.ll = ll
        self.θ = variables
        self.initial_offset = initial_offset
        self.dh_params = dh_params
        self.joint_limits = joint_limits
        self.speed = 5.0  # Default speed in units/sec
        self.J = None  # Cache for Jacobian matrix
        self.L = None

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

    # (✔)
    def jacobian_sym(self):
        if self.J is not None:
            return self.J

        n = len(self.θ)
        transforms = self.compute_dh_matrix(symbolic=True)

        # Position of the end effector
        T_end = sp.eye(4)
        for i in range(n + 1):
            T_end = T_end @ transforms[i]
        P_end = T_end[:3, 3]

        J = sp.Matrix.zeros(6, n)

        Z = sp.Matrix([0, 0, 1])  # Initial Z axis
        O = sp.Matrix([0, 0, 0])  # Initial origin

        for i in range(n):
            T_i = sp.eye(4)
            for j in range(i + 1):
                T_i = T_i @ transforms[j]
            R_i = T_i[:3, :3]
            O_i = T_i[:3, 3]
            Z_i = R_i * Z

            Jv = Z_i.cross(P_end - O_i)
            Jw = Z_i

            J[:3, i] = Jv
            J[3:, i] = Jw

        self.J = J
        return J

    # (✔)
    def jacobian_numeric(self, joint_angles):
        J_sym = self.jacobian_sym()
        angle_subs = {self.θ[i]: np.radians(
            joint_angles[i]) for i in range(len(joint_angles))}
        J_num = J_sym.subs(angle_subs)
        J_num = np.array(J_num).astype(np.float64)
        return J_num

    def set_calibration(self, L):

        self.L = L

    # def inverse_kinematics_broyden(self, features):
        


def main():
    ll = [2.0, 10.3, 9.6, 4.0, 2.5, 5.0]
    theta_symbols = sp.symbols('θ1 θ2 θ3 θ4 θ5')
    initial_offset = [0, 0, 0]
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

    kin = VisualServoing(
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
        print(f"Transform {i}:")
        # sp.pprint(t)

    # sp.pprint(t)

    # angles = [0, 0, 0, 0, 0]
    # end_eff_pos = kin.forward_kinematics(angles)
    # print(f"End Effector Position for joints {angles}:")
    # print(end_eff_pos)

    print("Jacobian Matrix at zero angles:")
    J_num = kin.jacobian_sym()
    sp.pprint(J_num)


if __name__ == "__main__":
    main()
