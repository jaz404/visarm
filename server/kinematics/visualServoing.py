import numpy as np
import sympy as sp
import math
import itertools
from scipy.optimize import least_squares


class VisualServoing:
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

    def homogeneous_transform(self, params, symbolic=True):
        a = params['a']
        alpha = params['alpha']   # now ALWAYS radians (float or sympy)
        d = params['d']
        theta = params['theta']   # now ALWAYS radians (float or sympy)

        if symbolic:
            # alpha, theta are directly used (already radians)
            alpha_rad = alpha
            theta_rad = theta

            return sp.Matrix([
                [sp.cos(theta_rad), -sp.sin(theta_rad)*sp.cos(alpha_rad),
                 sp.sin(theta_rad)*sp.sin(alpha_rad), a*sp.cos(theta_rad)],

                [sp.sin(theta_rad),  sp.cos(theta_rad)*sp.cos(alpha_rad),
                 -sp.cos(theta_rad)*sp.sin(alpha_rad), a*sp.sin(theta_rad)],

                [0,                  sp.sin(alpha_rad),
                 sp.cos(alpha_rad), d],

                [0, 0, 0, 1]
            ])

        else:
            # numeric case (NumPy): no conversion, directly use radians
            alpha_rad = alpha
            theta_rad = theta

            return np.array([
                [np.cos(theta_rad), -np.sin(theta_rad)*np.cos(alpha_rad),
                 np.sin(theta_rad)*np.sin(alpha_rad), a*np.cos(theta_rad)],

                [np.sin(theta_rad),  np.cos(theta_rad)*np.cos(alpha_rad),
                 -np.cos(theta_rad)*np.sin(alpha_rad), a*np.sin(theta_rad)],

                [0,                  np.sin(alpha_rad),
                 np.cos(alpha_rad), d],

                [0, 0, 0, 1]
            ])

    def compute_dh_matrix(self, symbolic=True):
        transforms = []

        # initial offset unchanged
        if symbolic:
            t0 = sp.eye(4)
        else:
            t0 = np.eye(4)

        t0[0, 3] = self.initial_offset[0]
        t0[1, 3] = self.initial_offset[1]
        t0[2, 3] = self.initial_offset[2]

        transforms.append(t0)

        # apply transforms
        for param in self.dh_params:
            transforms.append(self.homogeneous_transform(
                param, symbolic=symbolic))

        return transforms

    def forward_kinematics(self, joint_angles) -> np.ndarray:
        if hasattr(joint_angles, 'ndim') and joint_angles.ndim > 1:
            joint_angles = joint_angles[0]

        angle_subs = {self.θ[i]: float(joint_angles[i])
                      for i in range(len(joint_angles))}

        transforms = self.compute_dh_matrix(symbolic=True)
        trans = sp.eye(4)
        for mat in range(len(joint_angles) + 1):
            trans = trans @ transforms[mat]

        trans = trans.subs(angle_subs)
        return np.array(trans).astype(np.float64)

    def calculate_joint_limits(self, joints: np.ndarray | list) -> np.ndarray:
        if isinstance(joints, list):
            joints = np.array(joints)

        for i, angle in enumerate(joints):
            if not (self.joint_limits[i][0] <= angle <= self.joint_limits[i][1]):
                joints[i] = max(min(angle, self.joint_limits[i]
                                [1]), self.joint_limits[i][0])

        joints = joints.astype(np.float64)
        return joints

    def ik(self, pos):
        x, y, z = pos
        l1, l2, l3, l4, l5, l6 = self.ll

        # Correct lengths
        L_upper = l2
        L_forearm = l3
        # Wrist effective length and offset angle
        # Link 4 (l5) is perpendicular to Link 5 (l4+l6)
        L_wrist_eff = math.sqrt(l5**2 + (l4 + l6)**2)
        delta = math.atan2(l5, l4 + l6)

        solutions = []

        # Base rotation candidates
        # 1. Forward
        t1_forward = math.atan2(y, x)
        # 2. Backward (flipped)
        t1_backward = math.atan2(y, x) + math.pi

        t1_cands = [self._wrap_angle(
            t1_forward), self._wrap_angle(t1_backward)]

        # Discretize wrist pitch phi
        # Range can be restricted if we know limits, but full sweep is safer
        # phi is the global angle of the wrist link in the vertical plane
        phis = np.linspace(-np.pi, np.pi, 73)  # 5 degree steps

        for t1 in t1_cands:
            # Transform target to planar coordinates (r, z)
            # r is the signed distance along the arm plane
            r_target = x * math.cos(t1) + y * math.sin(t1)
            z_target = z - l1  # Relative to shoulder height

            for phi in phis:
                # Wrist joint position
                w_r = r_target - L_wrist_eff * math.cos(phi)
                w_z = z_target - L_wrist_eff * math.sin(phi)

                # Solve 2-link IK for (w_r, w_z)
                dist_sq = w_r**2 + w_z**2
                dist = math.sqrt(dist_sq)

                # Check reachability
                if dist > (L_upper + L_forearm) or dist < abs(L_upper - L_forearm):
                    continue

                # Law of cosines for elbow
                cos_theta3 = (dist_sq - L_upper**2 - L_forearm **
                              2) / (2 * L_upper * L_forearm)
                cos_theta3 = max(min(cos_theta3, 1.0), -1.0)

                theta3_mag = math.acos(cos_theta3)

                # Angle of wrist vector
                alpha = math.atan2(w_z, w_r)

                # Angle of upper arm relative to wrist vector
                # Cosine rule again
                # L3^2 = L2^2 + dist^2 - 2*L2*dist*cos(beta)
                cos_beta = (L_upper**2 + dist_sq - L_forearm**2) / \
                    (2 * L_upper * dist)
                cos_beta = max(min(cos_beta, 1.0), -1.0)
                beta = math.acos(cos_beta)

                # Two configurations:
                # 1. gamma2 = +theta3_mag, gamma1 = alpha - beta
                # 2. gamma2 = -theta3_mag, gamma1 = alpha + beta

                configs = [
                    (alpha - beta, theta3_mag),
                    (alpha + beta, -theta3_mag)
                ]

                for gamma1, gamma2 in configs:
                    # Convert to DH angles
                    # theta2 = gamma1 - pi/2
                    t2 = self._wrap_angle(gamma1 - math.pi/2)

                    # theta3 = -gamma2
                    t3 = self._wrap_angle(-gamma2)

                    # theta4
                    # phi = gamma1 + gamma2 + delta - theta4
                    # theta4 = gamma1 + gamma2 + delta - phi
                    t4 = self._wrap_angle(gamma1 + gamma2 + delta - phi)

                    geom_angles = [t1, t2, t3, t4, 0.0]

                    if self._check_joint_limits(geom_angles):
                        # Calculate exact error
                        T_sol = self.forward_kinematics(geom_angles)
                        pos_sol = T_sol[0:3, 3]
                        pos_err = np.linalg.norm(pos_sol - np.array(pos))

                        # Only add if error is small (it should be small by construction)
                        if pos_err < 1.0:  # 1 cm tolerance
                            solutions.append([geom_angles, pos_err])

        print(
            f"Found {len(solutions)} solutions after sweeping phi and filtering.")
        solutions.sort(key=lambda x: x[1])
        return solutions

    def _wrap_angle(self, angle):
        """Wrap angle to [-pi, pi]"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def _angles_equal(self, angles1, angles2, tol=0.01):
        """Check if two angle sets are equal within tolerance"""
        return all(abs(a1 - a2) < tol for a1, a2 in zip(angles1[:4], angles2[:4]))

    def _check_joint_limits(self, joint_angles):
        """Check if joint angles are within limits (in radians)"""
        for i, angle in enumerate(joint_angles[:4]):  # Only check first 4 joints
            min_limit, max_limit = self.joint_limits[i]
            if angle < min_limit or angle > max_limit:
                return False
        return True

def round_trip_test(kin, n=100):
    ok = True
    for _ in range(n):
        print(f"Test {_+1}/{n}")
        q_rand = []
        for (lo, hi) in kin.joint_limits:
            q_rand.append(np.random.uniform(lo, hi))

        q_rand = np.array(q_rand)
        T = kin.forward_kinematics(q_rand)
        pos = T[:3, 3]
        q_ik_all = kin.ik(pos)
        if len(q_ik_all) == 0:
            print("No IK solutions returned")
            ok = False
            continue
        # choose the first solution row
        q_ik, err = q_ik_all[0]
        T2 = kin.forward_kinematics(q_ik)
        pos_err = np.linalg.norm(T[:3, 3] - T2[:3, 3])
        angle_error = np.linalg.norm(q_rand - q_ik)
        if pos_err > 15:
            ok = False

        print(
            f"Pos err: {pos_err:.3f} cm, Angle err: {np.degrees(angle_error):.2f}°")
        print(f"Original Pos: {np.round(pos, 3)}")
        print(f"IK Sol Pos:   {np.round(T2[:3, 3], 3)}")
        print("Original joints (deg):", np.degrees(q_rand))
        print("IK joints (deg):", np.degrees(q_ik))
        print()

    print("Round-trip tests passed?", ok)


def main():
    ll = [10.5, 12.9, 11.0, 0.0, 2.5, 15.0]
    theta_symbols = sp.symbols('θ1 θ2 θ3 θ4 θ5')
    initial_offset = [0, 0, 0]
    dh_params = [
        {'a': 0, 'alpha': np.pi/2, 'd': ll[0], 'theta': theta_symbols[0]},
        {'a': ll[1], 'alpha': 0, 'd': 0, 'theta': theta_symbols[1] + np.pi/2},
        {'a': ll[2], 'alpha': 0, 'd': 0, 'theta': -theta_symbols[2]},
        {'a': ll[4], 'alpha': np.pi/2, 'd': 0,
            'theta': -theta_symbols[3] + np.pi/2},
        {'a': 0, 'alpha': 0, 'd': ll[3] + ll[5], 'theta': theta_symbols[4]},
    ]
    joint_limits = [(-80, 90), (-80, 80), (-90, 90), (-90, 90), (-90, 90)]
    joint_limits = [(np.radians(lo), np.radians(hi))
                    for (lo, hi) in joint_limits]

    kin = VisualServoing(
        ll=ll,
        dh_params=dh_params,
        joint_limits=joint_limits,
        variables=theta_symbols,
        initial_offset=initial_offset
    )

    angle3 = [0, -45, 80, 45, 0]
    start_pos_3 = np.deg2rad(angle3)
    end_eff_pos_3 = kin.forward_kinematics(start_pos_3)
    print(f"End Effector Position for joints {angle3}:")
    print(np.round(end_eff_pos_3, 3))
    pos3 = end_eff_pos_3[0:3, 3]

    print("Inverse Kinematics Analytic Solutions for a sample target:")
    theta = kin.ik(pos3)
    if theta:
        best_angles, best_err = min(theta, key=lambda x: x[1])
        print(np.round(np.degrees(best_angles), 3),
              f"with pos error: {best_err:.4f} cm")
    else:
        print("No solution found for sample target.")

    print("\nRunning Round Trip Tests...")
    round_trip_test(kin, n=5)


if __name__ == "__main__":
    main()
