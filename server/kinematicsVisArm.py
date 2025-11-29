import numpy as np
import sympy as sp
import math
import variables as v


class KinematicsVisArm:
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

        if self.initial_offset is not None:
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
        positions = []
        transforms = self.compute_dh_matrix(symbolic=True)

        # Substitute only available joints
        angle_subs = {self.θ[i]: float(joint_angles[i])
                      for i in range(len(joint_angles))}

        T = sp.eye(4)
        # Base frame + number of joints defined
        for i in range(len(transforms)):
            T = T @ transforms[i]
            T_num = np.array(T.subs(angle_subs)).astype(np.float64)
            positions.append(T_num[:3, 3])

        pos = positions[-1]
        return np.array(pos)

    def calculate_joint_limits(self, joints: np.ndarray | list) -> np.ndarray:
        if isinstance(joints, list):
            joints = np.array(joints)

        for i, angle in enumerate(joints):
            if not (self.joint_limits[i][0] <= angle <= self.joint_limits[i][1]):
                joints[i] = max(min(angle, self.joint_limits[i]
                                [1]), self.joint_limits[i][0])

        joints = joints.astype(np.float64)
        return joints

    def ik(self, pos, debug=False, offset=[0, 0, 0, 0, 0, 0]):
        x, y, z = pos
        l1, l2, l3, l4, l5, l6 = self.ll

        # 1. Reach Clamping: Project target into workspace if out of reach
        # Max reach from shoulder (approximate sum of link lengths)
        max_reach = l2 + l3 + l4 + l6

        # Vector from shoulder (0, 0, l1) to target (x, y, z)
        r_plane = math.sqrt(x**2 + y**2)
        z_rel = z - l1
        dist_from_shoulder = math.sqrt(r_plane**2 + z_rel**2)

        if dist_from_shoulder > max_reach:
            if debug:
                print(
                    f"Target out of reach ({dist_from_shoulder:.2f} > {max_reach:.2f}). Clamping.")
            scale = max_reach / dist_from_shoulder
            x *= scale
            y *= scale
            z = l1 + z_rel * scale
            # Update pos for error calculation
            pos = [x, y, z]

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
                    if debug:
                        print(
                            f"Unreachable position for t1={math.degrees(t1):.2f}°, phi={math.degrees(phi):.2f}°, dist={dist:.2f} cm, bounds=({abs(L_upper - L_forearm):.2f}, {L_upper + L_forearm:.2f}) cm")
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
                    t4 = self._wrap_angle(gamma1 + gamma2 + delta - phi)

                    geom_angles = [t1, t2, t3, t4, 0.0]

                    if debug:
                        print(
                            f"Geom Angles (rad): {geom_angles}, (deg): {np.degrees(geom_angles)}")

                    if self._check_joint_limits(geom_angles):
                        # Calculate exact error
                        pos_sol = self.forward_kinematics(geom_angles)
                        pos_err = np.linalg.norm(pos_sol - np.array(pos))

                        # Calculate "Comfort Cost"
                        # Penalize angles far from the center of their range
                        cost = 0
                        for i, angle in enumerate(geom_angles[:4]):
                            min_l, max_l = self.joint_limits[i]
                            mid = (min_l + max_l) / 2
                            rng = max_l - min_l
                            if rng > 0:
                                cost += ((angle - mid) / (rng/2)) ** 2

                        if debug:
                            print(
                                f"IK Solution: Angles (deg): {np.degrees(geom_angles)}, Pos Err: {pos_err:.4f} cm, Cost: {cost:.4f}")

                        # Only add if error is small (it should be small by construction)
                        if pos_err < 1.0:  # 1 cm tolerance
                            solutions.append([geom_angles, pos_err, cost])

        # Sort solutions:
        # 1. Prefer valid solutions (low error)
        # 2. Among valid solutions, prefer low cost (comfortable angles)
        solutions.sort(key=lambda x: (
            x[1] > 0.1, x[2] if x[1] <= 0.1 else x[1]))
        
        # # Add offset
        # for i in range(len(solutions)):
        #     angles = solutions[i][0]
        #     angles_offset = [angles[j] + offset[j] for j in range(len(angles))]
        #     solutions[i][0] = angles_offset

        print(f"Total IK solutions found: {len(solutions)}")

        # Return only [angles, err] to maintain compatibility
        return [[s[0], s[1]] for s in solutions]

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

    def pos_within_workspace(self, pos):
        x, y, z = pos
        r = math.sqrt(x**2 + y**2)
        l1, l2, l3, l4, l5, l6 = self.ll

        # Approximate workspace as a cylinder
        r_min = abs(l2 - l3) + l4  # Minimum reach in horizontal plane
        r_max = l2 + l3 + l4 + l6  # Maximum reach in horizontal plane
        z_min = 0  # Base level
        z_max = l1 + l2 + l3 + l4 + l6  # Maximum height

        if r_min <= r <= r_max and z_min <= z <= z_max:
            return True
        else:
            return False

    def clamp_to_workspace(self, pos):
        x, y, z = pos

        x = np.clip(x, v.min_x, v.max_x)
        y = np.clip(y, v.min_y, v.max_y)
        z = np.clip(z, v.min_z, v.max_z)

        return np.array([x, y, z])


def round_trip_test(kin, n=100):
    ok = True

    # Define workspace limits for testing
    # Approximate min/max reach based on link lengths
    l1, l2, l3, l4, l5, l6 = kin.ll
    max_reach = l2 + l3 + l4 + l6
    min_reach = 10.0  # Avoid singularity near base

    for i in range(n):
        print(f"Test {i+1}/{n}")

        # Generate random point in cylindrical coordinates
        r = np.random.uniform(min_reach, max_reach)  # 85% of max reach
        theta = np.random.uniform(-np.pi, np.pi)
        z = np.random.uniform(v.min_z, v.max_z)  # 80% of max height

        x = r * np.cos(theta)
        y = r * np.sin(theta)
        target_pos = np.array([x, y, z])
        if not kin.pos_within_workspace(target_pos):
            print(f"Generated point outside workspace, skipping: {target_pos}")
            continue

        # Solve IK
        q_ik_all = kin.ik(target_pos)

        if len(q_ik_all) == 0:
            print(
                f"No IK solutions found for target: {np.round(target_pos, 3)}")
            ok = False
            continue

        # Choose the best solution (lowest error)
        q_ik, err = q_ik_all[0]

        # Compute FK for the solution
        pos_ik = kin.forward_kinematics(q_ik)

        # Calculate error
        pos_err = np.linalg.norm(target_pos - pos_ik)

        if pos_err > 1.0:  # 1cm tolerance
            ok = False
            print(f"FAIL: Large position error: {pos_err:.3f} cm")

        print(f"Target Pos: {np.round(target_pos, 3)}")
        print(f"IK Sol Pos: {np.round(pos_ik, 3)}")
        print(f"Pos Err:    {pos_err:.3f} cm")
        print(f"IK Angles:  {np.degrees(q_ik)}")
        print("-" * 30)

    print("Round-trip tests passed?", ok)


def main():
    kin = KinematicsVisArm(
        ll=v.ll,
        dh_params=v.dh_params,
        joint_limits=v.joint_limits_rad,
        variables=v.theta_symbols,
        initial_offset=v.initial_offset
    )

    # angle = [0, -45, 80, 45, 0]
    # start_pos = np.deg2rad(angle)
    # end_eff_pos_3 = kin.forward_kinematics(start_pos)
    # print(f"End Effector Position for joints {angle}:")
    # print(np.round(end_eff_pos_3, 3))

    print("Inverse Kinematics Analytic Solutions for a sample target:")
    pos = [5., -26.,  5.]
    theta = kin.ik(pos)
    for i in theta:
        print(np.degrees(i[0]), f"with pos error: {i[1]:.4f} cm")
    if theta:
        best_angles, best_err = min(theta, key=lambda x: x[1])
        print(np.round(np.degrees(best_angles), 3),
              f"with pos error: {best_err:.4f} cm")
    else:
        print("No solution found for sample target.")

    # print("\nRunning Round Trip Tests...")
    # round_trip_test(kin, n=20)


if __name__ == "__main__":
    main()
