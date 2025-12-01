import serial
import time
import glob
import sys
import numpy as np
import sympy as sp
from scipy.interpolate import CubicSpline
from kinematicsVisArm import KinematicsVisArm
import variables as v


class VisArm:
    def __init__(self):
        self.serial_port = None
        self.ser = None
        # Track all 6 joints including gripper
        self.current_angles = [0.0] * 6

        self.kinematics = KinematicsVisArm(
            ll=v.ll,
            dh_params=v.dh_params,
            joint_limits=v.joint_limits_rad,
            variables=v.theta_symbols,
        )
        self.joint_limits = v.joint_limits_deg

    def find_serial_ports(self):
        if sys.platform.startswith('win'):
            ports = ['COM%s' % (i + 1) for i in range(256)]
        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            ports = glob.glob('/dev/tty[A-Za-z]*')
        elif sys.platform.startswith('darwin'):
            ports = glob.glob('/dev/tty.*')
        else:
            raise EnvironmentError('Unsupported platform')

        result = []
        for port in ports:
            try:
                if "ttyACM" in port or "ttyUSB" in port or "COM" in port:
                    result.append(port)
            except:
                pass
        return result

    def connect(self, port=None):
        if port:
            return self.open_serial_port(port)

        print("[VisArm] Auto-detecting Arduino...")
        ports = self.find_serial_ports()
        if not ports:
            print("[VisArm] No serial ports found")
            return False

        for p in ports:
            print(f"[VisArm] Trying {p}...")
            if self.open_serial_port(p):
                time.sleep(2)
                self.ser.reset_input_buffer()

                resp = self.send_command("GET")
                if "ANGLES" in resp:
                    print(f"[VisArm] Found Arduino on {p}")
                    self.serial_port = p
                    return True

                self.disconnect()

        print("[VisArm] Could not find Arduino on any port")
        return False

    def open_serial_port(self, port):
        try:
            self.ser = serial.Serial(
                port,
                115200,
                timeout=1,
                write_timeout=1
            )
            time.sleep(2)
            self.ser.reset_input_buffer()
            print(f"[VisArm] Connected to {port} at 115200 baud")
            return True
        except serial.SerialException as e:
            print(f"[VisArm] Error opening {port}: {e}")
            return False

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.ser = None
            print("[VisArm] Disconnected from Arduino")

    def send_command(self, cmd):
        if not self.ser or not self.ser.is_open:
            print("[VisArm] Serial not connected")
            return ""

        self.ser.reset_input_buffer()

        msg = cmd
        if not msg.endswith('\n'):
            msg += '\n'
        self.ser.write(msg.encode('utf-8'))
        self.ser.flush()

        is_set = cmd.startswith("SET")
        timeout = 30 if is_set else 2

        start_time = time.time()
        full_log = ""

        while (time.time() - start_time) < timeout:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                if line == "READY" or line == "OK":
                    return line
                if line.startswith("ANGLES"):
                    return line
                if line.startswith("ERR"):
                    return line

                full_log += line + "\n"
            else:
                time.sleep(0.01)

        if is_set:
            print(
                f"[VisArm] Timeout waiting for READY. Collected output:\n{full_log}")
        return ""

    def get_joint_angles(self):
        resp = self.send_command("GET")
        if not resp.startswith("ANGLES"):
            print(f"[VisArm] Invalid GET response: {resp}")
            return self.current_angles

        parts = resp.split()
        try:
            angles = [float(x) for x in parts[1:]]
            if len(angles) >= 6:
                self.current_angles = angles[:6]
        except ValueError:
            pass

        return self.current_angles

    def clamp_joint_angles(self, angles):
        limits = self.joint_limits
        clamped = list(angles)
        for i in range(min(len(clamped), len(limits))):
            clamped[i] = max(limits[i][0], min(clamped[i], limits[i][1]))
        return clamped

    def set_joint_angles(self, angles, order=None):
        # Expect 6 angles including gripper
        if len(angles) < 6:
            print("[VisArm] set_joint_angles requires 6 angles (including gripper)")
            return False

        mod_angles = self.clamp_joint_angles(angles)

        # Ensure we have exactly 6 angles
        target_angles = mod_angles[:6]
        while len(target_angles) < 6:
            target_angles.append(0)

        # Move in a safe order; gripper last
        current_temp = list(self.current_angles)
        if order is not None:
            for idx in order:
                if idx >= len(target_angles):
                    continue

                target_val = target_angles[idx]
                current_val = current_temp[idx]

                if abs(target_val - current_val) > 1.0:  # Threshold
                    print(
                        f"[VisArm] Moving joint {idx+1} from {current_val} to {target_val}")
                    # Update just this joint in our temp state
                    current_temp[idx] = target_val

                    # Send command for this intermediate state (6 angles)
                    cmd_parts = ["SET"]
                    for a in current_temp:
                        cmd_parts.append(str(int(round(a))))
                    # Optional duration/speed token (ignored by client)
                    cmd_parts.append("0")

                    cmd = " ".join(cmd_parts)
                    resp = self.send_command(cmd)
                    if "ERR" in resp:
                        print(f"[VisArm] Error moving joint {idx+1}")
                        return False
        else:
            # Single command for all joints
            cmd_parts = ["SET"]
            for a in target_angles:
                cmd_parts.append(str(int(round(a))))
            # Optional duration/speed token (ignored by client)
            cmd_parts.append("0")

            cmd = " ".join(cmd_parts)
            resp = self.send_command(cmd)
            if "ERR" in resp:
                print(f"[VisArm] Error setting joint angles")
                return False

        # Update internal state
        self.current_angles = target_angles
        return True

    def move_to_angles(self, angles, update_gripper=False, order=None):
        """
        Wrapper to handle gripper preservation.
        """
        if not update_gripper:
            # Use current gripper angle
            current_gripper = self.current_angles[5] if len(
                self.current_angles) >= 6 else 0
            # Make a copy and update the 5th element
            angles = list(angles)
            if len(angles) < 6:
                # pad to 6 and set gripper
                while len(angles) < 5:
                    angles.append(0)
                angles.append(current_gripper)
            else:
                angles[5] = current_gripper

        angles[4] = -10  # fix joint 5 to -10 degrees

        return self.set_joint_angles(angles, order=order)

    def set_home(self):
        # Reverse order: 1,2,3,0,4 (skip gripper)
        return self.move_to_angles(v.HOME, update_gripper=False, order=[1, 2, 3, 0, 4])

    def set_survey(self):
        # Reverse order: 1,2,3,0,4 (skip gripper)
        return self.move_to_angles(v.POS_1, update_gripper=False, order=[1, 2, 3, 0, 4])

    def set_pos_2(self):
        # Reverse order: 1,2,3,0,4 (skip gripper)
        return self.move_to_angles(v.POS_2, update_gripper=False, order=[1, 2, 3, 0, 4])

    def fkine(self, q):
        # Kinematics are defined for arm joints (exclude gripper)
        return self.kinematics.forward_kinematics(q[:5])

    def get_eMc(self):
        t = np.eye(4)
        t[0, 0] = -1
        t[1, 1] = -1
        t[0, 3] = 10.0
        return t

    def get_rMe(self):
        q = self.get_joint_angles()
        print(f"[VISARM] Joint angles for FK: {q}")
        return self.fkine(q)

    def get_base_to_camera(self):
        rMe = self.get_rMe()
        eMc = self.get_eMc()
        return rMe @ eMc

    def inv_kinematics(self, target_pos):
        return self.kinematics.ik(target_pos)

    def spline_path(self, start_pos, end_pos, num_points=6, height=18.0):
        start_pos = np.array(start_pos, dtype=float)
        end_pos = np.array(end_pos, dtype=float)

        t = np.linspace(0, 1, num_points)

        x = (1 - t) * start_pos[0] + t * end_pos[0]
        y = (1 - t) * start_pos[1] + t * end_pos[1]

        z_apex = height
        z_control_t = np.array([0, 0.5, 1.0], dtype=float)
        z_control_z = np.array([start_pos[2], z_apex, end_pos[2]], dtype=float)

        cs = CubicSpline(z_control_t, z_control_z, bc_type='natural')
        z = cs(t)

        path = np.column_stack((x, y, z))

        # --- Workspace enforcement ---
        corrected_path = []
        for p in path:
            if self.kinematics.pos_within_workspace(p):
                corrected_path.append(p)
            else:
                corrected_path.append(self.kinematics.clamp_to_workspace(p))
        corrected_path = np.round(np.array(corrected_path), 3)
        return np.array(corrected_path[1:])

    def move_to_position(self, target_pos, steps=6, order=None, add_offset=None):
        self.set_home()  # Move to a safe position before pathing
        time.sleep(2)

        if order is not None:
            self.order = order

        curr_pos = self.fkine(self.get_joint_angles())
        print(f"[VisArm] Moving from {curr_pos} to {target_pos}")
        path_points = self.spline_path(
            curr_pos, target_pos, num_points=int(steps))

        for point in path_points:
            ik_solutions = self.inv_kinematics(point)
            if ik_solutions:
                # IK solutions are now sorted by "comfort" (cost) then error
                # So we just pick the first one
                best_angles, best_err = ik_solutions[0]
                print(
                    f"[VisArm] Moving to point {point} with angles {np.degrees(best_angles)} and error {best_err}")

                # Use move_to_angles to preserve gripper state
                if add_offset is not None:
                    best_angles = [best_angles[i] + add_offset[i]
                                   for i in range(len(best_angles))]

                success = self.move_to_angles(
                    np.degrees(best_angles), update_gripper=False)

                if not success:
                    print("[VisArm] Failed to move to point.")
                    return False
                time.sleep(0.5)  # Small delay between moves
            else:
                print(
                    f"[VisArm] No IK solution found for point {point}. Aborting path.")
                return False
        return True

    def close_gripper(self):
        curr_angles = self.get_joint_angles()
        print(f"[VisArm] close_gripper: Current angles: {curr_angles}")
        new_angles = list(curr_angles)
        if len(new_angles) < 6:
            while len(new_angles) < 6:
                new_angles.append(0)
        new_angles[5] = v.GRIPPER_CLOSE
        self.set_joint_angles(new_angles)

    def open_gripper(self):
        curr_angles = self.get_joint_angles()
        print(f"[VisArm] open_gripper: Current angles: {curr_angles}")
        new_angles = list(curr_angles)
        if len(new_angles) < 6:
            while len(new_angles) < 6:
                new_angles.append(0)
        new_angles[5] = v.GRIPPER_OPEN
        self.set_joint_angles(new_angles)


def main():
    visarm = VisArm()
    if not visarm.connect():
        return

    # set to survey position
    visarm.set_survey()
    # time.sleep(10)
    # visarm.set_pos_2()

    # keys = v.BINS.keys()
    # for key in keys:
    #     print(f"\n[TEST] Moving to bin: {key}")
    #     bin_params = v.BINS[key]
    #     pos = [bin_params[0], bin_params[1], bin_params[2]]
    #     visarm.set_survey()
    #     time.sleep(2)
    #     visarm.move_to_position(pos, steps=2)
    #     time.sleep(10)

    # visarm.close_gripper()
    # time.sleep(2)
    # visarm.open_gripper()
    while True:
        try:
            pass
        except KeyboardInterrupt:
            break

    # visarm.move_to_position(pos, steps=2)
    # time.sleep(10)
    visarm.set_home()

    visarm.disconnect()


if __name__ == "__main__":
    main()
