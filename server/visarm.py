import serial
import time
import glob
import sys
import numpy as np
import sympy as sp
from kinematicsVisArm import KinematicsVisArm
import variables as v


class VisArm:
    def __init__(self):
        self.serial_port = None
        self.ser = None
        self.current_angles = [0.0] * 5

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
            if len(angles) >= 5:
                self.current_angles = angles[:5]
        except ValueError:
            pass

        return self.current_angles

    def clamp_joint_angles(self, angles):
        limits = self.joint_limits
        clamped = list(angles)
        for i in range(min(len(clamped), len(limits))):
            clamped[i] = max(limits[i][0], min(clamped[i], limits[i][1]))
        return clamped

    def set_joint_angles(self, angles):
        if len(angles) < 5:
            print("[VisArm] set_joint_angles requires at least 5 angles")
            return False

        mod_angles = self.clamp_joint_angles(angles)

        cmd_parts = ["SET"]
        for a in mod_angles[:5]:
            cmd_parts.append(str(int(round(a))))
        cmd_parts.append("0")

        cmd = " ".join(cmd_parts)
        resp = self.send_command(cmd)

        if "ERR" in resp:
            print("[VisArm] Error reported.")
            return False
        if "READY" in resp or "OK" in resp:
            print(f"[VisArm] Motion complete ({resp}).")
            return True

        return False

    def set_home(self):
        return self.set_joint_angles([-2, 0, 53, 90, 0, 0])

    def fkine(self, q):
        return self.kinematics.forward_kinematics(q)

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


def main():
    visarm = VisArm()
    if not visarm.connect():
        return

    print("[VisArm] Current joint angles:", visarm.get_joint_angles())
    print("[VisArm] Moving to home position...")
    if visarm.set_home():
        angles = visarm.get_joint_angles()
        print("[VisArm] Moved to home position. Current angles:", angles)
        if angles == [-2.0, 0.0, 53.0, 90.0, 0.0]:
            print("[VisArm] Home position reached.")
        else:
            print("[VisArm] Home position not reached accurately.")
    else:
        print("[VisArm] Failed to move to home position.")

    pos3 = np.array([30.5, 11.7, 5])
    print(f"[VisArm] Computing IK for target position: {pos3}")
    ik_solution = visarm.inv_kinematics(pos3)
    if ik_solution:
        best_angles, best_err = min(ik_solution, key=lambda x: x[1])
        print(np.round(np.degrees(best_angles), 3),
              f"with pos error: {best_err:.4f} cm")
        print("[VisArm] Moving to IK solution...")
        if visarm.set_joint_angles(np.degrees(best_angles)):
            print("[VisArm] Move successful.")
    else:
        print("[VisArm] No IK solution found.")
    time.sleep(20)
    visarm.set_joint_angles([0, 0, 0, 0, 0])

    visarm.disconnect()


if __name__ == "__main__":
    main()
