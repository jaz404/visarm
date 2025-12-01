"""
Interactive 5-DOF Robotic Arm Visualizer

Features:
- 3D visualization of the robot arm using matplotlib
- Click anywhere in 3D space to set a target point
- The arm will move to that point using inverse kinematics
- Real-time joint angle display
- Workspace boundary visualization
- Rotate/zoom/pan the view

Controls:
- Left click: Set target position for end-effector
- Right drag: Rotate view
- Middle drag: Pan view
- Scroll: Zoom in/out
- 'r' key: Reset to home position [0,0,0,0,0]
- 'w' key: Toggle workspace boundary visualization
- 'q' key: Quit

Author: Generated for visarm project
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, Slider
import sympy as sp
import visarm
import variables as v

vis = visarm.VisArm()
vis.connect()


class ArmVisualizer:
    def __init__(self):
        """Initialize a simplified 2-DOF arm visualizer (first two joints only)."""
        self.kin = vis.kinematics
        self.joint_angles = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        # Track gripper separately so we always send 6 angles to the robot
        try:
            current = vis.get_joint_angles()
            self.gripper_angle = float(
                current[5]) if len(current) >= 6 else 0.0
        except Exception:
            self.gripper_angle = 0.0

        # Target point
        self.target_point = None

        # Setup matplotlib figure
        self.fig = plt.figure(figsize=(14, 10))
        self.ax = self.fig.add_subplot(111, projection='3d')

        # Visual elements
        self.arm_lines = None
        self.joint_scatter = None
        self.ee_scatter = None
        self.target_scatter = None
        self.workspace_artists = []
        self.show_workspace = False

        # Setup UI
        self.setup_ui()

        # Connect event handlers
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)

        # Initial draw
        self.update_arm()

    def setup_ui(self):
        """Setup the user interface elements."""
        # Adjust main plot position to make room for sliders
        self.ax.set_position([0.1, 0.15, 0.7, 0.75])

        # Title and labels
        self.ax.set_xlabel('X (cm)')
        self.ax.set_ylabel('Y (cm)')
        self.ax.set_zlabel('Z (cm)')
        self.ax.set_title(
            '5-DOF Robotic Arm Visualizer\nLeft-click to set target position')

        # Set equal aspect ratio and limits
        max_reach = 35
        self.ax.set_xlim([-max_reach, max_reach])
        self.ax.set_ylim([-max_reach, max_reach])
        self.ax.set_zlim([0, max_reach])

        # Create sliders for manual joint control
        slider_height = 0.03
        slider_left = 0.82
        slider_width = 0.15

        self.sliders = []
        for i in range(len(self.joint_angles)):
            ax_slider = plt.axes([
                slider_left, 0.75 - i*0.08, slider_width, slider_height
            ])
            lo, hi = v.joint_limits_deg[i]
            label = f"J{i+1} [{lo}°, {hi}°]"
            slider = Slider(
                ax_slider, label, lo, hi,
                valinit=self.joint_angles[i], valstep=1.0
            )
            slider.on_changed(
                lambda val, idx=i: self.on_slider_change(idx, val))
            self.sliders.append(slider)

        # Place control buttons above the Reset button (which is at y=0.08)
        ax_pos1 = plt.axes([slider_left, 0.28, slider_width, 0.04])
        self.btn_pos1 = Button(ax_pos1, 'Move to POS_1')
        self.btn_pos1.on_clicked(self.move_to_pos1)

        ax_pos2 = plt.axes([slider_left, 0.23, slider_width, 0.04])
        self.btn_pos2 = Button(ax_pos2, 'Move to POS_2')
        self.btn_pos2.on_clicked(self.move_to_pos2)

        ax_open = plt.axes([slider_left, 0.18, slider_width, 0.04])
        self.btn_open = Button(ax_open, 'Open Gripper')
        self.btn_open.on_clicked(self.open_gripper)

        ax_close = plt.axes([slider_left, 0.13, slider_width, 0.04])
        self.btn_close = Button(ax_close, 'Close Gripper')
        self.btn_close.on_clicked(self.close_gripper)

        # Add reset button
        ax_reset = plt.axes([slider_left, 0.08, slider_width, 0.04])
        self.btn_reset = Button(ax_reset, 'Reset to Home')
        self.btn_reset.on_clicked(self.reset_to_home)

        # Add workspace toggle button
        ax_workspace = plt.axes([slider_left, 0.03, slider_width, 0.04])
        self.btn_workspace = Button(ax_workspace, 'Toggle Workspace')
        self.btn_workspace.on_clicked(self.toggle_workspace)

        # Info text
        self.info_text = self.fig.text(
            0.05, 0.25, '',
            fontsize=11,
            family='monospace',
            verticalalignment='top'
        )

    def compute_link_positions(self, joint_angles):
        """Compute link positions for the 2-joint configuration."""
        positions = []
        transforms = self.kin.compute_dh_matrix(symbolic=True)

        # Substitute only available joints
        angle_subs = {self.kin.θ[i]: np.radians(joint_angles[i])
                      for i in range(len(joint_angles))}

        T = sp.eye(4)
        # Base frame + number of joints defined
        for i in range(len(transforms)):
            T = T @ transforms[i]
            T_num = np.array(T.subs(angle_subs)).astype(np.float64)
            positions.append(T_num[:3, 3])

        return np.array(positions)

    def get_end_effector_rotation(self, joint_angles):
        """Return the end-effector rotation matrix (3x3) for current angles."""
        transforms = self.kin.compute_dh_matrix(symbolic=True)
        angle_subs = {self.kin.θ[i]: np.radians(joint_angles[i])
                      for i in range(len(joint_angles))}

        T = sp.eye(4)
        for Ti in transforms:
            T = T @ Ti
        T_num = np.array(T.subs(angle_subs)).astype(np.float64)
        return T_num[:3, :3]

    def update_arm(self):
        """Update the arm visualization."""
        # Get link positions
        positions = self.compute_link_positions(self.joint_angles)

        # Clear previous arm rendering
        if self.arm_lines:
            self.arm_lines.remove()
        if self.joint_scatter:
            self.joint_scatter.remove()
        if self.ee_scatter:
            self.ee_scatter.remove()

        # Draw arm links
        self.arm_lines = self.ax.plot(
            positions[:, 0], positions[:, 1], positions[:, 2],
            'o-', linewidth=3, markersize=6, color='royalblue',
            label='Robot Arm'
        )[0]

        # Highlight joints
        self.joint_scatter = self.ax.scatter(
            positions[:-1, 0], positions[:-1, 1], positions[:-1, 2],
            c='darkblue', s=100, alpha=0.8, label='Joints'
        )

        # Highlight end-effector
        ee_pos = positions[-1]
        # Draw claws as two fingers along local z-axis; lateral offset rotates with J5
        finger_len = 3.0
        half_width = 1.0

        R_ee = self.get_end_effector_rotation(self.joint_angles)
        x_ee = R_ee[:, 0]
        z_ee = R_ee[:, 2]

        p1_start = ee_pos + x_ee * half_width
        p1_end = p1_start + z_ee * finger_len
        p2_start = ee_pos - x_ee * half_width
        p2_end = p2_start + z_ee * finger_len

        xs = [p1_start[0], p1_end[0], np.nan, p2_start[0], p2_end[0]]
        ys = [p1_start[1], p1_end[1], np.nan, p2_start[1], p2_end[1]]
        zs = [p1_start[2], p1_end[2], np.nan, p2_start[2], p2_end[2]]

        # Single Line3D artist containing both claw segments (NaN separates segments)
        self.ee_scatter = self.ax.plot(
            xs, ys, zs, color='red', linewidth=3)[0]
        # Draw target if set
        if self.target_point is not None:
            if self.target_scatter:
                self.target_scatter.remove()
            self.target_scatter = self.ax.scatter(
                [self.target_point[0]], [self.target_point[1]], [
                    self.target_point[2]],
                c='lime', s=150, marker='X', label='Target'
            )

        # Update info text
        ee_pos_str = f"[{ee_pos[0]:.1f}, {ee_pos[1]:.1f}, {ee_pos[2]:.1f}]"
        target_str = "None"
        if self.target_point is not None:
            target_str = f"[{self.target_point[0]:.1f}, {self.target_point[1]:.1f}, {self.target_point[2]:.1f}]"

        # Build table-like text
        info = "Joint   | Angle (°)\n"
        info += "-------------------\n"
        for i, angle in enumerate(self.joint_angles):
            info += f"J{i+1:<6}| {angle:6.1f}\n"

        info += "\nEE Pos  | " + ee_pos_str + "\n"
        info += "Target  | " + target_str + "\n"

        if self.target_point is not None:
            dist = np.linalg.norm(ee_pos - self.target_point)
            info += f"Error   | {dist:.2f} cm"

        self.info_text.set_text(info)

        # Update sliders without triggering events
        for i, slider in enumerate(self.sliders):
            slider.eventson = False
            slider.set_val(self.joint_angles[i])
            slider.eventson = True

        # Redraw
        self.ax.legend(loc='upper left')
        self.fig.canvas.draw_idle()

        # send to robot
        angles6 = list(self.joint_angles) + [self.gripper_angle]
        vis.set_joint_angles(angles6)

    def on_slider_change(self, joint_idx, value):
        """Handle slider change event."""
        self.joint_angles[joint_idx] = value
        self.update_arm()

    def on_click(self, event):
        """Handle mouse click events."""
        if event.inaxes != self.ax:
            return

        if event.button == 1:  # Left click
            # Get click coordinates in 3D
            # This is tricky with matplotlib 3D - we'll use a simple heuristic
            # Click at a specific depth based on current view

            # Project click to 3D space
            # For simplicity, we'll place the point at a fixed distance from camera
            # Users can rotate the view to select points in different planes

            if event.xdata is not None and event.ydata is not None:
                # Use the event data directly as x, y
                x, y = event.xdata, event.ydata

                # For z, use a heuristic: place it at the middle height
                z = 20.0

                # Alternatively, let user set z with the existing target height
                if self.target_point is not None:
                    z = self.target_point[2]

                self.target_point = np.array([x, y, z])
                print(f"Target set to: [{x:.2f}, {y:.2f}, {z:.2f}]")

                # Try to move arm to target
                self.move_to_target()

    def move_to_target(self):
        """Move the arm to the target point using IK."""

        if self.target_point is None:
            return

        T_target = np.eye(4)
        T_target[:3, 3] = self.target_point

        T_target[:3, :3] = np.array([
            [0, 0, 1],
            [0, -1, 0],
            [1, 0, 0]
        ])

        try:
            joint_angles_ik = self.kin.inverse_kinematics_analytic(T_target)
            self.joint_angles = joint_angles_ik
            self.update_arm()
            print(f"IK solution: {joint_angles_ik}")

        except Exception as e:
            print(f"IK failed: {e}")
            print("Target may be unreachable")

    def on_key(self, event):
        """Handle keyboard events."""
        if event.key == 'r':
            self.reset_to_home()
        elif event.key == 'w':
            self.toggle_workspace()
        elif event.key == 'q':
            plt.close(self.fig)
        elif event.key == 'up':
            # Increase target z
            if self.target_point is not None:
                self.target_point[2] += 2.0
                self.move_to_target()
        elif event.key == 'down':
            # Decrease target z
            if self.target_point is not None:
                self.target_point[2] = max(0, self.target_point[2] - 2.0)
                self.move_to_target()

    def reset_to_home(self, event=None):
        """Reset arm to home position (2 joints)."""
        self.joint_angles = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.target_point = None
        if self.target_scatter:
            self.target_scatter.remove()
            self.target_scatter = None
        self.update_arm()
        print("Reset to home position (2-joint mode)")

    def toggle_workspace(self, event=None):
        """Toggle workspace boundary visualization."""
        self.show_workspace = not self.show_workspace

        # Clear existing workspace visualization
        for artist in self.workspace_artists:
            artist.remove()
        self.workspace_artists.clear()

        if self.show_workspace:
            # Draw workspace boundary (approximate)
            # Sample points on the workspace boundary
            theta = np.linspace(0, 2*np.pi, 50)

            # Top circle (max reach at high z)
            r_max = 30
            z_high = 30
            x_top = r_max * np.cos(theta)
            y_top = r_max * np.sin(theta)
            z_top = np.full_like(x_top, z_high)

            circle_top = self.ax.plot(
                x_top, y_top, z_top, 'g--', alpha=0.3, linewidth=1)[0]
            self.workspace_artists.append(circle_top)

            # Bottom circle (at base height)
            z_low = 10
            circle_bottom = self.ax.plot(x_top, y_top, np.full_like(
                x_top, z_low), 'g--', alpha=0.3, linewidth=1)[0]
            self.workspace_artists.append(circle_bottom)

            # Vertical lines connecting them
            for i in range(0, len(theta), 10):
                line = self.ax.plot(
                    [x_top[i], x_top[i]],
                    [y_top[i], y_top[i]],
                    [z_low, z_high],
                    'g--', alpha=0.2, linewidth=1
                )[0]
                self.workspace_artists.append(line)

        self.fig.canvas.draw_idle()
        print(
            f"Workspace visualization: {'ON' if self.show_workspace else 'OFF'}")

    # --- Control button callbacks ---
    def move_to_pos1(self, event=None):
        """Call visarm's POS_1 movement."""
        try:
            vis.set_survey()
        except Exception as e:
            print(f'POS_1 command failed: {e}')

    def move_to_pos2(self, event=None):
        """Call visarm's POS_2 movement."""
        try:
            vis.set_pos_2()
        except Exception as e:
            print(f'POS_2 command failed: {e}')

    def open_gripper(self, event=None):
        """Open the gripper via visarm."""
        try:
            vis.open_gripper()
            # Mirror local state to avoid querying on every frame
            if hasattr(v, 'GRIPPER_OPEN'):
                self.gripper_angle = float(v.GRIPPER_OPEN)
        except Exception as e:
            print(f'Open gripper failed: {e}')

    def close_gripper(self, event=None):
        """Close the gripper via visarm."""
        try:
            vis.close_gripper()
            # Mirror local state to avoid querying on every frame
            if hasattr(v, 'GRIPPER_CLOSE'):
                self.gripper_angle = float(v.GRIPPER_CLOSE)
        except Exception as e:
            print(f'Close gripper failed: {e}')

    def run(self):
        """Start the interactive visualization."""
        print("\n" + "="*60)
        print("5-DOF Robotic Arm Visualizer")
        print("="*60)
        print("\nControls:")
        print("  - Left click: Set target position")
        print("  - Up/Down arrow: Adjust target Z coordinate")
        print("  - Drag sliders: Manual joint control")
        print("  - 'r' key: Reset to home")
        print("  - 'w' key: Toggle workspace visualization")
        print("  - 'q' key: Quit")
        print("  - Mouse drag: Rotate/pan/zoom view")
        print("\nTip: Rotate the view to select points at different depths!")
        print("="*60 + "\n")

        plt.show()


def main():
    """Main entry point."""
    visualizer = ArmVisualizer()
    visualizer.run()

    vis.disconnect()


if __name__ == "__main__":
    main()
