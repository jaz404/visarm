#!/usr/bin/env python3
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
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3D
import sympy as sp
from kinematics import Kinematics


class ArmVisualizer:
    def __init__(self):
        """Initialize the arm visualizer with kinematics and matplotlib setup."""
        # Initialize kinematics
        ll = [2.0, 10.3, 9.6, 4.0, 2.5, 5.0]
        theta_symbols = sp.symbols('θ1 θ2 θ3 θ4 θ5')
        initial_offset = [0, 0, 9.5]
        dh_params = [
            {'a': 0, 'alpha': 90, 'd': ll[0], 'theta': theta_symbols[0]},
            {'a': ll[1], 'alpha': 0, 'd': 0, 'theta': theta_symbols[1]},
            {'a': ll[2], 'alpha': 0, 'd': 0, 'theta': theta_symbols[2]},
            {'a': ll[4], 'alpha': 90, 'd': 0,
                'theta': theta_symbols[3] + sp.pi/2},
            {'a': 0, 'alpha': 0, 'd': ll[3] +
                ll[5], 'theta': theta_symbols[4]},
        ]
        joint_limits = [(-90, 75), (-90, 90), (-90, 80), (-90, 90), (-90, 90)]

        self.kin = Kinematics(
            ll=ll,
            dh_params=dh_params,
            joint_limits=joint_limits,
            variables=theta_symbols,
            initial_offset=initial_offset
        )

        # Current joint angles (degrees)
        self.joint_angles = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

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
        for i in range(5):
            ax_slider = plt.axes(
                [slider_left, 0.75 - i*0.12, slider_width, slider_height])
            lo, hi = self.kin.joint_limits[i]
            slider = Slider(
                ax_slider, f'J{i+1}',
                lo, hi,
                valinit=self.joint_angles[i],
                valstep=1.0
            )
            slider.on_changed(
                lambda val, idx=i: self.on_slider_change(idx, val))
            self.sliders.append(slider)

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
            0.82, 0.25, '',
            fontsize=9,
            family='monospace',
            verticalalignment='top'
        )

    def compute_link_positions(self, joint_angles):
        """Compute all link positions for visualization."""
        positions = []

        # Get individual transforms
        angle_subs = {self.kin.θ[i]: np.radians(
            joint_angles[i]) for i in range(5)}
        transforms = self.kin.compute_dh_matrix(symbolic=True)

        # Accumulate transforms
        T = sp.eye(4)
        for i in range(6):  # base + 5 joints
            T = T @ transforms[i]
            T_num = np.array(T.subs(angle_subs)).astype(np.float64)
            positions.append(T_num[:3, 3])

        return np.array(positions)

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
        self.ee_scatter = self.ax.scatter(
            [ee_pos[0]], [ee_pos[1]], [ee_pos[2]],
            c='red', s=200, marker='*', label='End-Effector'
        )

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

        info = f"Joint Angles (°):\n"
        for i, angle in enumerate(self.joint_angles):
            info += f"  J{i+1}: {angle:6.1f}\n"
        info += f"\nEE Position: {ee_pos_str}\n"
        info += f"Target: {target_str}\n"

        if self.target_point is not None:
            dist = np.linalg.norm(ee_pos - self.target_point)
            info += f"Error: {dist:.2f} cm"

        self.info_text.set_text(info)

        # Update sliders without triggering events
        for i, slider in enumerate(self.sliders):
            slider.eventson = False
            slider.set_val(self.joint_angles[i])
            slider.eventson = True

        # Redraw
        self.ax.legend(loc='upper left')
        self.fig.canvas.draw_idle()

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

        # Create a target transformation matrix
        # For simplicity, we'll keep the default orientation
        # (pointing along +z axis in the end-effector frame)
        T_target = np.eye(4)
        T_target[:3, 3] = self.target_point

        # Default orientation: z-axis pointing up, x-axis pointing forward
        # This is a reasonable default for pick-and-place tasks
        T_target[:3, :3] = np.array([
            [0, 0, 1],
            [0, -1, 0],
            [1, 0, 0]
        ])

        try:
            # Compute IK
            joint_angles_ik = self.kin.inverse_kinematics_analytic(T_target)

            # Animate transition (optional - just jump for now)
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
        """Reset arm to home position."""
        self.joint_angles = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.target_point = None
        if self.target_scatter:
            self.target_scatter.remove()
            self.target_scatter = None
        self.update_arm()
        print("Reset to home position")

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


if __name__ == "__main__":
    main()
