import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, CheckButtons, Button
from matplotlib.backend_bases import Event
from mpl_toolkits.mplot3d.axes3d import Axes3D
from PIL import Image
from pathlib import Path

from typing import Any, Optional, Callable, Union
from typing_extensions import Literal
try:
    from numpy._typing import NDArray
except ImportError:
    from numpy.typing import NDArray

class OpensurgbotViz:
    """3D visualization of the proposed model using matplotlib."""
    DIAL_POSITIONS = [(0.05, 0.55), (0.55, 0.55), (0.05, 0.05), (0.55, 0.05)]
    l1, l2, l3 = 0.015, 0.008, 0.0095 # m
    cyl_radius, cyl_height = 0.001, 0.005 # m
    lim = 0.011 # plot limit (absolute) in m
    theta_1_offset, theta_2_offset, theta_3_offset, theta_4_offset = 0.0, np.radians(-6), np.radians(7), np.radians(-7) # rad, meaning: at angles = +offset, neutral position of the tip

    def __init__(self):
        """3D visualization of the proposed model using matplotlib."""
        self.fig = plt.figure(figsize=(10, 6))
        self.ax: Axes3D = self.fig.add_subplot(111, projection='3d')
        self.do_inverse = True # If False, will do forward kinematics

        self.roll = self.pitch = self.jaw1 = self.jaw2 = 0.
        self.theta_1 = self.theta_2 = self.theta_3 = self.theta_4 = 0.

        self.line = None
        self.joint_cylinders = []
        self.jaw_lines = []
        self.dial_artists = []

        self.dial_img = Image.open(Path(__file__).parent / "assets/dial.png").convert("RGBA")
        self.setup_plot()
        self.update_plot()
        self.create_sliders()
        self.create_dial_images()
        self.create_inverse_checkbox()
        self.create_reset_button()
        self.user_buttons: list[Button] = []

        self.impossible_text = self.ax.text2D(
            0.5, 0.95, "", transform=self.fig.transFigure,
            fontsize=14, color='red', ha='center', va='top',
            bbox=dict(facecolor='white', alpha=0.8, edgecolor='red')
        )

        self.reset()

    def setup_plot(self) -> None: # Create the plots
        self.ax.set_xlim([-self.lim, self.lim])
        self.ax.set_ylim([-self.lim, self.lim])
        self.ax.set_zlim([0, 2*self.lim])
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")

    def create_reset_button(self) -> None: # Create the reset button
        print("Creating reset button")
        reset_ax = plt.axes((0.10, 0.92, 0.08, 0.05), facecolor= 'lightgray')  # Position: [left, bottom, width, height]
        self.reset_button = Button(reset_ax, 'Reset', color='lightcoral', hovercolor='salmon')

        self.reset_button.on_clicked(self.reset)

    def reset(self, event: Optional[Event] = None):
        """Resets the sliders and model. Can be used as a on_change callback from matplotlib.
        
        Args:
            event (Event, optional): unused, but required for on_change callbacks"""
        self.sliders[0].set_val(0) # Roll
        self.sliders[1].set_val(0) # Pitch
        self.sliders[2].set_val(0) # Jaw 1
        self.sliders[3].set_val(0) # Jaw 2
        self.sliders[4].set_val(np.degrees(OpensurgbotViz.theta_1_offset)) # Control 1 (with offset)
        self.sliders[5].set_val(np.degrees(OpensurgbotViz.theta_2_offset)) # Control 2 (with offset)
        self.sliders[6].set_val(np.degrees(OpensurgbotViz.theta_3_offset)) # Control 3 (with offset)
        self.sliders[7].set_val(np.degrees(OpensurgbotViz.theta_4_offset)) # Control 4 (with offset)
        if self.do_inverse:
            self.on_angle_slider_change(None)
        else:
            self.on_control_slider_change(None)
        self.theta_1, self.theta_2, self.theta_3, self.theta_4 = self.sliders[4].val, self.sliders[5].val, self.sliders[6].val, self.sliders[7].val
        self.pitch, self.roll, self.jaw1, self.jaw2 = self.sliders[1].val, self.sliders[0].val, self.sliders[2].val, self.sliders[3].val
        self.update_dials()

    def create_inverse_checkbox(self) -> None: # Make the Inverse Kinematics ? checkbox
        ax_checkbox = plt.axes((0.43, 0.02, 0.20, 0.05), facecolor=None)
        self.checkbox = CheckButtons(ax_checkbox, ['Inverse Kinematics ?'], [self.do_inverse])
        
        def on_checkbox_clicked(label: Union[str, None]):
            self.do_inverse = not self.do_inverse
            print("self.do_inverse set to:", self.do_inverse)
            # Update the plot to reflect current slider values under new mode
            if self.do_inverse:
                self.on_angle_slider_change(None)
            else:
                self.on_control_slider_change(None)

        self.checkbox.on_clicked(on_checkbox_clicked)

    def create_sliders(self) -> None: # Create the sliders
        axcolor = 'lightgoldenrodyellow'
        self.angle_slider_axes = [
            plt.axes((0.2, 0.02 + i*0.03, 0.20, 0.015), facecolor=axcolor)
            for i in range(4)
        ]
        self.sliders = [ # angle sliders
            Slider(self.angle_slider_axes[0], 'Roll $\\theta_r$', -180, 180, valinit=0),
            Slider(self.angle_slider_axes[1], 'Pitch $\\theta_p$', -180, 180, valinit=0),
            Slider(self.angle_slider_axes[2], 'Jaw 1 $\\theta_{j1}$', -180, 180, valinit=0),
            Slider(self.angle_slider_axes[3], 'Jaw 2 $\\theta_{j2}$', -180, 180, valinit=0),
        ]
        axcolor = 'lightgoldenrodyellow'
        self.control_slider_axes = [
            plt.axes((0.7, 0.02 + i*0.03, 0.20, 0.015), facecolor=axcolor)
            for i in range(4)
        ]
        self.sliders += [
            Slider(self.control_slider_axes[0], 'Control $\\theta_1$', -180, 180, valinit=0),
            Slider(self.control_slider_axes[1], 'Control $\\theta_2$', -180, 180, valinit=0),
            Slider(self.control_slider_axes[2], 'Control $\\theta_3$', -180, 180, valinit=0),
            Slider(self.control_slider_axes[3], 'Control $\\theta_4$', -180, 180, valinit=0),
        ]

        marker_values: list[ Union[tuple[float,float], Any] ] = [(-270, 270),
                         (-80, 80),
                         (-100, 100),
                         (-100, 100),
                         (),
                         (-170, 170),
                         (),
                         (),]
        for i, slider in enumerate(self.sliders):
            for value in marker_values[i]:
                slider.ax.axvline(value, color='orange', linestyle='-', linewidth=1.5, label='Center')

        self.sliders[0].on_changed(self.on_angle_slider_change) # Roll
        self.sliders[1].on_changed(self.on_angle_slider_change) # Pitch
        self.sliders[2].on_changed(self.on_angle_slider_change) # Jaw 1
        self.sliders[3].on_changed(self.on_angle_slider_change) # Jaw 2
        self.sliders[4].on_changed(self.on_control_slider_change) # Control 1
        self.sliders[5].on_changed(self.on_control_slider_change) # Control 2
        self.sliders[6].on_changed(lambda val: self.on_control_slider_change(val, "Jaw1")) # Control 3
        self.sliders[7].on_changed(lambda val: self.on_control_slider_change(val, "Jaw2")) # Control 4
        
    def create_dial_images(self) -> None: # Place the dials 
        # Set up a 2D axis for the dials (on top of 3D plot)
        self.ax_dial = self.fig.add_axes((0.75, 0.55, 0.2, 0.4))
        self.ax_dial.axis("off")
        self.ax_dial.set_xlim(0, 1)
        self.ax_dial.set_ylim(0, 1)

        positions = OpensurgbotViz.DIAL_POSITIONS
        labels = ["1", "2", "3", "4"]

        self.dial_artists.clear()
        for i, pos in enumerate(positions):
            artist = self.ax_dial.imshow(np.zeros_like(self.dial_img), extent=(0, 1, 0, 1))
            self.ax_dial.text(pos[0] + 0.2, pos[1] + 0.2 , labels[i],
                            fontsize=12, fontweight='bold', ha='center', va='center',
                            bbox=dict(facecolor='white', alpha=0.7))
            self.dial_artists.append(artist)

    def update_dials(self) -> None: # Update the dials' angles
        angles = [self.sliders[4].val, self.sliders[5].val, self.sliders[6].val, self.sliders[7].val] # Have the offsets
        positions = OpensurgbotViz.DIAL_POSITIONS

        for artist, angle, pos in zip(self.dial_artists, angles, positions):
            rotated = self.dial_img.rotate(angle, resample=Image.Resampling.BICUBIC)
            artist.set_data(rotated)
            artist.set_extent([pos[0], pos[0]+0.4, pos[1], pos[1]+0.4])

    def on_angle_slider_change(self, val: float) -> None: # Callback when a (output) slider has changed
        if self.do_inverse is True:
            # Update the 3D visualisation accordingly
            theta_1_offset, theta_2_offset, theta_3_offset, theta_4_offset = OpensurgbotViz.theta_1_offset, OpensurgbotViz.theta_2_offset, OpensurgbotViz.theta_3_offset, OpensurgbotViz.theta_4_offset
            pitch, roll, jaw1, jaw2 = map(np.radians, [self.sliders[1].val, self.sliders[0].val, self.sliders[2].val, self.sliders[3].val])
            theta_1, theta_2, theta_3, theta_4 = 0.0, 0.0, 0.0, 0.0

            # INVERSE KINEMATICS
            theta_2 = 170/270*roll

            out_vec3 = np.array([pitch, jaw1, jaw2])
            rel_matrix = np.array([
                [-80/80       , 0       , 0     ],
                [-70/80*110/90, 110/90  , 0     ],
                [-70/80*110/90, 0       , 110/90],
            ])
            in_vec3 = np.linalg.solve(rel_matrix, out_vec3)
            theta_1, theta_3, theta_4 = in_vec3[0], in_vec3[1], in_vec3[2]

            # Update
            self.theta_1, self.theta_2, self.theta_3, self.theta_4 = theta_1 + theta_1_offset, theta_2 + theta_2_offset, theta_3 + theta_3_offset, theta_4 + theta_4_offset # Add offsets
            
            self.roll, self.pitch, self.jaw1, self.jaw2 = roll, pitch, jaw1, jaw2
            # Update the sliders
            self.sliders[4].set_val(np.degrees(self.theta_1)) # With offset
            self.sliders[5].set_val(np.degrees(self.theta_2)) # With offset
            self.sliders[6].set_val(np.degrees(self.theta_3)) # With offset
            self.sliders[7].set_val(np.degrees(self.theta_4)) # With offset

            self.update_dials()
            self.update_plot()
        else:
            # Foward kinematics
            pass #do nothing in this case
        self.testfor_impossible_configuration()
        self.fig.canvas.draw_idle()
    
    def on_control_slider_change(self, val: float, origin: Literal["Jaw1","Jaw2", ""] = "") -> None: # Callback when a (input) slider has changed
        if self.do_inverse is True:
            pass #do nothing in this case
        else: # FORWARD KINEMATICS
            # Update values
            theta_1_offset, theta_2_offset, theta_3_offset, theta_4_offset = OpensurgbotViz.theta_1_offset, OpensurgbotViz.theta_2_offset, OpensurgbotViz.theta_3_offset, OpensurgbotViz.theta_4_offset
            self.theta_1, self.theta_2, self.theta_3, self.theta_4 = map(np.radians, [self.sliders[4].val, self.sliders[5].val, self.sliders[6].val, self.sliders[7].val])
            theta_1, theta_2, theta_3, theta_4 = self.theta_1 - theta_1_offset, self.theta_2 - theta_2_offset, self.theta_3 - theta_3_offset, self.theta_4 - theta_4_offset
            pitch, roll, jaw1, jaw2 = 0., 0., 0., 0.

            roll = 270/170*theta_2
            
            rel_matrix = np.array([
                [-80/80       , 0       , 0     ],
                [-70/80*110/90, 110/90  , 0     ],
                [-70/80*110/90, 0       , 110/90],
            ])
            pitch, jaw1, jaw2 = rel_matrix @ np.array([theta_1, theta_3, theta_4])
            
            # Update
            self.roll, self.pitch, self.jaw1, self.jaw2 = roll, pitch, jaw1, jaw2
            # Update the sliders
            self.sliders[0].set_val(np.degrees(self.roll))
            self.sliders[1].set_val(np.degrees(self.pitch))
            self.sliders[2].set_val(np.degrees(self.jaw1))
            self.sliders[3].set_val(np.degrees(self.jaw2))
            
            self.update_dials()
            self.update_plot()

        self.testfor_impossible_configuration()
        self.fig.canvas.draw_idle()
    
    def testfor_impossible_configuration(self) -> bool: # Check for boundary conditions
        # Returns True if configuration allowed, False otherwise
        conditions_output = { # 'message' : bool not in bound
            "$\\theta_p$ not in $[-80, 80]$": np.degrees(self.pitch) < -80 or np.degrees(self.pitch) > 80,
            "$\\theta_r$ not in $[-270, 270]$": np.degrees(self.roll) < -270 or np.degrees(self.roll) > 270,
            "$\\theta_{j1}$ not in $[-110, 110]$": np.degrees(self.jaw1) < -110 or np.degrees(self.jaw1) > 110,
            "$\\theta_{j2}$ not in $[-110, 110]$": np.degrees(self.jaw2) < -110 or np.degrees(self.jaw2) > 110,
            "Should not have $\\theta_{j1} < \\theta_{j2}$": self.jaw2 < self.jaw1,
        }
        delta_boundary = 0
        if self.theta_4 - self.theta_4_offset < np.radians(-90):
            delta_boundary += 80/60*(self.theta_3 - self.theta_3_offset + np.radians(90))
        elif np.radians(90) < self.theta_3 - self.theta_3_offset:
            delta_boundary += 80/60*(self.theta_4 - self.theta_4_offset - np.radians(90))
        msg = f"$\\theta_1$ not in $[{-80 + np.degrees(delta_boundary):.1f}, {80 + np.degrees(delta_boundary):.1f}] (dynamic boundaries)$"
        conditions_input = {
            "Should not have $\\theta_4 < \\theta_3$": self.theta_3 - self.theta_3_offset - (self.theta_4 - self.theta_4_offset) > 0.01, #floats
            msg : self.theta_1 - self.theta_1_offset < -np.radians(80) + delta_boundary or self.theta_1 - self.theta_1_offset > np.radians(80) + delta_boundary,
        }
        true_keys = [key for key, value in conditions_output.items() if value] + [key for key, value in conditions_input.items() if value]
        
        if len(true_keys) > 0:
            self.impossible_text.set_text("⚠️ Impossible configuration :\n" + "\n".join(true_keys))
            return False
        else:
            self.impossible_text.set_text("")
            return True
    
    def rotx(self, theta: float) -> NDArray[np.float64]: # rotx matrix snippet
        return np.array([
            [1, 0, 0],
            [0, np.cos(theta), -np.sin(theta)],
            [0, np.sin(theta),  np.cos(theta)]
        ])

    def roty(self, theta: float) -> NDArray[np.float64]: # roty matrix snippet
        return np.array([
            [np.cos(theta), 0, np.sin(theta)],
            [0, 1, 0],
            [-np.sin(theta), 0, np.cos(theta)]
        ])

    def rotz(self, theta: float) -> NDArray[np.float64]: # rotz matrix snippet
        return np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta),  np.cos(theta), 0],
            [0, 0, 1]
        ])

    def update_plot(self) -> None: # Update the plot
        T0 = np.eye(3)
        P0 = np.array([0, 0, 0])
        T1 = T0 @ self.rotz(self.roll)
        P1 = P0 + T1 @ np.array([0, 0, self.l1])
        T2 = T1 @ self.rotx(self.pitch)
        P2 = P1 + T2 @ np.array([0, 0, self.l2])
        T3_1 = T2 @ self.roty(self.jaw1)
        T3_2 = T2 @ self.roty(self.jaw2)
        P3_1 = P2 + T3_1 @ np.array([0, 0, self.l3])
        P3_2 = P2 + T3_2 @ np.array([0, 0, self.l3])

        # Clear previous lines
        if self.line:
            self.line.remove()
        for line in self.jaw_lines:
            line.remove()
        self.jaw_lines.clear()
        for cyl in self.joint_cylinders:
            cyl.remove()
        self.joint_cylinders.clear()

        self.line, = self.ax.plot(
            [P0[0], P1[0], P2[0]],
            [P0[1], P1[1], P2[1]],
            [P0[2], P1[2], P2[2]],
            marker='o', color='b'
        )
        jaw1_line, = self.ax.plot([P2[0], P3_1[0]], [P2[1], P3_1[1]], [P2[2], P3_1[2]], color='r', marker='o')
        jaw2_line, = self.ax.plot([P2[0], P3_2[0]], [P2[1], P3_2[1]], [P2[2], P3_2[2]], color='g', marker='o')
        self.jaw_lines.extend([jaw1_line, jaw2_line])

        self.joint_cylinders.append(self.draw_cylinder(P0, np.array([0, 0, 1]), color='purple', radius=self.cyl_radius, height=self.cyl_height))
        self.joint_cylinders.append(self.draw_cylinder(P1, T1[:, 0], color='teal', radius=self.cyl_radius, height=self.cyl_height))
        self.joint_cylinders.append(self.draw_cylinder(P2, T2[:, 1], color='gray', radius=self.cyl_radius, height=self.cyl_height))

    def draw_cylinder(self, origin: NDArray[np.float64], direction: NDArray[np.float64], radius: float=0.1, height: float=0.4, resolution: int=16, color: str='gray'): # Draw a cylinder
        direction = direction / np.linalg.norm(direction)
        z = np.linspace(-height / 2, height / 2, 2)
        theta = np.linspace(0, 2 * np.pi, resolution)
        theta_grid, z_grid = np.meshgrid(theta, z)
        x_grid = radius * np.cos(theta_grid)
        y_grid = radius * np.sin(theta_grid)

        xyz = np.stack((x_grid, y_grid, z_grid), axis=-1).reshape(-1, 3).T
        z_axis = np.array([0, 0, 1])
        v = np.cross(z_axis, direction)
        c = np.dot(z_axis, direction)

        if np.linalg.norm(v) < 1e-6:
            R = np.eye(3) if c > 0 else -np.eye(3)
        else:
            s = np.linalg.norm(v)
            vx = np.array([
                [0, -v[2], v[1]],
                [v[2], 0, -v[0]],
                [-v[1], v[0], 0]
            ])
            R = np.eye(3) + vx + vx @ vx * ((1 - c) / (s ** 2))

        rotated_xyz = R @ xyz + origin.reshape(3, 1)
        X = rotated_xyz[0].reshape(x_grid.shape)
        Y = rotated_xyz[1].reshape(y_grid.shape)
        Z = rotated_xyz[2].reshape(z_grid.shape)

        return self.ax.plot_surface(X, Y, Z, color=color, alpha=0.5, linewidth=0)

    def run(self, is_blocking: bool = True) -> None:
        """Display the figure.
        
        Args:
            is_blocking (bool, optional): If False, is non blocking by enabling matplotlib interacting mode, blocking if True. Defaults to True"""
        if not is_blocking:
            plt.ion()
        plt.show()

    def ext_create_user_button(self, name: str, user_button_action: Callable[[Event], Any], color: str = 'peachpuff', hover_color: str = 'chocolate', width: float = 0.08, x: float = 0.20, y: float = 0.92) -> None:
        """Create a user button (external call) if needed
        
        Args:
            name (str): button label
            user_button_action (Callable[[Event], Any): function to call when pressing the button
            color (str, optional): idle color. Defaults to 'peachpuff'
            hover_color (str, optional): color when hovering. Defaults to 'chocolate'
            width (float, optional): width of the button. Defaults to 0.08
            x (float, optional): x position
            y (float, optional): y position"""
        print("Creating user button")
        user_button_ax = plt.axes((x, y, width, 0.05), facecolor= color)  # Position: [left, bottom, width, height]
        new_button = Button(user_button_ax, name, color=color, hovercolor=hover_color)
        new_button.on_clicked(user_button_action)
        self.user_buttons.append(new_button)

if __name__ == "__main__":
    # ==== Example ====
    viz = OpensurgbotViz()
    viz.run(True)