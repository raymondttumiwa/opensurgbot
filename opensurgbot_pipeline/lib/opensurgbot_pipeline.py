import time
import numpy as np
import json
import re
from logging import Logger
from matplotlib.backend_bases import Event
from pathlib import Path
from typing import Optional, Union, Tuple
from .structs_generic import UserInterfaceABC, DriverInterfaceABC, KinematicsManagerABC, FowardKinematicsDescription, InverseKinematicsDescription
from .lib_serial import ThreadedSerialHandler
from .opensurgbot_kinevizu.kinevisu.kinevisu import OpensurgbotViz
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

try:
    from typing import override
except ImportError:
    def override(func): # dummy decorator for older python versions
        return func

class KinematicsManager(KinematicsManagerABC):
    """Class to manage kinematics, and transmit commands to driver interfaces"""
    # theta_1_offset, theta_2_offset, theta_3_offset, theta_4_offset = 0.0, np.radians(-6), np.radians(7), np.radians(-7) # rad, meaning: at angles = +offset, neutral position of the tip
    theta_1_offset, theta_2_offset, theta_3_offset, theta_4_offset = 0.0, 0.0, 0.0, 0.0
    @override
    def compute_forward_kinematics(self, forward_kinematics_params: FowardKinematicsDescription) -> Union[InverseKinematicsDescription, None]:
        """Compute forward kinematics. If invalid, returns None.""" 
        print(f"[opensurgbot][kinematics] compute_forward_kinematics input={forward_kinematics_params}", flush=True)
        # TODO: use matrix like for inverse
        theta_1_offset, theta_2_offset, theta_3_offset, theta_4_offset = OpensurgbotViz.theta_1_offset, OpensurgbotViz.theta_2_offset, OpensurgbotViz.theta_3_offset, OpensurgbotViz.theta_4_offset
        theta_1, theta_2, theta_3, theta_4 = forward_kinematics_params.theta_1, forward_kinematics_params.theta_2, forward_kinematics_params.theta_3, forward_kinematics_params.theta_4
        theta_1, theta_2, theta_3, theta_4 = theta_1 - theta_1_offset, theta_2 - theta_2_offset, theta_3 - theta_3_offset, theta_4 - theta_4_offset # Apply offsets

        # FORWARD KINEMATICS
        lambd = forward_kinematics_params.lambd
        pitch, roll, jaw1, jaw2 = 0., 0., 0., 0.

        roll = 270/170*theta_2
        
        rel_matrix = np.array([
            [-80/80       , 0       , 0     ],
            [-70/80*110/90, 110/90  , 0     ],
            [-70/80*110/90, 0       , 110/90],
        ])
        pitch, jaw1, jaw2 = rel_matrix @ np.array([theta_1, theta_3, theta_4])

        inv_params = InverseKinematicsDescription(roll, pitch, jaw1, jaw2, lambd)
        ret = self._condition_check(forward_kinematics_params, inv_params)
        if self.logger:
            self.logger.debug(f"Computed forward kinematics: valid={ret}, fwd={forward_kinematics_params}, inv={inv_params}")
        if ret is False: # Out of bounds
            print("[opensurgbot][kinematics] forward kinematics rejected by bounds check", flush=True)
            return None
        print(f"[opensurgbot][kinematics] compute_forward_kinematics output={inv_params}", flush=True)
        return inv_params
    
    @override
    def compute_inverse_kinematics(self, inverse_kinematics_params: InverseKinematicsDescription) -> Union[FowardKinematicsDescription, None]:
        """Compute inverse kinematics. If invalid, returns None."""
        print(f"[opensurgbot][kinematics] compute_inverse_kinematics input={inverse_kinematics_params}", flush=True)
        roll, pitch, jaw1, jaw2 = inverse_kinematics_params.theta_r, inverse_kinematics_params.theta_p, inverse_kinematics_params.theta_j1, inverse_kinematics_params.theta_j2

        # INVERSE KINEMATICS
        lambd = inverse_kinematics_params.lambd
        theta_2 = 170/270*roll

        out_vec3 = np.array([pitch, jaw1, jaw2])
        rel_matrix = np.array([
            [-80/80       , 0       , 0     ],
            [-70/80*110/90, 110/90  , 0     ],
            [-70/80*110/90, 0       , 110/90],
        ])
        in_vec3 = np.linalg.solve(rel_matrix, out_vec3)
        theta_1, theta_3, theta_4 = in_vec3[0], in_vec3[1], in_vec3[2]

        # Check conditions
        fwd_params = FowardKinematicsDescription(theta_1, theta_2, theta_3, theta_4, lambd)
        ret = self._condition_check(fwd_params, inverse_kinematics_params)
        if self.logger:
            self.logger.debug(f"Computed inverse kinematics: valid={ret}, fwd={fwd_params}, inv={inverse_kinematics_params}")
        if ret is False: # Out of bounds
            print("[opensurgbot][kinematics] inverse kinematics rejected by bounds check", flush=True)
            return None
        print(f"[opensurgbot][kinematics] compute_inverse_kinematics output={fwd_params}", flush=True)
        return fwd_params

    def _condition_check(self, fwd_params: FowardKinematicsDescription, inv_params: InverseKinematicsDescription) -> bool:
        """True if in bounds, False otherwise."""
        roll, pitch, jaw1, jaw2 = inv_params.theta_r, inv_params.theta_p, inv_params.theta_j1, inv_params.theta_j2
        theta_1, _, theta_3, theta_4 = fwd_params.theta_1, fwd_params.theta_2, fwd_params.theta_3, fwd_params.theta_4
        theta_1_offset, theta_2_offset, theta_3_offset, theta_4_offset = self.theta_1_offset, self.theta_2_offset, self.theta_3_offset, self.theta_4_offset

        # Trivial boundary conditions
        if ((np.degrees(pitch) < -80 or np.degrees(pitch) > 80) or
            (np.degrees(roll) < -270 or np.degrees(roll)) > 270 or
            (np.degrees(jaw1) < -110 or np.degrees(jaw1)) > 110 or
            (np.degrees(jaw2) < -110 or np.degrees(jaw2)) > 110
            ):
            return False

        # Variable boundary conditions
        delta_boundary = 0
        if theta_4 - theta_4_offset < np.radians(-90):
            delta_boundary += 80/60*(theta_3 - theta_3_offset + np.radians(90))
        elif np.radians(90) < theta_3 - theta_3_offset:
            delta_boundary += 80/60*(theta_4 - theta_4_offset - np.radians(90))
        if ((theta_3 - theta_3_offset - (theta_4 - theta_4_offset) > 0.01) or
            (theta_1 - theta_1_offset < -np.radians(80) + delta_boundary or theta_1 - theta_1_offset > np.radians(80) + delta_boundary)
         ):
            return False

        return True
    
class UserInterface3DViz(UserInterfaceABC):
    """User interface with 3D visualization."""

    def __init__(self, kinematics_manager: KinematicsManagerABC, logger: Union[Optional[Logger], None] = None) -> None:
        super().__init__(kinematics_manager, logger)
        self.viz = OpensurgbotViz()

        # === Add custom button ===
        self.viz.ext_create_user_button("Reconnect", self._on_reconnect_click, width=0.15, x=0.8)
        self.viz.ext_create_user_button("Send command (as inverse!)", self._on_send_click, width=0.21)
        self.viz.ext_create_user_button("Random position", self._on_random_click, width=0.18, x=0.43)

        # === Add CAN slider ===
        can_slider_ax = plt.axes((0.1, 0.3, 0.01, 0.3), facecolor="blue")  # Position: [left, bottom, width, height]
        self.can_slider = Slider(can_slider_ax, '$\\lambda$ (cm)', 0, 13, valinit=0, orientation="vertical")
        self.viz.reset_button.on_clicked(self._reset_can_slider)

    def _reset_can_slider(self, event: Event) -> None:
        self.can_slider.set_val(0)

    def _on_reconnect_click(self, event: Event) -> None: # What to do on user button click
        print("[opensurgbot][ui] reconnect clicked", flush=True)
        for interface in self.kinematics_manager.driver_interfaces:
            if isinstance(interface, DriverInterface): # Which has serial connection
                print("[opensurgbot][ui] restarting serial thread", flush=True)
                interface.serial.stop()
                time.sleep(1)
                interface.serial.start_threaded()

    def _on_send_click(self, event: Event) -> None: # What to do on user button click
        print("[opensurgbot][ui] send clicked", flush=True)
        if self.logger:
            self.logger.debug("User button Send click detected")

        # Retrieve **inverse** parameters 
        inv_params = InverseKinematicsDescription(
            theta_r=self.viz.roll,
            theta_p=self.viz.pitch,
            theta_j1=self.viz.jaw1,
            theta_j2=self.viz.jaw2,
            lambd=self.can_slider.val
        )
        print(f"[opensurgbot][ui] inverse params={inv_params}", flush=True)
        if not self.kinematics_manager.inverse_kinematics(inv_params):
            print("[opensurgbot][ui] inverse kinematics failed", flush=True)
            if self.logger:
                self.logger.info(f"WARNING: invalid kinematic parameters inv_params={inv_params}")
    
    def _on_random_click(self, event: Event) -> None: # What to do on user button click
        print("[opensurgbot][ui] random clicked", flush=True)
        if self.logger:
            self.logger.debug("User button Random click detected")
        while True:
            sample = np.random.random(4)
            theta_1 = np.radians((sample[0] - 0.5)*2 * (78.5 - 5)) # 5° border margin
            theta_2 = np.radians((sample[1] - 0.5)*2 * (170. - 5)) # 5° border margin
            theta_3 = np.radians((sample[2] - 0.5)*2 * (150. - 5)) # 5° border margin
            theta_4 = np.radians((sample[3] - 0.5)*2 * (150. - 5)) # 5° border margin
            lambd = (0.1 + np.random.random()*13) # 0-13 cm  -- 1mm safe zone
            fwd_params = FowardKinematicsDescription(theta_1, theta_2, theta_3, theta_4, lambd)
            rev_params = self.kinematics_manager.compute_forward_kinematics(fwd_params)
            if rev_params is not None:
                print(f"[opensurgbot][ui] random valid forward params={fwd_params}", flush=True)
                self.viz.sliders[0].set_val(np.degrees(rev_params.theta_r))
                self.viz.sliders[1].set_val(np.degrees(rev_params.theta_p))
                self.viz.sliders[2].set_val(np.degrees(rev_params.theta_j1))
                self.viz.sliders[3].set_val(np.degrees(rev_params.theta_j2))
                self.viz.sliders[4].set_val(np.degrees(fwd_params.theta_1))
                self.viz.sliders[5].set_val(np.degrees(fwd_params.theta_2))
                self.viz.sliders[6].set_val(np.degrees(fwd_params.theta_3))
                self.viz.sliders[7].set_val(np.degrees(fwd_params.theta_4))
                self.can_slider.set_val(lambd)
                self.viz.on_angle_slider_change(0)
                self.viz.on_control_slider_change(0)
                break


    def run(self, is_blocking: bool = True) -> None:
        """Display the figure.
        
        Args:
            is_blocking (bool, optional): If False, is non blocking by enabling matplotlib interacting mode, blocking if True. Defaults to True"""
        if self.logger:
            self.logger.debug(f"Running UserInterface3DViz with is_blocking={is_blocking}")
        print(f"[opensurgbot][ui] run is_blocking={is_blocking}", flush=True)
        self.viz.run(is_blocking)

class DriverInterface(DriverInterfaceABC):
    serial_baudrate = 115200
    servo_max_speed = 100.0 # deg per s
    ZDT_max_speed = 2 # cm per s # Max speed ~6cm /s
    ZDT_HOME_LAMBDA_CM = 0.0
    ZDT_MAX_TRAVEL_CM = 13.0
    ZDT_ZERO_GUARD_CM = 0.1 # 1 mm no-drive zone near software zero; return_home is allowed.
    ZDT_STATE_DIR = Path(__file__).resolve().parents[2] / ".opensurgbot_state"

    def __init__(self, serial_port: Optional[str] = None, serial_timeout: float = 0.1, serial_wait_time: float = 0.1, logger: Optional[Logger] = None) -> None:
        super().__init__(logger)
        print(f"[opensurgbot][driver] init serial_port={serial_port} baudrate={DriverInterface.serial_baudrate}", flush=True)
        self.serial_port = serial_port or "default"
        self._zdt_state_path = self._get_zdt_state_path(self.serial_port)
        self.serial = ThreadedSerialHandler(port=serial_port,baudrate=DriverInterface.serial_baudrate, timeout=serial_timeout, wait_time=serial_wait_time, logger=logger)
        self.zdt_current_steps = self._load_zdt_current_steps()
        if self.zdt_current_steps != 0:
            print(f"[opensurgbot][driver] saved ZDT position={self.zdt_current_steps}; returning home before startup", flush=True)
            self.return_home()
        else:
            self.set_current_position_zero()

    def _get_zdt_state_path(self, serial_port: str) -> Path:
        safe_port = re.sub(r"[^A-Za-z0-9_.-]+", "_", serial_port)
        return DriverInterface.ZDT_STATE_DIR / f"zdt_{safe_port}.json"

    def _load_zdt_current_steps(self) -> int:
        try:
            with self._zdt_state_path.open("r", encoding="utf-8") as state_file:
                state = json.load(state_file)
            return int(state.get("zdt_current_steps", 0))
        except (FileNotFoundError, OSError, ValueError, TypeError, json.JSONDecodeError):
            return 0

    def _save_zdt_current_steps(self) -> None:
        try:
            self._zdt_state_path.parent.mkdir(parents=True, exist_ok=True)
            state = {
                "serial_port": self.serial_port,
                "zdt_current_steps": self.zdt_current_steps,
                "updated_at": time.time(),
            }
            with self._zdt_state_path.open("w", encoding="utf-8") as state_file:
                json.dump(state, state_file, indent=2)
        except OSError as e:
            print(f"[opensurgbot][driver] failed to save ZDT state: {e}", flush=True)
            if self.logger:
                self.logger.warning(f"Failed to save ZDT state: {e}")

    def _wait_for_status(self, label: str, success_prefix: str, timeout: float, cmd_begin_time: Optional[float] = None) -> bool:
        if cmd_begin_time is None:
            cmd_begin_time = time.time()

        while time.time() - cmd_begin_time < timeout:
            recv = self.serial.receive_line(timeout=timeout)
            if recv is None:
                print(f"[opensurgbot][driver] waiting for {label}...", flush=True)
                continue
            if self.logger:
                self.logger.debug(f"Received serial line: {recv}")
            if recv.startswith(success_prefix):
                print(f"[opensurgbot][driver] {label} received: {recv}", flush=True)
                return True
            if recv.startswith("248"):
                print(f"[opensurgbot][driver] {label} error received: {recv}", flush=True)
                if self.logger:
                    self.logger.warning(f"Command failed while waiting for {label}: {recv}")
                return False

        print(f"[opensurgbot][driver] {label} timeout", flush=True)
        return False

    def _zdt_target_is_near_zero(self, lambd: float) -> bool:
        return lambd <= DriverInterface.ZDT_HOME_LAMBDA_CM + DriverInterface.ZDT_ZERO_GUARD_CM

    def set_current_position_zero(self, timeout: float = 5.0) -> None:
        """Set Arduino ZDT software zero at the current physical position. Uses command id 19."""
        print("[opensurgbot][driver] setting startup ZDT software zero", flush=True)
        if not self.serial.wait_until_open(timeout=timeout):
            print("[opensurgbot][driver] serial not open; startup zero command skipped", flush=True)
            if self.logger:
                self.logger.warning("Serial not open; startup zero command skipped")
            return

        msg = "19"
        self.serial.queue_message(msg)
        if self.logger:
            self.logger.debug(f"Queued serial startup-zero message msg={msg}")

        cmd_begin_time = time.time()
        if not self._wait_for_status("startup-zero ack", "250", timeout, cmd_begin_time):
            return

        if self._wait_for_status("startup-zero completion", "249", timeout, cmd_begin_time):
            self.zdt_current_steps = 0
            self._save_zdt_current_steps()
    
    @override
    def drive(self, forward_kinematics_params: FowardKinematicsDescription, timeout: float = 10.0) -> None:
        """Send drive control and wait for the movement to finish. Will use: command id 101 - opensurgbot drive all blocking"""
        print(f"[opensurgbot][driver] drive input={forward_kinematics_params} timeout={timeout}", flush=True)
        # ==== Building the message ====
        msg = '101'
        # === servos ===
        th1, th2, th3, th4 = forward_kinematics_params.theta_1, forward_kinematics_params.theta_2, forward_kinematics_params.theta_3, forward_kinematics_params.theta_4
        msg += f",{-np.degrees(th1):.2f}" # Invert sign: negative of dial angle convention
        msg += f",{-np.degrees(th2):.2f}" # Invert sign: negative of dial angle convention
        msg += f",{-np.degrees(th3):.2f}" # Invert sign: negative of dial angle convention
        msg += f",{-np.degrees(th4):.2f}" # Invert sign: negative of dial angle convention
        msg += f",{self.servo_max_speed:.2f}"
        
        # === ZDT Linear actuator ===
        lambd = forward_kinematics_params.lambd
        if self._zdt_target_is_near_zero(lambd):
            print("[opensurgbot][driver] ZDT target blocked: within 1 mm of software zero; use return_home() on close", flush=True)
            if self.logger:
                self.logger.warning("ZDT target blocked because it is within 1 mm of software zero")
            return
        steps, speed = self._get_zdt_args(lambd, self.ZDT_max_speed)
        msg += f",1,{steps},{speed},0,{round(timeout*1000)}"

        # ==== Sending the message ====
        print(f"[opensurgbot][driver] queueing serial msg={msg}", flush=True)
        self.serial.queue_message(msg)
        if self.logger:
            self.logger.debug(f"Queued serial message msg={msg}")
        
        # ==== Waiting for acknowledgment ====
        cmd_begin_time = time.time()
        if not self._wait_for_status("ack", "250", timeout, cmd_begin_time):
            if self.logger:
                self.logger.warning(f"Did not receive message acknowledgment in {timeout} seconds")
            return
        
        if not self._wait_for_status("completion", "249", timeout, cmd_begin_time):
            if self.logger:
                self.logger.warning(f"Did not receive command success message in {timeout} seconds")
            return
        self.zdt_current_steps = steps
        self._save_zdt_current_steps()

    def home(self, timeout: float = 15.0) -> None:
        """Trigger ZDT homing through the firmware. Uses command id 18."""
        print(f"[opensurgbot][driver] home timeout={timeout}", flush=True)
        msg = "18"
        self.serial.queue_message(msg)
        if self.logger:
            self.logger.debug(f"Queued serial homing message msg={msg}")

        cmd_begin_time = time.time()
        if not self._wait_for_status("homing ack", "250", timeout, cmd_begin_time):
            if self.logger:
                self.logger.warning(f"Did not receive homing acknowledgment in {timeout} seconds")
            return

        if not self._wait_for_status("homing completion", "249", timeout, cmd_begin_time):
            if self.logger:
                self.logger.warning(f"Did not receive homing success message in {timeout} seconds")
            return
        self.zdt_current_steps = 0
        self._save_zdt_current_steps()

    def return_home(self, timeout: float = 60.0) -> None:
        """Return the ZDT axis to the Arduino startup software-home position. Uses command id 199."""
        print(f"[opensurgbot][driver] return_home timeout={timeout}", flush=True)
        msg = f"199,1200,0,{round(timeout*1000)},{self.zdt_current_steps}"
        self.serial.queue_message(msg)
        if self.logger:
            self.logger.debug(f"Queued serial return-home message msg={msg}")

        cmd_begin_time = time.time()
        if not self._wait_for_status("return-home ack", "250", timeout, cmd_begin_time):
            return

        if not self._wait_for_status("return-home completion", "249", timeout, cmd_begin_time):
            if self.logger:
                self.logger.warning(f"Did not receive return-home success message in {timeout} seconds")
            return
        self.zdt_current_steps = 0
        self._save_zdt_current_steps()

    def close(self, return_home: bool = True, timeout: float = 60.0) -> None:
        """Safely close the hardware driver, returning the ZDT axis to startup zero first."""
        try:
            if return_home:
                self.return_home(timeout=timeout)
        finally:
            self.serial.stop()

    def _get_zdt_args(self, alg_dist: float, max_speed: float) -> Tuple[int,int]:
        """Convert parameters
         Args:
            alg_idst (float): in cm
            max_speed (float): in cm/s"""
        ZDTStepPerTurn = 0.1 # cm per full motor revolution, same as ARDUINO_COMBINED_CHECK
        ZDTStepRot = 1.8 # degrees per full motor step
        ZDTMicrostep = 16 # microsteps per full motor step
        normdist = alg_dist - DriverInterface.ZDT_HOME_LAMBDA_CM
        normdist = min(max(normdist, 0.0), DriverInterface.ZDT_MAX_TRAVEL_CM)

        n_step = round(normdist * (360/ZDTStepPerTurn) * (1/(ZDTStepRot/ZDTMicrostep)))
        speed_rpm = round(max_speed * 60 / ZDTStepPerTurn)
        print(f"[opensurgbot][driver] zdt_args alg_dist={alg_dist} -> steps={n_step}, speed={speed_rpm}", flush=True)

        return (n_step, speed_rpm)
