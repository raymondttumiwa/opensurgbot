
"""
hijacks a SurRoL task (with minimal copied code) to insert the opensurgbot pipeline.
"""

"""
cd "C:\Raymond T\opensurgbot"

$env:PYTHONPATH = "$PWD\SurRoL\Benchmark\state_based;$PWD"

.\.venv_surrol\Scripts\python.exe `
  .\opensurgbot_SurRoL\opensurgbot_surrol_direct_injection.py

"""

import time
import numpy as np

import pybullet as p
from surrol.tasks.needle_pick_org import NeedlePick
from lib_threading import Threader
from opensurgbot_pipeline.lib.opensurgbot_kinevizu.kinevisu.kinevisu import OpensurgbotViz 
from opensurgbot_pipeline.lib.structs_generic import KinematicsManagerABC, UserInterfaceABC, DriverInterfaceABC, InverseKinematicsDescription

from typing import Optional, Tuple

class OpensurgbotNeedlePick(NeedlePick):
    """Hacky way of modifying the NeedlePick task to insert opensurgbot pipeline."""
    counter = 0
    
    def __init__(self, ui: Optional[OpensurgbotViz]=None, pipeline_entry_interface: Optional[KinematicsManagerABC]=None, render_mode=None):
        super().__init__(render_mode=render_mode)
        self._env_setup()
        self.ui = ui
        self. pipeline_entry_interface = pipeline_entry_interface
        if self.ui or self.pipeline_entry_interface:
            self.jaw_center = 0.0 # Current center of jaws
        if self.pipeline_entry_interface:
            self.last_ikd = InverseKinematicsDescription(0,0,0,0,0)  # Store the last inverse kinematics description

    def _env_setup(self):
        super()._env_setup()

        self.psm1.move = self._psm1_move
        self.psm1.move_jaw = self._psm1_move_jaw
    
    def _psm1_move(self, abs_input: np.ndarray, link_index=None) -> Tuple[bool, np.ndarray]:
        # === ORIGINAL CODE ===
        assert abs_input.shape == (4, 4)
        if link_index is None:
            # default link index is the DoF
            link_index = self.psm1.EEF_LINK_INDEX
        pose_world = self.psm1.pose_rcm2world(abs_input, 'tuple')
        # joints_inv = np.array(inverse_kinematics(self.body, self.EEF_LINK_INDEX,
        #                                          pose_world[0], pose_world[1]))
        joints_inv = self.psm1.inverse_kinematics(pose_world, link_index)
        print(f"SurRoL lambda raw={joints_inv[2]}, cm={joints_inv[2] * 10.0}")
        # return self.psm1.move_joint(joints_inv)

        # === CUSTOM CODE ===
        # joints_inv[1] = joints_inv[1] + self.counter * 1e-4 * (-1) ** self.counter  # just for testing, to see the change in joint positions
        # self.counter+=1
        # print(f"abs_input={abs_input}")
        # print(f"joints_inv = {joints_inv}")F
        # print(f"joint_positions = {self.psm1._get_joint_positions_all(joints_inv)}")
        # === NOTE ====
        # joints_inv[0] : arm joint 1 (arm roll)
        # joints_inv[1] : arm joint 2 (paralellogram)
        # joints_inv[2] : translation
        # joints_inv[3] : roll
        # joints_inv[4] : pitch
        # joints_inv[5] : center of jaws
        if self.ui or self.pipeline_entry_interface:
            self.jaw_center = -joints_inv[5]  # center of jaws
        if self.ui:
            self.ui.sliders[0].set_val(np.degrees(joints_inv[3])) # roll
            self.ui.sliders[1].set_val(np.degrees(joints_inv[4])) # pitch
        if self.pipeline_entry_interface:
            jaw_deviation = self.last_ikd.theta_j2 - self.last_ikd.theta_j1
            new_theta_j1 = self.jaw_center - (jaw_deviation / 2)
            new_theta_j2 = self.jaw_center + (jaw_deviation / 2)

            # self.jaw_center = -joints_inv[5]  # center of jaws
            new_ikd = InverseKinematicsDescription(
                joints_inv[3],
                joints_inv[4],
                new_theta_j1,
                new_theta_j2,
                joints_inv[2] * 10.0, # in cm
            )
            self.last_ikd = new_ikd
            self.pipeline_entry_interface.inverse_kinematics(new_ikd)
        return self.psm1.move_joint(joints_inv)

    def _psm1_move_jaw(self, angle_radian: float) -> bool:
        """ Set the jaw tool to angle_radian in radians without actual move. """
        # === ORIGINAL CODE ===
        angle = angle_radian / 2
        self.psm1._jaw_angle = angle
        for joint in (6, 7):
            position = - angle if joint == 6 else angle
            p.setJointMotorControl2(self.psm1.body,
                                    joint,  # jaw joints
                                    p.POSITION_CONTROL,
                                    targetPosition=position,
                                    force=2.)  # TODO: not sure about the force, need tune
        # return True

        # === CUSTOM CODE ===
        print(f"Setting jaw angle to {angle_radian} radians")
        if self.ui:
            self.ui.sliders[2].set_val(np.degrees(self.jaw_center - angle_radian/2)) # jaw1
            self.ui.sliders[3].set_val(np.degrees(self.jaw_center + angle_radian/2)) # jaw2
        if self.pipeline_entry_interface:
            new_ikd = InverseKinematicsDescription(
                self.last_ikd.theta_r,
                self.last_ikd.theta_p,
                self.jaw_center - angle_radian/2,
                self.jaw_center + angle_radian/2,
                self.last_ikd.lambd,
            )
            self.last_ikd = new_ikd
            self.pipeline_entry_interface.inverse_kinematics(new_ikd)

        return True
if __name__ == "__main__":
    DO_VIZ = True # Whether to use the OpensurgbotViz UI
    DO_HARDWARE = True # Whether to use the hardware interface

    ui = None # Optional OpensurgbotViz instance
    manager = None # Optional KinematicsManager instance

    def start_surrol():
        env = OpensurgbotNeedlePick(ui, manager, render_mode='human')  # create one process and corresponding env

        try:
            while True:
                try:
                    print("Starting test...")
                    env.test()
                    print("Test finished !")
                    input("Press Enter to continue...")
                except KeyboardInterrupt:
                    print("KeyboardInterrupt received, stopping the test.")
                    break
        finally:
            if manager is not None:
                for driver in manager.driver_interfaces:
                    close = getattr(driver, "close", None)
                    if close is not None:
                        close()
            env.close()
            time.sleep(2)

    if DO_HARDWARE:
        from opensurgbot_pipeline.lib.opensurgbot_pipeline import KinematicsManager, DriverInterface
        import logging
        logger = logging.Logger("", logging.CRITICAL)
        logger.addHandler(logging.StreamHandler())

        driver1 = DriverInterface(serial_port="COM3",logger=logger)
        manager = KinematicsManager([driver1], logger=logger)

    if DO_VIZ:
        ui = OpensurgbotViz()
        threader = Threader()            
        ui.ext_create_user_button("Start surrol", lambda e: threader.start_threaded(start_surrol))
        ui.run(True)
    if not DO_VIZ:
        start_surrol()



