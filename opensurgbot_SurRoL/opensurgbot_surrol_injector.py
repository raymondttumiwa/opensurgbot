
"""
Wrap a SurRoL task to insert the opensurgbot pipeline
"""

import gym
import time
import logging
from abc import ABC, abstractmethod
from typing import Optional, Dict, Tuple
import numpy as np
import pybullet as p
from typing_extensions import override

from surrol.tasks.psm_env import PsmEnv
from surrol.robots.psm import Psm
from opensurgbot_pipeline.lib.opensurgbot_pipeline import InverseKinematicsDescription
from opensurgbot_pipeline.lib.opensurgbot_pipeline import KinematicsManager, DriverInterface

class PipelineGeneric(ABC):
    """Abstract base class for a generic pipeline."""
    @abstractmethod
    def move_action(self, fkd: InverseKinematicsDescription) -> bool:
        """Move the robot based on the provided forward kinematics description.
        
        Args:
            fkd (FowardKinematicsDescription): The forward kinematics for the movement action.

        Returns (end of action):
            bool: True if the action was successful, False otherwise.
        """
        raise NotImplementedError("This method should be overridden by subclasses.")

    def close(self) -> None:
        """Close the pipeline and release any hardware resources."""
        pass
    
class PipelinePrinter(PipelineGeneric):
    """Dummy test pipeline, will print a message when arm.move is called."""
    @override
    def move_action(self, fkd: InverseKinematicsDescription) -> bool:
        print(f"Injection successful ! Moving arm to position: {fkd}")
        return True

class PipelineOpensurgbot(PipelineGeneric):
    """Pipeline for Opensurgbot hardware pipeline."""
    def __init__(self, 
                 serial_port: str,
                 drive_timeout: float = 10.0,
                 serial_timeout: float = 0.1,
                 serial_wait_time: float = 0.1,
                 logger: Optional[logging.Logger] = None):
        self.logger = logger
        driver = DriverInterface(serial_port, serial_timeout, serial_wait_time, logger=logger)
        self.manager = KinematicsManager([driver], drive_timeout, logger=logger)

    @override
    def move_action(self, ikd: InverseKinematicsDescription) -> bool:
        """Move the robot based on the provided inverse kinematics description."""
        if self.logger:
            self.logger.info(f"Moving to position: {ikd}")

        return self.manager.inverse_kinematics(ikd)

    def close(self) -> None:
        """Return hardware to startup zero and stop serial communication."""
        for driver in self.manager.driver_interfaces:
            close = getattr(driver, "close", None)
            if close is not None:
                close()


class OpensurgbotSurRoLInjector(gym.Env):
    """Hijacks the given SurRoL task (**using Psm arms**) to insert the Opensurgbot pipeline. Allows using opensurgbot with any SurRoL task that uses psm arms."""

    def __init__(self, pipelines_to_insert: Dict[str, PipelineGeneric]):
        """Hijacks the given SurRoL task (**using Psm arms**) to insert the Opensurgbot pipeline.

        Args:
            pipelines_to_insert (Dict[str, PipelineGeneric]): A dictionary mapping PSM names to their respective pipelines.

        Example pipelines_to_insert: {'psm1': PipelineOpensurgbot(...), 'psm2': PipelineOpensurgbot(...)}
            will override the move and move_jaw methods of self.psm1 and self.psm2 respectively (where self is the SurRoL task).
        """
        self.last_ikds: dict[str, InverseKinematicsDescription] = {}
        self.pipelines_to_insert = pipelines_to_insert

    def inject(self, surrol_task: PsmEnv):
        """Inject the pipeline into the SurRoL task."""

        original_close = surrol_task.close

        def override_reset(selfe: PsmEnv):
            """Override the reset method of the SurRoL task to insert the pipelines (note that reset is also called during first init)."""
            super_result = surrol_task.__class__.reset(selfe) # call original reset method

            # ==== Insert pipelines upon reset ====
            for psm_name, pipeline in self.pipelines_to_insert.items():
                if not isinstance(pipeline, PipelineGeneric):
                    raise TypeError(f"Pipeline for {psm_name} must be an instance of PipelineGeneric")
                
                if psm_name not in surrol_task.__dict__:
                    raise KeyError(f"PSM name {psm_name} not found in SurRoL task's attributes !") 
                
                self.last_ikds.clear() # clear last ikds dict
                self.last_ikds[psm_name] = InverseKinematicsDescription(0, 0, 0, 0, 0) # Initialize last ikd for each psm

                # ==================================================================
                #  Define psm.move override with inserted pipeline
                # ==================================================================
                def override_move(selfp: Psm, abs_input: np.ndarray, link_index=None, uniqueness_key: str=psm_name) -> Tuple[bool, np.ndarray]:
                    """Override the move method of the PSM to use the pipeline. Note: uniqueness_key allows python storing different methods for all pipelines (unique function signature)."""
                    # === ORIGINAL CODE ===
                    assert abs_input.shape == (4, 4)
                    if link_index is None:
                        # default link index is the DoF
                        link_index = selfp.EEF_LINK_INDEX
                    pose_world = selfp.pose_rcm2world(abs_input, 'tuple')
                    # joints_inv = np.array(inverse_kinematics(self.body, self.EEF_LINK_INDEX,
                    #                                          pose_world[0], pose_world[1]))
                    joints_inv = selfp.inverse_kinematics(pose_world, link_index)
                    # return self.psm1.move_jaw(joints_inv)

                    # === CUSTOM CODE ===
                    # joints_inv[1] = joints_inv[1] + self.counter * 1e-4 * (-1) ** self.counter  # just for testing, to see the change in joint positions
                    # self.counter+=1
                    # print(f"abs_input={abs_input}")
                    # print(f"joints_inv = {joints_inv}")
                    # print(f"joint_positions = {self.psm1._get_joint_positions_all(joints_inv)}")
                    # === NOTE ====
                    # joints_inv[0] : arm joint 1 (arm roll)
                    # joints_inv[1] : arm joint 2 (paralellogram)
                    # joints_inv[2] : translation
                    # joints_inv[3] : roll
                    # joints_inv[4] : pitch
                    # joints_inv[5] : center of jaws
                    jaw_center = joints_inv[5]  # center of jaws
                    jaw_open_angle = self.last_ikds[psm_name].theta_j2 - self.last_ikds[psm_name].theta_j1
                    new_theta_j1 = jaw_center - (jaw_open_angle / 2)
                    new_theta_j2 = jaw_center + (jaw_open_angle / 2)

                    new_ikd = InverseKinematicsDescription(
                        joints_inv[3],
                        -joints_inv[4], # Sign correction convention different from kinevizu convention
                        new_theta_j1,
                        new_theta_j2,
                        joints_inv[2] / 100.0, # in cm
                    )
                    self.last_ikds[psm_name] = new_ikd
                    
                    pipeline.move_action(new_ikd)
                    return selfp.move_joint(joints_inv)
                
                # ==================================================================
                #  Define psm.move_jaw override with inserted pipeline
                # ==================================================================
                def override_move_jaw(selfp: Psm, angle_radian: float) -> bool:
                    """Override the move_jaw method of the PSM to use the pipeline."""
                    # === ORIGINAL CODE ===
                    angle = angle_radian / 2
                    selfp._jaw_angle = angle
                    for joint in (6, 7):
                        position = - angle if joint == 6 else angle
                        p.setJointMotorControl2(selfp.body,
                                                joint,  # jaw joints
                                                p.POSITION_CONTROL,
                                                targetPosition=position,
                                                force=2.)  # TODO: not sure about the force, need tune
                    # return True

                    # === CUSTOM CODE ===
                    print(f"Setting jaw angle to {angle_radian} radians")
                    angle_radian = 0.70 - angle_radian
                    last_ikd = self.last_ikds[psm_name]
                    jaw_center = (last_ikd.theta_j1 + last_ikd.theta_j2) / 2
                    new_ikd = InverseKinematicsDescription(
                        last_ikd.theta_r,
                        last_ikd.theta_p,
                        jaw_center - angle_radian/2,
                        jaw_center + angle_radian/2,
                        last_ikd.lambd,
                    )
                    self.last_ikds[psm_name] = new_ikd
                    pipeline.move_action(new_ikd)

                    return True
                
                # === Override the methods ===
                surrol_task.__dict__[psm_name].move = override_move.__get__(surrol_task.__dict__[psm_name], Psm)
                surrol_task.__dict__[psm_name].move_jaw = override_move_jaw.__get__(surrol_task.__dict__[psm_name], Psm)
            return super_result

        def override_close(self: PsmEnv):
            """Return inserted hardware pipelines home before closing the SurRoL task."""
            try:
                for pipeline in self.pipelines_to_insert.values():
                    pipeline.close()
            finally:
                return original_close()
        
        surrol_task.reset = override_reset.__get__(surrol_task, PsmEnv)
        surrol_task.close = override_close.__get__(surrol_task, PsmEnv)

if __name__ == '__main__':
    from surrol.tasks.needle_pick import NeedlePick
    from surrol.tasks.needle_regrasp_bimanual import NeedleRegrasp
    surrol_task = NeedlePick(render_mode='human')
    logger = logging.Logger("", logging.DEBUG)
    logger.addHandler(logging.StreamHandler()) 

    pipelines: Dict[str, PipelineGeneric] = {} # Pipelines to insert
    pipelines['psm1'] = PipelineOpensurgbot(serial_port="COM3", logger=logger)  # Override self.psm1's move and move_jaw methods
    # pipelines["psm1"] = PipelinePrinter()

    injector = OpensurgbotSurRoLInjector(pipelines)
    injector.inject(surrol_task)

    try:
        surrol_task.test()
        print("Test finished !")
        input("Press Enter to continue...")
    finally:
        surrol_task.close()
        time.sleep(2)

    #To run:  $env:PYTHONPATH="C:\Raymond T\opensurgbot_fresh\SurRoL\Benchmark\state_based;C:\Raymond T\opensurgbot_fresh" .venv_surrol\Scripts\python.exe .\opensurgbot_SurRoL\opensurgbot_surrol_direct_injection.py
