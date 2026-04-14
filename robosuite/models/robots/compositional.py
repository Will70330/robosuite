import numpy as np

from robosuite.models.robots import *


class PandaOmron(Panda):
    @property
    def default_base(self):
        return "OmronMobileBase"

    @property
    def default_arms(self):
        return {"right": "Panda"}

    @property
    def init_qpos(self):
        return np.array([0, np.pi / 16.0 - 0.2, 0.00, -np.pi / 2.0 - np.pi / 3.0, 0.00, np.pi - 0.4, np.pi / 4])

    @property
    def init_torso_qpos(self):
        return np.array([0.2])

    @property
    def base_xpos_offset(self):
        return {
            "bins": (-0.6, -0.1, 0),
            "empty": (-0.6, 0, 0),
            "table": lambda table_length: (-0.16 - table_length / 2, 0, 0),
        }


class SpotWithArm(SpotArm):
    @property
    def default_base(self):
        return "Spot"

    @property
    def default_arms(self):
        return {"right": "SpotArm"}

    @property
    def init_qpos(self):
        return np.array([0.0, -2, 1.26, -0.335, 0.862, 0.0])

    @property
    def base_xpos_offset(self):
        return {
            "bins": (-1.05, -0.1, -0.22),
            "empty": (-1.1, 0, -0.22),
            "table": lambda table_length: (-0.5 - table_length / 2, 0.0, -0.22),
        }


class SpotWithArmFloating(SpotArm):
    def __init__(self, idn=0):
        super().__init__(idn=idn)

    @property
    def init_qpos(self):
        return np.array([0.0, -2, 1.26, -0.335, 0.862, 0.0])

    @property
    def default_base(self):
        return "SpotFloating"

    @property
    def default_arms(self):
        return {"right": "SpotArm"}

    @property
    def base_xpos_offset(self):
        return {
            "bins": (-0.7, -0.1, 0.0),
            "empty": (-0.6, 0, 0.0),
            "table": lambda table_length: (-0.5 - table_length / 2, 0.0, 0.0),
        }


class TidybotKinova(Kinova3):
    """
    TidyBot2 mobile base with Kinova Gen3 7-DOF arm and Robotiq 2F-85 gripper.
    Follows the PandaOmron compositional pattern: inherits arm from Kinova3,
    overrides base to TidybotBase and gripper to Robotiq2F85Gripper.
    """

    @property
    def default_base(self):
        return "TidybotBase"

    @property
    def default_arms(self):
        return {"right": "Kinova3"}

    @property
    def default_gripper(self):
        return {"right": "Robotiq2F85Gripper"}

    @property
    def default_controller_config(self):
        return {"right": "default_tidybotkinova"}

    @property
    def init_qpos(self):
        # Gen3 home configuration from homer tidybot keyframe
        return np.array([0.0, 0.26179939, 3.14159265, -2.26892803,
                         0.0, 0.95993109, 1.57079633])

    @property
    def base_xpos_offset(self):
        return {
            "bins": (-0.5, -0.1, 0),
            "empty": (-0.6, 0, 0),
            "table": lambda table_length: (-0.16 - table_length / 2, 0, 0),
        }



class TidybotYam(YamArm):
    """
    TidyBot2 mobile base with YAM 6-DOF arm and integrated parallel-jaw gripper.
    Follows the PandaOmron compositional pattern: inherits arm from YamArm,
    overrides base to TidybotBase.
    """

    @property
    def default_base(self):
        return "TidybotBase"

    @property
    def default_arms(self):
        return {"right": "YamArm"}

    @property
    def default_controller_config(self):
        return {"right": "default_tidybotyam"}

    @property
    def init_qpos(self):
        # YAM arm home (6 DOF only; inline gripper excluded from arm DOFs)
        return np.array([0.0, 1.047, 1.047, 0.0, 0.0, 0.0])

    @property
    def base_xpos_offset(self):
        return {
            "bins": (-0.5, -0.1, 0),
            "empty": (-0.6, 0, 0),
            "table": lambda table_length: (-0.16 - table_length / 2, 0, 0),
        }


class PandaDexRH(Panda):
    @property
    def default_gripper(self):
        return {"right": "InspireRightHand"}

    @property
    def gripper_mount_pos_offset(self):
        return {"right": [0.0, 0.0, 0.0]}

    @property
    def gripper_mount_quat_offset(self):
        return {"right": [-0.5, 0.5, 0.5, -0.5]}


class PandaDexLH(Panda):
    @property
    def default_gripper(self):
        return {"right": "InspireLeftHand"}

    @property
    def gripper_mount_pos_offset(self):
        return {"right": [0.0, 0.0, 0.0]}

    @property
    def gripper_mount_quat_offset(self):
        return {"right": [0.5, -0.5, 0.5, -0.5]}
