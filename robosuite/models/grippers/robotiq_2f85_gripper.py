"""
Robotiq 2F-85 gripper with 4-bar linkage mechanism.

This is a more physically accurate model of the Robotiq 2-Finger 85mm gripper
compared to Robotiq85Gripper. It uses a 4-bar linkage per finger (driver,
coupler, spring_link, follower) with tendon-based actuation, matching the
model used on the TidyBot2 platform.
"""
import numpy as np

from robosuite.models.grippers.gripper_model import GripperModel
from robosuite.utils.mjcf_utils import xml_path_completion


class Robotiq2F85GripperBase(GripperModel):
    """
    Robotiq 2F-85 gripper with 4-bar linkage.

    Args:
        idn (int or str): Number or some other unique identification string for this gripper instance
    """

    def __init__(self, idn=0):
        super().__init__(xml_path_completion("grippers/robotiq_2f85_gripper.xml"), idn=idn)

    def format_action(self, action):
        return action

    @property
    def init_qpos(self):
        # 8 joints: right_driver, right_coupler, right_spring_link, right_follower,
        #           left_driver, left_coupler, left_spring_link, left_follower
        return np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    @property
    def _important_geoms(self):
        return {
            "left_finger": [
                "left_driver_collision",
                "left_spring_link_collision",
                "left_follower_collision",
                "left_pad1",
                "left_pad2",
            ],
            "right_finger": [
                "right_driver_collision",
                "right_spring_link_collision",
                "right_follower_collision",
                "right_pad1",
                "right_pad2",
            ],
            "left_fingerpad": ["left_pad1", "left_pad2"],
            "right_fingerpad": ["right_pad1", "right_pad2"],
        }


class Robotiq2F85Gripper(Robotiq2F85GripperBase):
    """
    1-DOF variant of Robotiq2F85GripperBase.
    Maps a single [-1, 1] action to gripper open/close via tendon control.
    """

    def format_action(self, action):
        assert len(action) == 1
        self.current_action = np.clip(
            self.current_action * np.array([1.0]) + self.speed * np.sign(action), -1.0, 1.0
        )
        return self.current_action

    @property
    def speed(self):
        return 0.20

    @property
    def dof(self):
        return 1
