"""
TidyBot2 holonomic wheeled mobile base.
"""
import numpy as np

from robosuite.models.bases.mobile_base_model import MobileBaseModel
from robosuite.utils.mjcf_utils import xml_path_completion


class TidybotBase(MobileBaseModel):
    """
    TidyBot2 mobile base with holonomic drive (forward, side, yaw).

    Includes camera mount geometry and two base cameras matching the
    physical TidyBot2 platform camera positions.

    Args:
        idn (int or str): Number or some other unique identification string for this base instance
    """

    def __init__(self, idn=0):
        super().__init__(xml_path_completion("bases/tidybot_base.xml"), idn=idn)

    @property
    def top_offset(self):
        return np.array((0, 0, 0))

    @property
    def horizontal_radius(self):
        return 0.30
