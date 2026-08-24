"""Data loader interface."""

from dataclasses import dataclass, field
from typing import Any, Dict

import numpy as np

from hydra_python.trajectory import Pose


@dataclass
class InputPacket:
    """Input packet for hydra."""

    timestamp: int
    pose: Pose
    color: np.ndarray
    depth: np.ndarray
    labels: np.ndarray
    extras: Dict[str, Any] = field(default_factory=dict)

    @property
    def world_T_body(self):
        """Get homogeneous transform."""
        return self.pose.matrix()
