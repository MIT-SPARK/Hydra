"""Test trajectory class."""
from hydra_python.trajectory import Trajectory
import numpy as np


def test_trajectory_manual():
    """Test that we can construction a trajectory of the right shape manually."""
    times = np.array([1, 2, 3, 4])
    poses = np.array(
        [
            [1, 2, 3, 1, 0, 0, 0],
            [2, 3, 4, 0, 1, 0, 0],
            [3, 4, 5, 0, 0, 1, 0],
            [4, 5, 6, 0, 0, 0, 1],
        ]
    )
    trajectory = Trajectory(times, poses)
    assert len(trajectory) == 4
