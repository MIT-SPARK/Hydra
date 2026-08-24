"""Data loader interface."""

import pathlib

import imageio.v3
import yaml

from hydra_python.dataloader import InputPacket
from hydra_python.trajectory import Trajectory


class FileDataLoader:
    """Class for loading files."""

    def __init__(self, datapath):
        """Load poses and prep for running."""
        self._path = pathlib.Path(datapath).expanduser().absolute()
        self._poses = Trajectory.from_csv(
            self._path / "poses.csv",
            pose_cols=["tx", "ty", "tz", "qx", "qy", "qz", "qw"],
        )

        with (self._path / "camera_info.yaml").open("r") as fin:
            self._camera_info = yaml.safe_load(fin.read())

    @property
    def intrinsics(self):
        """Get camera info."""
        return self._camera_info

    def __len__(self):
        """Get underlying trajectory length."""
        return len(self._poses)

    def __iter__(self):
        """Get next input packet."""
        for idx, stamped_pose in enumerate(self._poses):
            timestamp, pose = stamped_pose

            depth_path = self._path / "depth" / f"depth_{idx:07d}.tiff"
            if depth_path.exists():
                depth = imageio.v3.imread(depth_path)
            else:
                depth = None

            label_path = self._path / "labels" / f"labels_{idx:07d}.png"
            if label_path.exists():
                labels = imageio.v3.imread(label_path)
            else:
                labels = None

            color_path = self._path / "color" / f"rgb_{idx:07d}.png"
            if color_path.exists():
                color = imageio.v3.imread(color_path)
            else:
                color = None

            yield InputPacket(
                timestamp=timestamp,
                pose=pose,
                color=color,
                depth=depth,
                labels=labels,
            )
