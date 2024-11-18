"""Data loader interface."""

import abc
import pathlib
from dataclasses import dataclass, field
from typing import Any, Dict

import imageio.v3
import numpy as np
import tqdm
import yaml
from scipy.spatial.transform import Rotation as R

from hydra_python._hydra_bindings import Camera, CameraConfig, ExtrinsicsConfig
from hydra_python.trajectory import Trajectory


@dataclass
class InputPacket:
    """Input packet for hydra."""

    timestamp: int
    world_q_body: np.ndarray
    world_t_body: np.ndarray
    color: np.ndarray
    depth: np.ndarray
    labels: np.ndarray
    extras: Dict[str, Any] = field(default_factory=dict)

    @property
    def world_T_body(self):
        """Get homogeneous transform."""
        q_xyzw = np.roll(self.world_q_body, -1)
        world_T_body = np.eye(4)
        world_T_body[:3, 3] = self.world_t_body
        world_T_body[:3, :3] = R.from_quat(q_xyzw).as_matrix()
        return world_T_body


class DataLoader(abc.ABC):
    """Interface that all data loaders comply to."""

    @property
    @abc.abstractmethod
    def sensor(self):
        """Get current sensor."""
        pass

    @property
    @abc.abstractmethod
    def reset(self):
        """Reset iterator."""
        pass

    @abc.abstractmethod
    def next(self) -> InputPacket:
        """Get next input packet."""
        pass

    @staticmethod
    def run(
        pipeline,
        data,
        show_progress=True,
        max_steps=None,
        data_callbacks=None,
        step_callbacks=None,
    ):
        """Iterate through the dataloader."""
        data_callbacks = [] if data_callbacks is None else data_callbacks
        step_callbacks = [] if step_callbacks is None else step_callbacks

        data_iter = DataLoaderIter(data)
        data_iter = tqdm.tqdm(data_iter) if show_progress else data_iter
        for idx, packet in enumerate(data_iter):
            if max_steps and idx >= max_steps:
                return

            for func in data_callbacks:
                func(packet)

            pipeline.step(
                packet.timestamp,
                packet.world_t_body,
                packet.world_q_body,
                packet.depth,
                packet.labels,
                packet.color,
                **packet.extras,
            )

            for func in step_callbacks:
                func(pipeline.graph)


class DataLoaderIter:
    """Iterator over dataloader."""

    def __init__(self, data):
        """Make a iterator from an underlying dataloader."""
        self._data = data
        self._data.reset()

    def __len__(self):
        """Get the dataloader length if it exists."""
        return len(self._data)

    def __iter__(self):
        """Iterate through the dataloader."""
        value = self._data.next()
        while value is not None:
            yield value
            value = self._data.next()


class FileDataLoader:
    """Class for loading files."""

    def __init__(self, datapath):
        """Load poses and prep for running."""
        self._path = pathlib.Path(datapath).expanduser().absolute()
        self._poses = Trajectory.from_csv(self._path / "poses.csv")
        self._index = 0
        with (self._path / "camera_info.yaml").open("r") as fin:
            self._camera_info = yaml.safe_load(fin.read())

    @property
    def sensor(self, min_range=0.1, max_range=5.0):
        """Get camera info."""
        config = CameraConfig()
        config.min_range = min_range
        config.max_range = max_range
        config.width = self._camera_info["width"]
        config.height = self._camera_info["height"]
        config.fx = self._camera_info["fx"]
        config.fy = self._camera_info["fy"]
        config.cx = self._camera_info["cx"]
        config.cy = self._camera_info["cy"]
        config.extrinsics = ExtrinsicsConfig()
        return Camera(config, "camera")

    def __len__(self):
        """Get underlying trajectory length."""
        return len(self._poses)

    def reset(self):
        """Reset image index to 0."""
        self._index = 0

    def next(self):
        """Get next input packet."""
        if self._index >= len(self._poses):
            return None

        timestamp, world_t_body, world_q_body = self._poses[self._index]
        depth = imageio.v3.imread(self._path / f"depth_{self._index:07d}.tiff")
        labels = imageio.v3.imread(self._path / f"labels_{self._index:07d}.png")
        color = imageio.v3.imread(self._path / f"rgb_{self._index:07d}.png")
        self._index += 1
        return InputPacket(
            timestamp=timestamp,
            world_q_body=world_q_body,
            world_t_body=world_t_body,
            color=color,
            depth=depth,
            labels=labels,
        )


DataLoader.register(FileDataLoader)


# TODO(nathan) register data loaders by extension
def get_dataloader(scene_path, **kwargs):
    """
    Get the dataloader for the specific scene.

    Args:
        scene_path: Path to scene data
        **kwargs: Extra arguments to pass to underlying dataloader
    """
    scene_path = pathlib.Path(scene_path).expanduser().absolute()
    if scene_path.is_dir():
        return FileDataLoader(scene_path)

    if scene_path.suffix == ".glb":
        from hydra_python.simulators import HabitatInterface

        return HabitatInterface(scene_path)

    return None
