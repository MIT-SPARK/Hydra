"""Simple logger to store images to disk."""

import pathlib
import imageio.v3
import numpy as np


class DatasetLogger:
    """Save dataset to disk."""

    def __init__(self, output_path):
        """Construct a rosbag."""
        self._output_path = pathlib.Path(output_path).expanduser().absolute()
        self._output_path.mkdir(parents=True, exist_ok=False)
        (self._output_path / "color").mkdir(parents=True, exist_ok=False)
        (self._output_path / "depth").mkdir(parents=True, exist_ok=False)
        (self._output_path / "labels").mkdir(parents=True, exist_ok=False)

        self._pose_file = None
        self._index = 0

    def __enter__(self):
        """Start pose file."""
        self._pose_file = (self._output_path / "poses.csv").open("w")
        self._pose_file.write("timestamp_ns,tx,ty,tz,qw,qx,qy,qz\n")
        self._index = 0
        return self

    def __exit__(self, *args):
        """Stop pose file."""
        self._pose_file.close()

    def _color_suffix(self, index):
        return f"rgb_{index:07d}.png"

    def _depth_suffix(self, index):
        return f"depth_{index:07d}.tiff"

    def _labels_suffix(self, index):
        return f"labels_{index:07d}.png"

    def _image_path(self, camera, index=None):
        index = index if index is not None else self._index
        if camera == "color":
            suffix = self._color_suffix(index)
        elif camera == "depth":
            suffix = self._depth_suffix(index)
        elif camera == "labels":
            suffix = self._labels_suffix(index)
        else:
            raise RuntimeError(f"Invalid camera type {camera}")

        return self._output_path / camera / suffix

    def step(self, timestamp, t, q, depth, labels, rgb):
        """Save output."""
        assert self._pose_file is not None
        self._pose_file.write(
            f"{timestamp},{t[0]},{t[1]},{t[2]},{q[0]},{q[1]},{q[2]},{q[3]}\n"
        )
        imageio.v3.imwrite(self.image_path("color"), rgb)
        imageio.v3.imwrite(self.image_path("depth"), depth)
        imageio.v3.imwrite(self.image_path("labels"), labels.astype(np.uint8))
        self._index += 1
