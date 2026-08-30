#!/usr/bin/env python3
import pathlib

import click
import imageio.v3 as iio
import matplotlib.pyplot as plt
import numpy as np
from hydra_python.dataloaders.rosbag_dataloader import (
    RosbagDataLoader,
)
from ianvs.bag_reader import BagReader


def _convert_start_time(bag_start_s: float | None):
    if bag_start_s is None:
        return None

    return int(bag_start_s * 1.0e9)


@click.command()
@click.argument("bag_path", type=click.Path(exists=True))
@click.option("--max-frames", "-m", default=None, type=int)
@click.option("--bag-start-s", default=None, type=float)
@click.option("--name", "-n", default="hamilton")
@click.option("--output", "-o", type=click.Path())
@click.option("--output-threshold", "-t", type=float)
def main(bag_path, max_frames, bag_start_s, name, output, output_threshold):
    bag_path = pathlib.Path(bag_path).expanduser().absolute()

    if output is not None:
        output = pathlib.Path(output).expanduser()
        output.mkdir(parents=True, exist_ok=True)

    with BagReader(bag_path) as bag:
        dataloader = RosbagDataLoader(
            bag,
            f"/{name}/{name}_zed/rgb/image_rect_color",
            other_topics=[f"/{name}/{name}_zed/depth/depth_registered"],
            progress=True,
            start_time_ns=_convert_start_time(bag_start_s),
        )

        finite_percent = []
        for frame_idx, data in enumerate(dataloader):
            if max_frames and frame_idx >= max_frames:
                break

            stamp, pose, images = data
            depth = images[1]
            num_finite = np.sum(np.isfinite(depth))
            total = np.prod(depth.shape)
            ratio = num_finite / total
            finite_percent.append(100 * ratio)

            if output is not None and ratio < output_threshold:
                rgb = images[0][..., ::-1]
                iio.imwrite(output / f"frame_{stamp}.png", rgb)

    plt.plot(finite_percent)
    plt.show()


if __name__ == "__main__":
    main()
