#!/usr/bin/env python3
import click
import hydra_python as hydra
import matplotlib.pyplot as plt
import numpy as np
from hydra_python.dataloaders.rosbag_dataloader import (
    RosbagDataLoader,
    load_trajectory_from_bag,
)
from ianvs.bag_reader import BagReader


def _repair_args(values, flag):
    return list(zip(len(values) * [flag], values))


@click.command()
@click.argument("bag_path", type=click.Path(exists=True))
@click.option("--max-steps", "-m", default=None, type=int)
@click.option("--config-utilities-files", "-f", multiple=True)
@click.option("--config-utilities-yaml", "-c", multiple=True)
@click.option("--config-utilities-var", "-v", multiple=True)
def main(
    bag_path,
    max_steps,
    config_utilities_files,
    config_utilities_yaml,
    config_utilities_var,
):
    args = _repair_args(config_utilities_files, "-f")
    args += _repair_args(config_utilities_yaml, "-c")
    args += _repair_args(config_utilities_var, "-v")

    hydra.set_glog_level(0, 0)
    hydra.init_config_context(args)

    with BagReader(bag_path) as bag:
        trajectory = load_trajectory_from_bag(
            bag, "hamilton/odom", "hamilton/body", progress=True
        )
        dataloader = RosbagDataLoader(
            bag,
            trajectory,
            "/hamilton/hamilton_zed/rgb/image_rect_color",
            ["/hamilton/hamilton_zed/depth/depth_registered"],
            body_frame="hamilton/body",
        )

        frame_idx = 0
        last_stamp: int | None = None
        threshold_ns = int(0.2 * 1.0e9)
        camera = hydra.make_camera(**dataloader.intrinsics)
        pipeline = hydra.ReconstructionPipeline(camera)
        for stamp, pose, images in dataloader:
            if max_steps and frame_idx >= max_steps:
                break

            if last_stamp is not None and abs(stamp - last_stamp) < threshold_ns:
                continue

            rgb, depth = images
            rgb = rgb[..., :3]
            rgb = rgb[..., ::-1]

            fig, ax = plt.subplots(3)
            ax[0].imshow(rgb)
            ax[1].imshow(depth)
            ax[2].imshow(np.isfinite(depth))
            plt.show()

            q_xyzw = pose.rotation.as_quat()
            q_wxyz = [q_xyzw[i] for i in [3, 0, 1, 2]]
            pipeline.step(stamp, q_wxyz, pose.translation, rgb, depth)
            last_stamp = stamp
            frame_idx += 1


if __name__ == "__main__":
    main()
