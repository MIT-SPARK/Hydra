#!/usr/bin/env python3
import pathlib

import click
import hydra_python as hydra
import ianvs
from hydra_python.dataloaders.rosbag_dataloader import (
    RosbagDataLoader,
    load_trajectory_from_bag,
)
from hydra_python.trajectory import Trajectory
from ianvs.bag_reader import BagReader


def _repair_args(values, flag):
    all_flags = []
    for value in values:
        all_flags.append(flag)
        all_flags.append(value)

    return all_flags


def _convert_start_time(bag_start_s: float | None):
    if bag_start_s is None:
        return None

    return int(bag_start_s * 1.0e9)


@click.command()
@click.argument("bag_path", type=click.Path(exists=True))
@click.option("--trajectory-path", "-t", type=click.Path(exists=True))
@click.option("--max-steps", "-m", default=None, type=int)
@click.option("--min-separation-s", "-s", default=0.0, type=float)
@click.option("--bag-start-s", default=None, type=float)
@click.option("--config-utilities-files", "-f", multiple=True)
@click.option("--config-utilities-yaml", "-c", multiple=True)
@click.option("--config-utilities-var", "-v", multiple=True)
@click.option("--name", "-n", default="hamilton")
def run(
    bag_path,
    trajectory_path,
    max_steps,
    min_separation_s,
    bag_start_s,
    config_utilities_files,
    config_utilities_yaml,
    config_utilities_var,
    name,
):
    bag_path = pathlib.Path(bag_path).expanduser().absolute()
    args = _repair_args(config_utilities_files, "-f")
    args += _repair_args(config_utilities_yaml, "-c")
    args += _repair_args(config_utilities_var, "-v")
    args += ["-c", "{app_plugins: [{type: ConfigServerPlugin}, {type: SpinPlugin}]}"]
    args += ["-c", "{verbosity: 1}"]

    hydra.set_glog_level(0, 0)
    hydra.init_config_context(args)
    with BagReader(bag_path) as bag, ianvs.init_node_handle(
        "hydra"
    ), hydra.external_plugins("hydra_ros"):
        if trajectory_path is None:
            trajectory_path = bag_path / "poses.csv"

        if not trajectory_path.exists():
            trajectory = load_trajectory_from_bag(
                bag, f"{name}/odom", f"{name}/body", progress=True
            )
            trajectory.to_csv(trajectory_path)
        else:
            trajectory = Trajectory.from_csv(trajectory_path)

        dataloader = RosbagDataLoader(
            bag,
            f"/{name}/{name}_zed/rgb/image_rect_color",
            trajectory=trajectory,
            other_topics=[f"/{name}/{name}_zed/depth/depth_registered"],
            body_frame=f"{name}/body",
            progress=False,
            start_time_ns=_convert_start_time(bag_start_s),
        )

        frame_idx = 0
        last_stamp: int | None = None
        threshold_ns = int(min_separation_s * 1.0e9)
        camera = hydra.make_camera(**dataloader.intrinsics)
        pipeline = hydra.ReconstructionPipeline(camera)
        for stamp, pose, images in dataloader:
            if max_steps and frame_idx >= max_steps:
                break

            if last_stamp is not None and abs(stamp - last_stamp) < threshold_ns:
                continue

            rgb = images[0][..., ::-1]
            depth = images[1]

            q_xyzw = pose.rotation.as_quat()
            q_wxyz = [q_xyzw[i] for i in [3, 0, 1, 2]]
            pipeline.step(stamp, q_wxyz, pose.translation, rgb, depth)
            last_stamp = stamp
            frame_idx += 1

            click.pause("Press any key to continue to next frame...")

        click.pause("Press any key to exit...")


if __name__ == "__main__":
    run()
