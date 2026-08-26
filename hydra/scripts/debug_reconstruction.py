#!/usr/bin/env python3
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


@click.group()
def cli():
    pass


@cli.command(name="trajectory")
@click.argument("bag_path", type=click.Path(exists=True))
@click.argument("output", type=click.Path())
def save_trajectory(bag_path, output):
    with BagReader(bag_path) as bag:
        trajectory = load_trajectory_from_bag(
            bag, "hamilton/odom", "hamilton/body", progress=True
        )
        trajectory.to_csv(output)


def _convert_start_time(bag_start_s: float | None):
    if bag_start_s is None:
        return None

    return int(bag_start_s * 1.0e9)


@cli.command(name="run")
@click.argument("bag_path", type=click.Path(exists=True))
@click.argument("trajectory_path", type=click.Path(exists=True))
@click.option("--max-steps", "-m", default=None, type=int)
@click.option("--min-separation-s", "-s", default=0.0, type=float)
@click.option("--bag-start-s", default=None, type=float)
@click.option("--config-utilities-files", "-f", multiple=True)
@click.option("--config-utilities-yaml", "-c", multiple=True)
@click.option("--config-utilities-var", "-v", multiple=True)
def run(
    bag_path,
    trajectory_path,
    max_steps,
    min_separation_s,
    bag_start_s,
    config_utilities_files,
    config_utilities_yaml,
    config_utilities_var,
):
    args = _repair_args(config_utilities_files, "-f")
    args += _repair_args(config_utilities_yaml, "-c")
    args += _repair_args(config_utilities_var, "-v")
    args += ["-c", "{app_plugins: [{type: ConfigServerPlugin}, {type: SpinPlugin}]}"]
    args += ["-c", "{verbosity: 1}"]

    hydra.set_glog_level(0, 0)
    hydra.init_config_context(args)

    trajectory = Trajectory.from_csv(trajectory_path)
    with BagReader(bag_path) as bag, ianvs.init_node_handle(
        "hydra"
    ), hydra.external_plugins("hydra_ros"):
        dataloader = RosbagDataLoader(
            bag,
            trajectory,
            "/hamilton/hamilton_zed/rgb/image_rect_color",
            ["/hamilton/hamilton_zed/depth/depth_registered"],
            body_frame="hamilton/body",
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
    cli()
