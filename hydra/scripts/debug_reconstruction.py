#!/usr/bin/env python3
import click
import hydra_python as hydra
from hydra_python.dataloaders.rosbag_dataloader import (
    RosbagDataLoader,
    load_trajectory_from_bag,
)
from ianvs.bag_reader import BagReader


@click.command()
@click.argument("bag_path", type=click.Path(exists=True))
@click.option("--max-steps", "-m", default=None, type=int)
def main(bag_path, max_steps):
    hydra.set_glog_level(0, 0)

    with BagReader(bag_path) as bag:
        trajectory = load_trajectory_from_bag(bag, "hamilton/odom", "hamilton/body")
        dataloader = RosbagDataLoader(
            bag,
            trajectory,
            "/hamilton/hamilton_zed/rgb/image_rect_color",
            ["/hamilton/hamilton_zed/depth/depth_registered"],
            body_frame="hamilton/body",
        )

        for timestamp, pose, messages in dataloader:
            break

    # sensor = hydra.make_camera(**dataloader.intrinsics)
    # pipeline = hydra.load_pipeline(
    #     sensor, "habitat", "ade20k_mp3d", freeze_global_info=False
    # )
    # if pipeline is None:
    #     click.secho("Failed to load pipeline!", fg="red")
    #     return
    #
    # for idx, packet in enumerate(dataloader):
    #     if max_steps and idx >= max_steps:
    #         return
    #
    #     q_xyzw = packet.pose.rotation.as_quat()
    #     q_wxyz = [q_xyzw[i] for i in [3, 0, 1, 2]]
    #     pipeline.step(
    #         packet.timestamp,
    #         packet.pose.translation,
    #         q_wxyz,
    #         packet.depth,
    #         packet.labels,
    #         packet.color,
    #         **packet.extras,
    #     )


if __name__ == "__main__":
    main()
