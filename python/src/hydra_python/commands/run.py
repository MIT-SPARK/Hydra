#! /usr/bin/env python3
"""Script to run Hydra against MP3D image datasets."""

import pathlib
import shutil
import signal
import traceback

import click
import spark_dataset_interfaces as sdi

import hydra_python as hydra


def _get_scene_output(output_path, scene_name, force):
    scene_output = output_path / scene_name
    if scene_output.exists():
        if not force:
            click.secho(f"Skipping existing output: '{scene_output}'")
            return None

        click.secho(f"Removing existing output: '{scene_output}'", fg="yellow")
        shutil.rmtree(scene_output)

    return scene_output


def _decompose_pose(pose):
    q_xyzw = pose.rotation.as_quat()
    q_wxyz = [q_xyzw[i] for i in [3, 0, 1, 2]]
    return q_wxyz, pose.translation


@click.group(name="run")
def cli():
    """Commands to run hydra against a dataset(s)."""
    pass


@cli.command("mp3d")
@click.argument("scenes", nargs=-1, type=click.Path(exists=True))
@click.option("--visualize", "-v", is_flag=True)
@click.option("--zmq-url", default="tcp://127.0.0.1:8001")
@click.option("--max-steps", "-m", default=None, type=int)
@click.option("--force", "-f", is_flag=True, help="overwrite existing scenes.")
@click.option("--output", "-o", default=None, type=click.Path())
def mp3d(scenes, visualize, zmq_url, max_steps, force, output):
    hydra.set_glog_level(0, 0)
    scenes = [pathlib.Path(x).expanduser().resolve() for x in scenes]
    click.secho(f"Processing scenes: {', '.join(x.stem for x in scenes)}", fg="cyan")

    if output is not None:
        output = pathlib.Path(output).expanduser().absolute()
    else:
        output = pathlib.Path(".").absolute()

    output.mkdir(parents=True, exist_ok=True)
    click.secho(f"Output directory: {output}", fg="green")

    # NOTE(nathan) something messes with SIGINT and disables it (this puts it back?)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    for scene_path in scenes:
        try:
            dataloader = sdi.FileDataLoader(scene_path)
            sensor = hydra.make_camera(**dataloader.intrinsics)
            scene_output = _get_scene_output(output, scene_path.stem, force)
            if scene_output is None:
                continue

            pipeline = hydra.load_pipeline(
                sensor,
                "habitat",
                "ade20k_mp3d",
                output_path=scene_output,
                freeze_global_info=False,
                zmq_url=zmq_url if visualize else None,
            )
            if pipeline is None:
                click.secho("Failed to load pipeline!", fg="red")
                continue

            def _step_pipeline(packet):
                rotation, translation = _decompose_pose(packet.pose)
                pipeline.step(
                    packet.timestamp,
                    translation,
                    rotation,
                    packet.depth,
                    packet.labels,
                    packet.color,
                    **packet.extras,
                )

            sdi.DataLoader.run(
                dataloader,
                _step_pipeline,
                max_steps=max_steps,
                step_mode=False,
                show_progress=True,
            )

            pipeline.save(f"{output}")

        except Exception:
            click.secho(f"Pipeline failed for '{scene_path}'", fg="red")
            click.secho(f"{traceback.format_exc()}", fg="red")
