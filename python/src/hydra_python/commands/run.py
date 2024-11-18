"""Entry point for running Hydra incrementally."""

import pathlib

import click
import hydra_python
import spark_dsg
from hydra_python.commands import resolve_output_path
from hydra_python.data_loader import DataLoader
from hydra_python.pipeline import load_pipeline


@click.command(name="run")
@click.argument("scenes", type=click.Path(exists=True), nargs=-1)
@click.option("-c", "--config-name", default="habitat")
@click.option("-l", "--labelspace", default="ade20k_mp3d")
@click.option("-o", "--output-path", default=None)
@click.option("-g", "--glog-level", default=0, help="minimum glog level")
@click.option("-y", "--force", is_flag=True, help="overwrite previous output")
@click.option("-m", "--max-steps", default=None, type=int, help="total number of steps")
@click.option("--openset-model", default=None, type=str, help="clip model to use")
@click.option("-v", "--verbosity", default=0, help="glog verbosity")
@click.option("--show-images", default=False, help="show semantics", is_flag=True)
@click.option("--show-config", default=False, help="show hydra config", is_flag=True)
@click.option("--publish/--no-publish", default=True, help="publish over zmq")
@click.option("--progress/--no-progress", default=True, help="show progress bar")
def cli(
    scenes,
    config_name,
    output_path,
    labelspace,
    glog_level,
    force,
    max_steps,
    openset_model,
    verbosity,
    show_images,
    show_config,
    publish,
    progress,
):
    """Run Hydra against various scenes."""
    hydra_python.set_glog_level(glog_level, verbosity)
    output_path = resolve_output_path(output_path, force=force)

    for scene_path in scenes:
        scene_path = pathlib.Path(scene_path).expanduser().absolute()
        scene_output_path = (
            None if output_path is None else output_path / scene_path.stem
        )
        data = hydra_python.get_dataloader(scene_path)
        pipeline = load_pipeline(
            data,
            config_name,
            labelspace,
            output_path=scene_output_path,
            config_verbosity=0 if show_config else 1,
            place_feature_strategy=None if openset_model is None else "fusion",
            zmq_url=None if not publish else "tcp://127.0.0.1:8001",
        )

        if not pipeline:
            click.secho(f"Failed to load pipeline for {scene_path}", fg="red")
            continue

        callbacks = {}
        if openset_model is not None:
            callbacks["clip"] = {"model_name": openset_model}
        if show_images:
            callbacks["image_viewer"] = {}

        try:
            DataLoader.run(
                pipeline,
                data,
                max_steps=max_steps,
                show_progress=progress,
                data_callbacks=hydra_python.DataCallbackRegistry.create(callbacks),
            )
        finally:
            pipeline.save()
