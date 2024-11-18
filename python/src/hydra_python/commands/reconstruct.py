"""Entry point for running metric-semantic reconstruction for a scene(s)."""

import pathlib

import click
import hydra_python
import yaml
from hydra_python._hydra_bindings import HydraReconstruction
from hydra_python.commands import resolve_output_path
from hydra_python.data_loader import DataLoader


@click.command(name="reconstruct")
@click.argument("scenes", type=click.Path(exists=True), nargs=-1)
@click.option("-o", "--output-path", default=None, help="output directory")
@click.option(
    "-s", "--voxel-size", default=0.1, type=float, help="reconstruction resolution"
)
@click.option("-l", "--label-space", default="ade20k_mp3d", help="semantic label space")
@click.option("-f", "--force", is_flag=True, help="overwrite previous output")
@click.option(
    "-m", "--max-updates", default=None, type=int, help="max number of updates"
)
@click.option("-g", "--glog-level", default=0, help="minimum glog level")
@click.option("-v", "--verbosity", default=0, help="glog verbosity")
@click.option("--images/--no-images", default=False, help="show input color images")
@click.option("--visualize/--no-visualize", default=False, help="publish mesh to zmq")
@click.option("--progress/--no-progress", default=True, help="show progress bar")
def cli(
    scenes,
    output_path,
    voxel_size,
    label_space,
    force,
    max_updates,
    glog_level,
    verbosity,
    images,
    visualize,
    progress,
):
    """Reconstruct a mesh and TSDF of a habitat scene."""
    hydra_python.set_glog_level(glog_level, verbosity)
    output_path = resolve_output_path(output_path, force=force)

    for scene_path in scenes:
        scene_path = pathlib.Path(scene_path).expanduser().absolute()
        data = hydra_python.get_dataloader(scene_path)

        config = {
            "verbosity": verbosity,
            "visualize_mesh": visualize,
            "reconstruction": {"volumetric_map": {"voxel_size": voxel_size}},
        }
        pipeline = HydraReconstruction.from_config(yaml.safe_dump(config), data.sensor)
        try:
            DataLoader.run(
                pipeline, data, max_steps=max_updates, show_progress=progress
            )
        finally:
            if output_path is not None:
                pipeline.save(output_path / scene_path.stem)
