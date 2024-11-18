"""Run Hydra stuff."""

import subprocess
import pathlib
import click
import os


@click.group(name="mp3d")
def cli():
    """Commands to run against all of MP3D."""
    pass


@cli.command(name="run")
@click.argument("mp3d_path", type=click.Path(exists=True))
@click.argument("output")
@click.option("-m", "--model_file", type=click.Path(exists=True), default=None)
@click.option("-s", "--seed", default=None, help="random seed")
@click.option("-v", "--visualize", default=False, help="start visualizer", is_flag=True)
@click.option("-f", "--use-full-scene", is_flag=True, help="use-full-scene")
def run(mp3d_path, output, model_file, seed, visualize, use_full_scene):
    """Run Hydra online against the mp3d dataset."""
    mp3d_path = pathlib.Path(mp3d_path).expanduser().absolute()
    output = pathlib.Path(output).expanduser().absolute()
    mp3d_scenes = [x for x in mp3d_path.glob("**/*.glb")]
    for scene in mp3d_scenes:
        output_path = output / scene.stem
        args = ["hydra", "habitat", "run", str(scene), "-o", str(output_path)]
        if model_file:
            args += ["--model_file", str(model_file)]

        if seed:
            args += ["--seed", str(seed)]

        if visualize:
            args += ["--publish"]

        if use_full_scene:
            args += ["--use-full-scene"]

        args += ["--show-progress", "--force", "--config-verbosity", "1"]

        env = os.environ.copy()
        env["GLOG_minloglevel"] = "2"
        subprocess.run(args, env=env)


@cli.command(name="reconstruction")
@click.argument("mp3d_path", type=click.Path(exists=True))
@click.argument("output")
@click.option("--show-images", default=False, help="show semantics", is_flag=True)
@click.option("--voxel-size", default=0.1, type=float)
def run_reconstruction(mp3d_path, output, show_images, voxel_size):
    """Run reconstruction on all mp3d scenes."""
    mp3d_path = pathlib.Path(mp3d_path).expanduser().absolute()
    output = pathlib.Path(output).expanduser().absolute()
    mp3d_scenes = [x for x in mp3d_path.glob("**/*.glb")]
    for scene in mp3d_scenes:
        output_path = output / scene.stem
        args = [
            "hydra",
            "habitat",
            "reconstruction",
            str(scene),
            "-o",
            str(output_path),
        ]
        args += ["--show-progress", "--force", "--use-full-scene"]
        args += ["--voxel-size", str(voxel_size)]

        if show_images:
            args += ["--show_images"]

        env = os.environ.copy()
        env["GLOG_minloglevel"] = "2"
        subprocess.run(args, env=env)
