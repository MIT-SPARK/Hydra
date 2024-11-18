"""Commands for running with the habitat simulator."""

import pathlib

import click
import hydra_python as hydra
from spark_dsg.open3d_visualization import DsgVisualizer


def _get_trajectory(data, prev_dsg, seed, use_full_scene=False):
    if prev_dsg:
        prev_path = pathlib.Path(prev_dsg).resolve()
        poses = hydra.Trajectory.from_scene_graph(str(prev_path))
    elif use_full_scene:
        poses = data.get_full_trajectory(inflation_radius=0.25, seed=seed)
        if poses is None:
            click.secho(
                "Failed to find trajectory for single room! Defaulting to random",
                fg="yellow",
            )
            poses = data.get_random_trajectory(inflation_radius=0.25, seed=seed)
    else:
        poses = data.get_random_trajectory(inflation_radius=0.25, seed=seed)

    click.secho(f"Trajectory is {poses.get_path_length()} meters long", fg="green")
    return poses


@click.group(name="habitat")
def cli():
    """Entry points for running on habitat."""
    pass


@cli.command(name="run")
@click.argument("scene_path", type=click.Path(exists=True))
@click.option("-o", "--output-path", default=None)
@click.option("-l", "--label-space", default="ade20k_mp3d")
@click.option("-p", "--prev-dsg", default=None, help="dsg containing trajectory")
@click.option("-s", "--seed", default=None, help="random seed")
@click.option("-v", "--visualize", default=False, help="start visualizer", is_flag=True)
@click.option("-g", "--glog-level", default=0, help="minimum glog level")
@click.option("-f", "--use-full-scene", is_flag=True, help="use-full-scene")
@click.option("-y", "--force", is_flag=True, help="overwrite previous output")
@click.option("--verbosity", default=0, help="glog verbosity")
@click.option("--show-images", default=False, help="show semantics", is_flag=True)
@click.option("--show-config", default=False, help="show hydra config", is_flag=True)
@click.option("--show-progress", default=False, help="show progress bar", is_flag=True)
@click.option("--config-verbosity", default=0, help="glog verbosity to print configs")
def run(
    scene_path,
    output_path,
    label_space,
    prev_dsg,
    seed,
    visualize,
    glog_level,
    use_full_scene,
    force,
    verbosity,
    show_images,
    show_config,
    show_progress,
    config_verbosity,
):
    """Run Hydra against a habitat scene."""
    from hydra_python._plugins import habitat

    hydra.set_glog_level(glog_level, verbosity)
    output_path = hydra.resolve_output_path(output_path, force=force)
    data = habitat.HabitatInterface(scene_path)
    poses = _get_trajectory(data, prev_dsg, seed, use_full_scene=use_full_scene)

    configs = hydra.load_configs("habitat", labelspace_name=label_space)
    if not configs:
        click.secho(
            f"Invalid config: dataset 'habitat' and label space '{label_space}'",
            fg="red",
        )
        return

    pipeline_config = hydra.PipelineConfig(configs)
    pipeline_config.enable_reconstruction = True
    pipeline_config.label_names = {i: x for i, x in enumerate(data.colormap.names)}
    data.colormap.fill_label_space(pipeline_config.label_space)
    if output_path:
        pipeline_config.logs.log_dir = str(output_path)

    pipeline = hydra.HydraPipeline(
        pipeline_config, robot_id=0, config_verbosity=config_verbosity
    )
    pipeline.init(configs, hydra.create_camera(data.camera_info))
    if output_path:
        glog_dir = output_path / "logs"
        if not glog_dir.exists():
            glog_dir.mkdir()
        # TODO(nathan) make pathlib bindings
        hydra.set_glog_dir(str(glog_dir))

    if visualize:
        visualizer = DsgVisualizer(start_remote=False)
        visualizer.update_graph(pipeline.graph)
    else:
        visualizer = None

    try:
        hydra.run(
            pipeline,
            data,
            poses,
            visualizer=visualizer,
            show_images=show_images,
            show_progress=show_progress,
        )
    finally:
        pipeline.save()
        if visualizer is not None:
            visualizer.stop()
