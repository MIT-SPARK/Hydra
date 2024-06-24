"""Commands to run Hydra."""

import yaml
import click
import pathlib
import imageio.v3
import subprocess
import hydra_python as hydra
from spark_dsg.open3d_visualization import DsgVisualizer


@click.group(name="run")
def cli():
    """Commands to run hydra against generic data sources."""
    pass


class FileDataInterface:
    """Class for loading files."""

    def __init__(self, datapath):
        """Load poses and prep for running."""
        self._path = pathlib.Path(datapath).expanduser().absolute()
        self._poses = hydra.Trajectory.from_csv(self._path / "poses.csv")
        self._index = 0
        self._colormap = None
        with (self._path / "camera_info.yaml").open("r") as fin:
            self._camera_info = yaml.safe_load(fin.read())

    def set_colormap(self, colormap):
        """Set colormap."""
        self._colormap = colormap

    def set_pose(self, timestamp, world_T_pose):
        """Increment index."""
        self._index += 1
        if self._index >= len(self._poses):
            self._index = len(self._poses) - 1

    @property
    def poses(self):
        """Get saved trajectory."""
        return self._poses

    @property
    def colormap(self):
        """Get current colormap."""
        return self._colormap

    @property
    def camera_info(self):
        """Get camera info."""
        return self._camera_info

    @property
    def depth(self):
        """Get depth image."""
        return imageio.v3.imread(self._path / f"depth_{self._index:07d}.tiff")

    @property
    def labels(self):
        """Get label image."""
        return imageio.v3.imread(self._path / f"labels_{self._index:07d}.png")

    @property
    def rgb(self):
        """Get rbg image."""
        return imageio.v3.imread(self._path / f"rgb_{self._index:07d}.png")


hydra.DataInterface.register(FileDataInterface)


def _load_data(data_path):
    data_path = pathlib.Path(data_path).expanduser().absolute()
    if data_path.is_dir():
        return FileDataInterface(data_path)

    if data_path.suffix == ".glb":
        from hydra_python._plugins import habitat

        return habitat.HabitatInterface(data_path)

    if data_path.suffix == ".bag":
        click.secho("rosbag not implemented yet!", fg="red")
        return None

    return None


def _load_pipeline(
    data, config_name, label_space, output_path=None, config_verbosity=0
):
    configs = hydra.load_configs(config_name, labelspace_name=label_space)
    if not configs:
        click.secho(
            f"Invalid config: dataset '{config_name}' and label space '{label_space}'",
            fg="red",
        )
        return

    pipeline_config = hydra.PipelineConfig(configs)
    if data.colormap is None:
        names = []
        max_idx = max([x for x in pipeline_config.label_names])
        min_idx = min([x for x in pipeline_config.label_names])
        for i in range(min_idx, max_idx + 1):
            names.append(pipeline_config.label_names.get(i, f"label_{i}"))

        data.set_colormap(hydra.SegmentationColormap.from_names(names))
    else:
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

    return pipeline


@cli.command(name="scene")
@click.argument("scene_path", type=click.Path(exists=True))
@click.option("-c", "--config-name", default="habitat")
@click.option("-o", "--output-path", default=None)
@click.option("-l", "--label-space", default="ade20k_mp3d")
@click.option("-v", "--visualize", default=False, help="start visualizer", is_flag=True)
@click.option("-g", "--glog-level", default=0, help="minimum glog level")
@click.option("-y", "--force", is_flag=True, help="overwrite previous output")
@click.option("-m", "--max-frames", default=None, type=int, help="cap max frames")
@click.option("--verbosity", default=0, help="glog verbosity")
@click.option("--show-images", default=False, help="show semantics", is_flag=True)
@click.option("--show-config", default=False, help="show hydra config", is_flag=True)
@click.option("--show-progress", default=False, help="show progress bar", is_flag=True)
def run(
    scene_path,
    config_name,
    output_path,
    label_space,
    visualize,
    glog_level,
    force,
    max_frames,
    verbosity,
    show_images,
    show_config,
    show_progress,
):
    """Run Hydra against a single scene."""
    hydra.set_glog_level(glog_level, verbosity)
    output_path = hydra.resolve_output_path(output_path, force=force)
    data = _load_data(scene_path)
    cfg_verbosity = 0 if show_config else 1
    pipeline = _load_pipeline(
        data,
        config_name,
        label_space,
        output_path=output_path,
        config_verbosity=cfg_verbosity,
    )

    visualizer = None
    if visualize:
        visualizer = DsgVisualizer(start_remote=False)
        visualizer.update_graph(pipeline.graph)

    poses = data.poses
    if max_frames is not None:
        poses = poses[:max_frames]

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


@cli.command(name="scenes")
@click.argument("output_path", type=click.Path())
@click.argument("scene_paths", type=click.Path(exists=True), nargs=-1)
@click.option("-c", "--config-name", default="habitat")
@click.option("-l", "--label-space", default="ade20k_mp3d")
@click.option("-v", "--visualize", default=False, help="start visualizer", is_flag=True)
@click.option("-g", "--glog-level", default=0, help="minimum glog level")
@click.option("-y", "--force", is_flag=True, help="overwrite previous output")
@click.option("-m", "--max-frames", default=None, type=int, help="cap max frames")
@click.option("--verbosity", default=0, help="glog verbosity")
@click.option("--show-images", default=False, help="show semantics", is_flag=True)
def run_scenes(
    output_path,
    scene_paths,
    config_name,
    label_space,
    visualize,
    glog_level,
    force,
    max_frames,
    verbosity,
    show_images,
):
    """Run Hydra against a single scene."""
    args = ["hydra", "run", "scene"]
    args += ["-c", config_name]
    args += ["-l", label_space]

    if visualize:
        args += ["-v"]
    args += ["-g", str(glog_level)]

    if force:
        args += ["-y"]

    if max_frames is not None:
        args += ["-m", str(max_frames)]

    args += ["--verbosity", str(verbosity)]
    if show_images:
        args += ["--show-images"]

    args += ["--show-progress"]

    hydra.set_glog_level(glog_level, verbosity)
    output_path = hydra.resolve_output_path(output_path, force=force)

    for scene_path in scene_paths:
        scene_path = pathlib.Path(scene_path).expanduser().absolute()
        scene_name = scene_path.stem
        scene_output = output_path / scene_name
        click.secho(f"Starting {scene_path}")

        scene_args = args + ["-o", str(scene_output), str(scene_path)]
        subprocess.run(scene_args)
