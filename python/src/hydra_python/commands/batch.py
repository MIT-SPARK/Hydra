"""Entry point for constructing scene graph from scene reconstruction."""
import pathlib

import click
import hydra_python as hydra


@click.command(name="batch")
@click.argument("input_paths", type=click.Path(exists=True), nargs=-1)
@click.option("-v", "--verbosity", default=0)
def cli(input_paths, verbosity):
    """Make scene graphs from reconstructed information."""
    hydra.set_glog_level(0, verbosity)
    configs = hydra.load_configs("habitat", labelspace_name="ade20k_mp3d")
    if not configs:
        click.secho(
            "Invalid config: dataset 'habitat' and label space 'ade20k_mp3d'",
            fg="red",
        )
        return

    pipeline_config = hydra.PipelineConfig(configs)
    # TODO(nathan) colormap
    graph_builder = hydra.BatchPipeline(pipeline_config)

    click.echo(f"processing {len(input_paths)} graph(s):")
    for input_path in input_paths:
        input_path = pathlib.Path(input_path).expanduser().absolute()
        click.echo(f"  - {input_path} ... ", nl=False)
        map_path = input_path / "map"
        dense_map = hydra.VolumetricMap.load(map_path)

        graph = graph_builder.construct(configs, dense_map)
        graph.save(str(input_path / "dsg_with_mesh.json"), include_mesh=True)
        graph.save(str(input_path / "dsg.json"), include_mesh=False)
        click.secho("done", fg="green")
