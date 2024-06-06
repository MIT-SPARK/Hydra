"""Entry points for hydra."""

import hydra_python.commands.habitat as habitat
import hydra_python.commands.mp3d as mp3d
import hydra_python.commands.run as run

from spark_dsg.open3d_visualization import RemoteVisualizer
import click


@click.group()
def cli():
    """Entry point target for subcommands."""
    pass


@cli.command(name="visualize")
def visualize():
    """Run Hydra visualizer."""
    visualizer = RemoteVisualizer(num_dynamic_to_skip=1)
    visualizer.run()


cli.add_command(habitat.cli)
cli.add_command(mp3d.cli)
cli.add_command(run.cli)
