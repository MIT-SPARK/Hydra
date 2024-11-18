"""Entry points for hydra."""

import click
import hydra_python.commands.batch as batch
import hydra_python.commands.reconstruct as reconstruct
import hydra_python.commands.run as run
import hydra_python.commands.visualize as visualize


@click.group()
def cli():
    """Entry point target for subcommands."""
    pass


cli.add_command(batch.cli)
cli.add_command(reconstruct.cli)
cli.add_command(run.cli)
cli.add_command(visualize.cli)
