"""Entry points for Hydra."""

import click

import hydra_python.commands.run as run
import hydra_python.commands.timing as timing
import hydra_python.image_store.cli as image_store


@click.group()
def cli():
    """Run Hydra and inspect its outputs."""


cli.add_command(run.cli)
cli.add_command(timing.cli)
cli.add_command(image_store.cli)
