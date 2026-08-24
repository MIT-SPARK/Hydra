"""Entry points for hydra."""

import click

import hydra_python.commands.run as run


@click.group()
def cli():
    """Entry point target for subcommands."""
    pass


cli.add_command(run.cli)
