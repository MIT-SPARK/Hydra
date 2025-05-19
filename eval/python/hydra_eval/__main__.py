"""Entry points for hydra."""

import logging

import click

import hydra_eval.commands.collect as collect
import hydra_eval.commands.timing as timing
from hydra_eval.utils import ClickHandler


@click.group()
def cli():
    """Entry point target for subcommands."""
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)

    handler = ClickHandler()
    handler.setLevel(logging.INFO)
    formatter = logging.Formatter("%(message)s")
    handler.setFormatter(formatter)

    logger.handlers.clear()
    logger.addHandler(handler)


cli.add_command(timing.cli)
cli.add_command(collect.cli)

if __name__ == "__main__":
    cli()
