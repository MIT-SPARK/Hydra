"""Commands for doing 3D reconstruction."""

import click


@click.group(name="run")
def cli():
    """Commands to run hydra against a dataset(s)."""
    pass
