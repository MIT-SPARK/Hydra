"""Command-line interface."""

import pathlib
import shutil
from typing import Optional, Union

import click

OptPath = Optional[Union[str, pathlib.Path]]


def resolve_output_path(output_path: OptPath, force: bool = False):
    """
    Check if an output path is valid.

    Args:
        output_path: Optional path for hydra to output to
        force: Automatically clear previous output if it exists
    """
    if output_path is None:
        return None

    output_path = pathlib.Path(output_path).expanduser().absolute()
    if not output_path.exists():
        return output_path

    click.secho(f"[WARNING]: output {output_path} already exists", fg="yellow")
    if not force:
        click.confirm(f"remove {output_path}?", abort=True, default=False)

    shutil.rmtree(output_path)
    return output_path
