"""Command to gather timing information."""

import pathlib
import sys

import click
import hydra_eval.timing as timing


@click.group(name="timing")
def cli():
    """Display info about Hydra timing in various ways."""
    pass


@cli.command(name="show")
@click.argument("result_path", type=click.Path(exists=True))
@click.option(
    "-k",
    "--key",
    "keys",
    multiple=True,
    default=None,
    help="optional regex filters to select relevant timers",
)
def show(result_path, keys):
    """Print timing distribution from Hydra's output at RESULT_PATH."""
    result_path = pathlib.Path(result_path).expanduser().absolute()
    durations = timing.collect_timing_info(result_path)
    if len(durations) == 0:
        click.secho(f"Result path {result_path} does not contain timing info", fg="red")
        sys.exit(1)

    key_regex = None
    if keys:
        key_regex = "|".join(keys)

    timing.show_timing_info(durations, key_regex=key_regex)


@cli.command(name="plot")
@click.argument("result_path", type=click.Path(exists=True))
@click.option(
    "-k",
    "--key",
    "keys",
    multiple=True,
    default=("frontend.*", "backend.*"),
    help="optional regex filters to select timers for each subplot",
)
def plot(result_path, keys):
    """Plot timing distribution from Hydra's output at RESULT_PATH."""
    result_path = pathlib.Path(result_path).expanduser().absolute()
    durations = timing.collect_timing_info(result_path)
    if len(durations) == 0:
        click.secho(f"Result path {result_path} does not contain timing info", fg="red")
        sys.exit(1)

    timing.plot_durations(durations, keys=keys)


@cli.command(name="timeline")
@click.argument("result_path", type=click.Path(exists=True))
@click.option(
    "-k",
    "--key",
    "keys",
    multiple=True,
    default=("frontend.*", "backend.*"),
    help="optional regex filters to select timers to plot",
)
def timeline(result_path, keys):
    """Plot timing information against time from Hydra's output at RESULT_PATH."""
    result_path = pathlib.Path(result_path).expanduser().absolute()
    durations = timing.collect_timing_info(result_path)
    if len(durations) == 0:
        click.secho(f"Result path {result_path} does not contain timing info", fg="red")
        sys.exit(1)

    timing.plot_trends(durations, keys=keys)


@cli.command(name="compare")
@click.argument("results", nargs=-1, type=click.Path(exists=True))
@click.option(
    "-k",
    "--key",
    "keys",
    multiple=True,
    default=("frontend/spin", "backend/spin"),
    help="timer names to use for comparison",
)
@click.option(
    "--use-bar",
    is_flag=True,
    default=False,
    help="use a bar plot instead of a boxen plot",
)
def compare(results, keys, use_bar):
    """Plot timing comparison between different runs of Hydra from RESULTS."""
    if len(results) == 0:
        return

    paths = [pathlib.Path(x).expanduser().absolute() for x in results]
    results = {path.stem: timing.collect_timing_info(path) for path in paths}
    timing.plot_comparison(results, keys, use_bars=use_bar)
