"""Command to gather timing information."""

import json
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
@click.option("-k", "--key", "keys", multiple=True, default=None)
def show(result_path, keys):
    """Print timing information for a single set of Hydra results."""
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
@click.option("-t", "--show-trends", is_flag=True, default=False)
@click.option("-k", "--key", "keys", multiple=True, default=("frontend", "backend"))
def plot(result_path, show_trends, keys):
    """Plot timing information for a single set of Hydra results."""
    result_path = pathlib.Path(result_path).expanduser().absolute()
    durations = timing.collect_timing_info(result_path)
    if len(durations) == 0:
        click.secho(f"result path {result_path} has no contain timing data", fg="red")
        sys.exit(1)

    if show_trends:
        timing.plot_trends(durations, keys=keys)
    else:
        timing.plot_durations(durations, keys=keys)


@cli.command(name="compare")
@click.argument("results", nargs=-1, type=click.Path(exists=True))
@click.option("-k", "--key", "keys", multiple=True, default=timing.DEFAULT_COMPARE_KEYS)
@click.option("-c", "--config", default=None)
def compare(results, keys, config):
    """Plot timing comparison between different runs of Hydra."""
    if len(results) == 0:
        return

    paths = [pathlib.Path(x).expanduser().absolute() for x in results]
    results = {path.stem: timing.collect_timing_info(path) for path in paths}

    plot_config = {} if config is None else json.loads(config)
    timing.plot_comparison(results, plot_config, keys)
