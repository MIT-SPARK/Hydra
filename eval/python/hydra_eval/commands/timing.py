"""Command to gather timing information."""

import json
import pathlib
import sys

import click
import hydra_eval.timing as timing
import numpy as np
from hydra_eval.utils import get_logger

OBJECT_TIMERS = ["frontend/objects_detection", "frontend/objects_graph_update"]
PLACE_TIMERS = ["frontend/detect_gvd", "frontend/update_gvd_places"]
ROOM_TIMERS = ["backend/room_detection"]
COMPARISON_KEYS = (
    "frontend/object_detection",
    "gvd/extract_graph",
    "backend/room_detection",
)


@click.group(name="timing")
def cli():
    """Display info about Hydra timing in various ways."""
    pass


@cli.command(name="show")
@click.argument("results_dir", type=click.Path(exists=True))
def show(results_dir):
    """Print timing information for a single set of Hydra results."""
    logger = get_logger()
    result_path = pathlib.Path(results_dir).expanduser().absolute()
    durations = timing.collect_timing_info(result_path)
    if len(durations) == 0:
        logger.critical(f"result path {result_path} does not contain timing info")
        sys.exit(1)

    object_timing = timing.collate_timers(durations, OBJECT_TIMERS)[1]
    place_timing = timing.collate_timers(durations, PLACE_TIMERS)[1]
    room_timing = timing.collate_timers(durations, ROOM_TIMERS)[1]

    def _get_stat_str(values):
        mean = np.mean(1000 * values)
        std = np.std(1000 * values)
        min_v = np.min(1000 * values)
        max_v = np.max(1000 * values)
        return rf"{mean:>5.1f} ± {std:>6.1f} [{min_v:>5.1f}, {max_v:>5.1f}]"

    logger.info("Timing:   mean ± stddev [  min,   max]")
    logger.info("===========================================")
    logger.info(f"Objects: {_get_stat_str(object_timing)} [ms]")
    logger.info(f"Places:  {_get_stat_str(place_timing)} [ms]")
    logger.info(f"Rooms:   {_get_stat_str(room_timing)} [ms]")


@cli.command(name="plot")
@click.argument("result_path", type=click.Path(exists=True))
@click.option("-t", "--show-trends", is_flag=True, default=False)
@click.option("-l", "--show-lcd", is_flag=True, default=False)
@click.option("-n", "--name", "names", multiple=True)
@click.option("-k", "--key", "keys", multiple=True, default=("frontend", "backend"))
@click.option("-r", "--realtime-threshold", default=None, type=float)
def plot(result_path, show_trends, show_lcd, names, keys, realtime_threshold):
    """Plot timing information for a single set of Hydra results."""
    logger = get_logger()
    result_path = pathlib.Path(result_path).expanduser().absolute()
    durations = timing.collect_timing_info(result_path)
    if len(durations) == 0:
        logger.critical(f"result path {result_path} does not contain timing info")
        sys.exit(1)

    if len(names) > 0:
        durations = {n: data for n, data in durations.items() if n in names}

    if show_lcd:
        keys = keys + ("lcd",)

    if show_trends:
        timing.plot_trends(durations, keys=keys, rt_threshold=realtime_threshold)
    else:
        timing.plot_durations(durations, keys=keys, rt_threshold=realtime_threshold)


@cli.command(name="compare")
@click.argument("results", nargs=-1, type=click.Path(exists=True))
@click.option("-k", "--key", "keys", multiple=True, default=COMPARISON_KEYS)
@click.option("-c", "--config", default=None)
@click.option("-r", "--realtime-threshold", default=None, type=float)
def compare(results, keys, config, realtime_threshold):
    """Plot timing comparison between different runs of Hydra."""
    if len(results) == 0:
        return

    paths = [pathlib.Path(x).expanduser().absolute() for x in results]
    results = {path.stem: timing.collect_timing_info(path) for path in paths}
    plot_config = {} if config is None else json.loads(config)
    timing.plot_comparison(results, plot_config, keys, rt_threshold=realtime_threshold)
