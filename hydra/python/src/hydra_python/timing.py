"""Some small helpers for getting timing information."""

import logging
import pathlib
import re

import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

DEFAULT_COMPARE_KEYS = [
    "frontend/object_detection",
    "gvd/extract_graph",
    "backend/room_detection",
]


def _get_time_array_from_log(filename):
    with pathlib.Path(filename).open() as stream:
        next(stream, None)
        rows = [line for line in stream if line.strip()]

    if not rows:
        return np.empty((0, 2))

    arr = np.loadtxt(rows, delimiter=",", ndmin=2)
    if arr.shape[1] != 2:
        raise ValueError(f"Expected timestamp(ns),elapsed(s) columns in {filename}")

    return arr


def _get_filenames(result_path):
    result_path = pathlib.Path(result_path)
    if not result_path.exists():
        return []

    return [f for f in result_path.rglob("*timing_raw.csv")]


def _get_timer_name(path):
    parent_name = path.parent.name
    parent_name = "" if parent_name == "timing" else parent_name + "_"
    name = parent_name + path.stem[: -len("_timing_raw")]
    return name.replace("/", "_")


def _draw_realtime_threshold(ax, threshold, padding=0.022):
    if threshold is None:
        return

    ax.set_ylabel("Elapsed Time [s]")
    ax.axhline(threshold, ls="--", c="k")
    ax.text(-0.3, threshold - padding, "Real-Time (Keyframe Period)")
    if ax.get_ylim()[1] < threshold + 0.02:
        ax.set_ylim([ax.get_ylim()[0], threshold + 0.01])


def _get_longform_df(durations, key=None):
    import pandas as pd

    names = []
    data = np.array([])
    matcher = re.compile(key) if key is not None else None
    for name, info in durations.items():
        if matcher is not None and not matcher.match(name):
            continue

        times = info[:, 1]
        data = np.concatenate((data, times), axis=None)
        names += [name] * len(times)

    if len(names) == 0:
        return None

    return pd.DataFrame({"Name": names, "Elapsed Time [s]": data})


def collect_timing_info(path, folders_to_use=None):
    """Collect timing info for a specfic path."""
    timing_files = _get_filenames(path)
    if folders_to_use:
        timing_files = [x for x in timing_files if x.parent in folders_to_use]

    return {_get_timer_name(f): _get_time_array_from_log(f) for f in timing_files}


def collate_timers(info, timers, max_diff_ns=1000000):
    """Combine multiple timers together."""
    logger = logging.getLogger(__name__)

    stamps = []
    elapsed = []
    if not timers or any(info[timer].shape[0] == 0 for timer in timers):
        return np.array(stamps), np.array(elapsed)

    for stamp in info[timers[0]][:, 0]:
        curr_elapsed = []
        for timer in timers:
            diff = np.abs(info[timer][:, 0] - stamp)
            idx = np.argmin(diff)
            if diff[idx] > max_diff_ns:
                best_stamp = info[timer][idx][0]
                logger.warning(f"could not find stamp @ {stamp} [ns] for timer {timer}")
                logger.warning(f"best {best_stamp} [ns] (diff {diff[idx]} [ns]")
                continue

            curr_elapsed.append(info[timer][idx, 1])

        if len(curr_elapsed) != len(timers):
            continue

        stamps.append(stamp)
        elapsed.append(sum(curr_elapsed))

    return np.array(stamps), np.array(elapsed)


def show_timing_info(data, key_regex=None):
    """Display table of timing information."""
    from rich.console import Console
    from rich.table import Table

    def _get_stat_str(stat):
        return rf"{1000 * stat:>.3f}"

    sorted_keys = sorted(data)
    if key_regex is not None:
        matcher = re.compile(key_regex)
        sorted_keys = [x for x in sorted_keys if matcher.match(x)]

    table = Table(title="Timing Information")
    table.add_column("Timer")
    table.add_column(r"μ \[ms]")
    table.add_column(r"σ \[ms]")
    table.add_column(r"Min \[ms]")
    table.add_column(r"Max \[ms]")

    for key in sorted_keys:
        values = data[key][:, 1]
        if values.size == 0:
            continue

        stats = [np.mean(values), np.std(values), np.min(values), np.max(values)]
        table.add_row(key, *(_get_stat_str(x) for x in stats))

    Console().print(table)


def plot_durations(durations, keys):
    """Make a plot of durations."""
    sns.set()
    sns.set_style("whitegrid")
    sns.set_context("notebook")
    fig, ax = plt.subplots(len(keys), 1, squeeze=False)

    for idx, key in enumerate(keys):
        ax[idx][0].set_title(f"{key} Timing Distributions")
        df = _get_longform_df(durations, key=key)
        if df is None:
            continue

        sns.boxenplot(data=df, x="Name", y="Elapsed Time [s]", ax=ax[idx][0])
        lax = ax[idx][0]
        lax.set_xticks(lax.get_xticks(), lax.get_xticklabels(), rotation=30, ha="right")

    fig.set_size_inches([14, 12 * len(keys)])
    fig.tight_layout()
    plt.show()


def plot_trends(durations, keys):
    """Make a plot of durations."""
    sns.set()
    sns.set_style("whitegrid")
    sns.set_context("notebook")
    fig, ax = plt.subplots(len(keys), 1, squeeze=False)

    for idx, key in enumerate(keys):
        added_plot = False
        matcher = re.compile(key)
        for name, time_array in durations.items():
            if not matcher.match(name):
                continue

            assert len(time_array.shape) == 2 and time_array.shape[1] == 2
            to_plot = time_array
            if to_plot.shape[0] < 2:
                continue

            ax[idx][0].plot(to_plot[:, 0] * 1.0e-9, to_plot[:, 1], label=name)
            added_plot = True

        ax[idx][0].set_title(f"{key} Timing Trends")
        ax[idx][0].set_xlabel("Timestamp [s]")
        ax[idx][0].set_ylabel("Elapsed Time [s]")
        if added_plot:
            ax[idx][0].legend()

    fig.set_size_inches([16, 8 * len(keys)])
    plt.show()


def plot_comparison(results, keys, use_bars=False):
    """Plot timing comparison for different results directories."""
    import pandas as pd

    sns.set()
    sns.set_style("whitegrid")
    sns.set_context("notebook")

    data = np.array([])
    labels = []
    result_set = []

    matcher = None
    if keys:
        matcher = re.compile("|".join(keys))

    for stem, result in results.items():
        for key, values in result.items():
            if matcher and not matcher.match(key):
                continue

            num_values = len(values)
            data = np.hstack((data, np.squeeze(values[:, 1])))
            labels += num_values * [key]
            result_set += num_values * [stem.upper()]

    value_key = "Elapsed Time [s]"
    timer_key = "Timer Name"
    result_key = "Result Name"

    fig, ax = plt.subplots()
    df = pd.DataFrame({value_key: data, timer_key: labels, result_key: result_set})
    if use_bars:
        sns.barplot(x=timer_key, y=value_key, hue=result_key, data=df, ax=ax)
    else:
        sns.boxenplot(x=timer_key, y=value_key, hue=result_key, data=df, ax=ax)

    ax.legend()
    ax.set_xticks(ax.get_xticks(), ax.get_xticklabels(), rotation=30, ha="right")

    fig.set_size_inches([14, 12])
    fig.tight_layout()
    plt.show()
