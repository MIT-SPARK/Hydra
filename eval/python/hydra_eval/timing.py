"""Some small helpers for getting timing information."""

import logging
import pathlib
import re

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import rich
import rich.console
import seaborn as sns

DEFAULT_COMPARE_KEYS = [
    "frontend/object_detection",
    "gvd/extract_graph",
    "backend/room_detection",
]


def _get_time_array_from_log(filename):
    arr = np.genfromtxt(str(filename), delimiter=",", skip_header=1)
    if len(arr.shape) == 1:
        arr = arr.reshape(1, 2)

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


def _get_longform_df(durations, filter_func=None):
    names = []
    data = np.array([])
    for name, info in durations.items():
        if filter_func is not None and not filter_func(name):
            continue

        times = np.squeeze(info[:, 1])
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
    for stamp in info[timers[0]][:, 0]:
        curr_elapsed = []
        for timer in timers:
            diff = np.abs(info[timer][:, 0] - stamp)
            idx = np.argmin(diff)
            if diff[idx] > max_diff_ns:
                logger.warn(f"could not find stamp @ {stamp} [ns] for timer {timer}")
                logger.warn(f"best {info[timer][idx][0]} [ns] (diff {diff[idx]} [ns]")
                continue

            curr_elapsed.append(info[timer][idx, 1])

        if len(curr_elapsed) != len(timers):
            continue

        stamps.append(stamp)
        elapsed.append(sum(curr_elapsed))

    return np.array(stamps), np.array(elapsed)


def show_timing_info(data, key_regex=None):
    def _get_stat_str(stat):
        return rf"{1000 * stat:>.3f}"

    console = rich.console.Console()
    table = rich.table.Table(title="Timing Information")
    table.add_column("Timer")
    table.add_column(r"μ \[ms]")
    table.add_column(r"σ \[ms]")
    table.add_column(r"Min \[ms]")
    table.add_column(r"Max \[ms]")

    sorted_keys = sorted([key for key in data])
    if key_regex is not None:
        matcher = re.compile(key_regex)
        sorted_keys = [x for x in sorted_keys if matcher.match(x)]

    for key in sorted_keys:
        timing = data[key]
        mean = _get_stat_str(np.mean(timing[:, 1]))
        std = _get_stat_str(np.std(timing[:, 1]))
        v_min = _get_stat_str(np.min(timing[:, 1]))
        v_max = _get_stat_str(np.max(timing[:, 1]))
        table.add_row(key, mean, std, v_min, v_max)

    console.print(table)


def plot_durations(durations, keys):
    """Make a plot of durations."""
    sns.set()
    sns.set_style("whitegrid")
    sns.set_context("notebook")
    fig, ax = plt.subplots(len(keys), 1, squeeze=False)

    for idx, key in enumerate(keys):
        matcher = re.compile(key)
        ax[idx][0].set_title(f"{key} Timing Distributions")
        df = _get_longform_df(durations, lambda x: matcher.match(x))
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
            to_plot = time_array[time_array[:, 0] > 1.0]
            if to_plot.shape[0] < 2:
                continue

            ax[idx][0].plot(to_plot[:, 0], to_plot[:, 1], label=name)
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
    sns.set()
    sns.set_style("whitegrid")
    sns.set_context("notebook")

    data = np.array([])
    labels = []
    result_set = []

    keys = [x.replace("/", "_") for x in keys]
    for stem, result in results.items():
        for key in keys:
            if key not in result:
                continue

            num_values = len(result[key])
            data = np.hstack((data, np.squeeze(result[key][:, 1])))
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
