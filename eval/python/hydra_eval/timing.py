"""Some small helpers for getting timing information."""

import numpy as np
import logging
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt


DEFAULT_FOLDERS = ["frontend", "backend", "lcd"]
FRONTEND_TIMERS = ["frontend/spin"]
BACKEND_TIMERS = ["backend/spin"]


def _get_filenames(result_path, subdir):
    if not (result_path / subdir).exists():
        return []
    return [f for f in (result_path / subdir).iterdir() if f.match("*timing_raw.csv")]


def _get_timer_name(path):
    return path.parent.name + "/" + path.stem[: -len("_timing_raw")]


def _get_time_array_from_log(filename):
    arr = np.genfromtxt(str(filename), delimiter=",", skip_header=1)
    if len(arr.shape) == 1:
        arr = arr.reshape(1, 2)

    return arr


def collect_timing_info(path, default_folders=None):
    """Collect timing info for a specfic path."""
    timing_files = []
    folders_to_use = DEFAULT_FOLDERS if default_folders is None else default_folders
    for folder in folders_to_use:
        timing_files += _get_filenames(path, folder)

    return {_get_timer_name(f): _get_time_array_from_log(f) for f in timing_files}


def compute_timing_statistics(timing_data, order):
    """Compute timing metrics for the dsg layers for a result directory."""
    means = np.full((1, len(order)), np.nan)
    stddevs = np.full((1, len(order)), np.nan)

    for idx, timer in enumerate(order):
        if timer in timing_data:
            values = timing_data[timer]
            times_to_use = np.squeeze(values) if values.shape[1] == 1 else values[:, 1]
            means[0, idx] = np.mean(times_to_use)
            stddevs[0, idx] = np.std(times_to_use)
        else:
            logger = logging.getLogger(__name__)
            logger.warning(f"Missing timer {timer} in timing data")

    return means, stddevs


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
                logger.warn(
                    f"best stamp @ {info[timer][idx][0]} [ns] (diff {diff[idx]} [ns]"
                )
                continue

            curr_elapsed.append(info[timer][idx, 1])

        if len(curr_elapsed) != len(timers):
            continue

        stamps.append(stamp)
        elapsed.append(sum(curr_elapsed))

    return np.array(stamps), np.array(elapsed)


def read_incremental_stamps(path, frontend_timers=None, backend_timers=None):
    """Read and collate incremental times."""
    timing_info = collect_timing_info(path)
    frontend_info = collate_timers(
        timing_info, FRONTEND_TIMERS if frontend_timers is None else frontend_timers
    )
    backend_info = collate_timers(
        timing_info, BACKEND_TIMERS if backend_timers is None else backend_timers
    )
    return frontend_info, backend_info


def read_batch_stamps(path, batch_name="batch_dsg", tsdf_name="tsdf_updates"):
    """Read and collate batch times."""
    batch_times = _get_time_array_from_log(path / f"{batch_name}_timing_raw.csv")
    tsdf_times = _get_time_array_from_log(path / f"{tsdf_name}_timing_raw.csv")

    stamps = batch_times[:, 0]
    elapsed = batch_times[:, 1]

    for i, stamp in enumerate(stamps):
        tsdf_total = tsdf_times[tsdf_times[:, 0] <= stamp, 1].sum()
        elapsed[i] += tsdf_total

    return stamps, elapsed


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
        part_name = name.split("/")[1]
        data = np.concatenate((data, times), axis=None)
        names += [part_name] * len(times)

    if len(names) == 0:
        return None

    return pd.DataFrame({"Name": names, "Elapsed Time [s]": data})


def plot_durations(durations, keys=["frontend", "backend"], rt_threshold=None):
    """Make a plot of durations."""
    sns.set()
    sns.set_style("whitegrid")
    sns.set_context("notebook")
    fig, ax = plt.subplots(len(keys), 1, squeeze=False)

    for idx, key in enumerate(keys):
        ax[idx][0].set_title(f"{key.capitalize()} Timing Distributions")
        df = _get_longform_df(durations, lambda x: key not in x)
        if df is None:
            continue

        sns.boxenplot(data=df, x="Name", y="Elapsed Time [s]", ax=ax[idx][0])
        lax = ax[idx][0]
        lax.set_xticks(lax.get_xticks(), lax.get_xticklabels(), rotation=30, ha="right")
        _draw_realtime_threshold(lax, rt_threshold)

    fig.set_size_inches([14, 12 * len(keys)])
    fig.tight_layout()
    plt.show()


def plot_trends(durations, keys=["frontend", "backend"], rt_threshold=None):
    """Make a plot of durations."""
    sns.set()
    sns.set_style("whitegrid")
    sns.set_context("notebook")

    fig, ax = plt.subplots(len(keys), 1, squeeze=False)

    for idx, key in enumerate(keys):
        added_plot = False
        for name, time_array in durations.items():
            if key not in name:
                continue

            assert len(time_array.shape) == 2 and time_array.shape[1] == 2
            to_plot = time_array[time_array[:, 0] > 1.0]
            if to_plot.shape[0] < 2:
                continue

            ax[idx][0].plot(to_plot[:, 0], to_plot[:, 1], label=name)
            added_plot = True

        ax[idx][0].set_title(f"{key.capitalize()} Timing Trends")
        ax[idx][0].set_xlabel("Timestamp [s]")
        ax[idx][0].set_ylabel("Elapsed Time [s]")
        if added_plot:
            _draw_realtime_threshold(ax[idx][0], rt_threshold)
            ax[idx][0].legend()

    fig.set_size_inches([16, 8 * len(keys)])
    plt.show()


def plot_comparison(results, plot_config, keys, use_bars=False, rt_threshold=None):
    """Plot timing comparison for different results directories."""
    data = np.array([])
    labels = []
    result_set = []

    for stem, result in results.items():
        result_label = plot_config.get("result_map", {}).get(stem, stem.upper())
        for key in keys:
            if key not in result:
                continue

            result_df = _get_longform_df(result, lambda x: key in x)
            # TODO(nathan) remap key to actual name
            # TODO(nathan) add result key
            key_label = plot_config.get("key_map", {}).get(key, key)

            num_values = len(result[key])
            labels += num_values * [key_label]
            result_set += num_values * [result_label]

    sns.set()
    sns.set_style("white")
    sns.set_context("poster", font_scale=1.7, rc={"lines.linewidth": 2.0})

    fig, ax = plt.subplots()
    value_key = "Elapsed Time [s]"
    timer_key = plot_config.get("xlabel", "Timer Name")
    result_key = "Result Name"

    df = pd.DataFrame({value_key: data, timer_key: labels, result_key: result_set})
    if use_bars:
        sns.barplot(x=timer_key, y=value_key, hue=result_key, data=df)
    else:
        sns.boxenplot(x=timer_key, y=value_key, hue=result_key, data=df)

    if "title" in plot_config:
        ax.set_title(plot_config["title"])

    _draw_realtime_threshold(ax, rt_threshold)
    ax.legend()

    sns.despine()
    fig.set_size_inches([14, 12])
    fig.tight_layout()
    plt.show()
