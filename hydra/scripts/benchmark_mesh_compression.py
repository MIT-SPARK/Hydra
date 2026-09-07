#!/usr/bin/env python3
"""Replay a rosbag synchronously and compare background mesh compression.

Reconstruction runs once per input frame. C++ steady-clock measurements include
compression and archival, but exclude reconstruction, Python, bag I/O, and mesh
application. Mesh application is measured separately. Paired mode alternates the
execution order each update and keeps independent accumulated meshes.

Example (uHumans2 office):
  benchmark_mesh_compression.py BAG --output results --max-steps 300 \
      --rgb-topic /tesse/left_cam/rgb/image_raw \
      --depth-topic /tesse/depth_cam/mono/image_raw \
      --camera-info-topic /tesse/left_cam/camera_info \
      --odom-frame world --body-frame base_link_gt

Use --help for recording-specific topics, frames, configuration, and sampling.
No frontend, backend, ROS spin, or visualization modules are started.
"""

import csv
import json
import pathlib
import platform
import sys

import click
import hydra_python as hydra
import numpy as np
import yaml
from hydra_python.dataloaders.rosbag_dataloader import (
    RosbagDataLoader,
    load_trajectory_from_bag,
)
from hydra_python.trajectory import Trajectory
from ianvs.bag_reader import BagReader

STATS_FIELDS = (
    "method",
    "timestamp_ns",
    "execution_order",
    "compression_ms",
    "mesh_update_ms",
    "updated_blocks",
    "archived_blocks",
    "active_vertices",
    "archived_vertices",
    "active_faces",
    "archived_faces",
    "mesh_vertices",
    "mesh_faces",
)
CSV_FIELDS = ("run", "frame", "warmup", "reconstruction_ms", *STATS_FIELDS)


def configuration_args(files, snippets, variables, options):
    """Supply benchmark defaults before user overrides and enforce no sinks."""
    defaults = {
        "default_num_threads": options["threads"],
        "timing_disabled": True,
        "disable_timer_output": True,
        "map_window": {"type": "spatial", "max_radius_m": options["window_radius"]},
        "mesh_compression": {
            "type": "MeshCompression",
            "resolution": options["resolution"],
        },
        "comparison_compression": {
            "type": "DeltaCompression",
            "resolution": options["resolution"],
        },
        "reconstruction": {
            "volumetric_map": {
                "voxel_size": options["voxel_size"],
                "truncation_distance": 3 * options["voxel_size"],
                "with_semantics": False,
                "with_tracking": False,
            },
            "full_update_separation_s": 0.0,
            "mesh": {"integrator_threads": options["threads"]},
            "tsdf": {"sensors": {"num_threads": options["threads"]}},
        },
    }
    if options["method"] != "both":
        defaults.pop("comparison_compression")
        if options["method"] == "old":
            defaults["mesh_compression"]["type"] = "DeltaCompression"

    args = ["-c", yaml.safe_dump(defaults)]
    for flag, values in (("-f", files), ("-c", snippets), ("-v", variables)):
        for value in values:
            args.extend([flag, value])

    args.extend(["-c", "{app_plugins: [], reconstruction: {sinks: []}}"])
    return args


def make_loader(bag, options):
    """Load poses in the requested body frame and align them to RGB timestamps."""
    trajectory_path = options["trajectory_path"]
    if trajectory_path:
        trajectory = Trajectory.from_csv(trajectory_path)
    else:
        trajectory = load_trajectory_from_bag(
            bag, options["odom_frame"], options["body_frame"], progress=False
        )
    if not len(trajectory):
        raise click.ClickException(
            "No trajectory poses found; check frames or --trajectory-path"
        )

    return RosbagDataLoader(
        bag,
        options["rgb_topic"],
        trajectory=trajectory,
        other_topics=[options["depth_topic"]],
        body_frame=options["body_frame"],
        rgb_info_topic=options["camera_info_topic"],
        threshold_us=options["sync_tolerance_us"],
        start_time_ns=int(options["bag_start_s"] * 1.0e9),
        progress=False,
    )


def input_frames(loader, options):
    """Apply identical frame selection before feeding either compressor."""
    last_stamp = None
    count = 0
    threshold = int(options["min_separation_s"] * 1.0e9)
    for stamp, pose, images in loader:
        if options["max_steps"] is not None and count >= options["max_steps"]:
            break
        if last_stamp is not None and stamp - last_stamp < threshold:
            continue
        rgb = images[0]
        if options["color_order"] == "bgr":
            rgb = rgb[..., ::-1]
        depth = np.asarray(images[1], dtype=np.float32) * options["depth_scale"]
        rotation = pose.rotation.as_quat()[[3, 0, 1, 2]]
        yield stamp, rotation, pose.translation, np.ascontiguousarray(rgb), depth
        last_stamp = stamp
        count += 1


def run_trial(loader, camera, run, writer, output, options):
    pipeline = hydra.ReconstructionPipeline(camera)
    (output / "resolved_reconstruction.txt").write_text(pipeline.config)
    rows = []
    updates = 0
    for frame, packet in enumerate(input_frames(loader, options)):
        if not pipeline.step(*packet):
            continue
        for stats in pipeline.compression_stats:
            row = {name: getattr(stats, name) for name in STATS_FIELDS}
            row.update(
                run=run,
                frame=frame,
                warmup=updates < options["warmup_steps"],
                reconstruction_ms=pipeline.reconstruction_ms,
            )
            writer.writerow(row)
            rows.append(row)
        updates += 1
        if updates % 50 == 0:
            click.echo(f"Run {run + 1}: {updates} reconstruction updates", err=True)
    if options["save_mesh"]:
        pipeline.save(output / f"run_{run}")
    if not rows:
        raise click.ClickException(
            "No mesh updates produced; check topics, poses, and configuration"
        )
    return rows


def timing_summary(values):
    values = np.asarray(values)
    return {
        "mean_ms": float(np.mean(values)),
        "median_ms": float(np.median(values)),
        "p95_ms": float(np.percentile(values, 95)),
        "p99_ms": float(np.percentile(values, 99)),
        "max_ms": float(np.max(values)),
        "total_ms": float(np.sum(values)),
    }


def summarize(rows):
    result = {}
    for method in sorted({row["method"] for row in rows}):
        selected = [r for r in rows if r["method"] == method and not r["warmup"]]
        if not selected:
            raise click.ClickException("No measured updates remain after warmup")
        result[method] = {
            "samples": len(selected),
            "compression": timing_summary([r["compression_ms"] for r in selected]),
            "mesh_update": timing_summary([r["mesh_update_ms"] for r in selected]),
            "compression_and_mesh_update": timing_summary(
                [r["compression_ms"] + r["mesh_update_ms"] for r in selected]
            ),
            "final_mesh_vertices": selected[-1]["mesh_vertices"],
            "final_mesh_faces": selected[-1]["mesh_faces"],
            "archived_vertices": sum(r["archived_vertices"] for r in selected),
            "updates_with_archival": sum(r["archived_blocks"] > 0 for r in selected),
        }
    if "DeltaCompression" in result and "MeshCompression" in result:
        old = result["DeltaCompression"]["compression"]["mean_ms"]
        new = result["MeshCompression"]["compression"]["mean_ms"]
        result["old_over_new_mean_compression_ratio"] = old / new if new else None
    return result


@click.command(context_settings={"show_default": True})
@click.argument("bag_path", type=click.Path(exists=True, path_type=pathlib.Path))
@click.option("--output", required=True, type=click.Path(path_type=pathlib.Path))
@click.option(
    "--trajectory-path", "-t", type=click.Path(exists=True, path_type=pathlib.Path)
)
@click.option("--rgb-topic", required=True)
@click.option("--depth-topic", required=True)
@click.option("--camera-info-topic", default=None)
@click.option("--odom-frame", default="world")
@click.option("--body-frame", default="base_link")
@click.option("--max-steps", "-m", type=click.IntRange(min=1), default=None)
@click.option("--min-separation-s", type=click.FloatRange(min=0), default=0.2)
@click.option("--bag-start-s", type=click.FloatRange(min=0), default=0.0)
@click.option("--sync-tolerance-us", type=click.IntRange(min=0), default=10000)
@click.option(
    "--depth-scale",
    type=click.FloatRange(min=0, min_open=True),
    default=1.0,
    help="Convert depth samples to meters; use 0.001 for millimeters.",
)
@click.option("--min-range", type=click.FloatRange(min=0), default=0.1)
@click.option("--max-range", type=click.FloatRange(min=0, min_open=True), default=5.0)
@click.option("--color-order", type=click.Choice(["rgb", "bgr"]), default="rgb")
@click.option("--method", type=click.Choice(["both", "old", "new"]), default="both")
@click.option(
    "--resolution", type=click.FloatRange(min=0, min_open=True), default=0.005
)
@click.option("--voxel-size", type=click.FloatRange(min=0, min_open=True), default=0.1)
@click.option(
    "--window-radius", type=click.FloatRange(min=0, min_open=True), default=8.0
)
@click.option("--threads", type=click.IntRange(min=1), default=1)
@click.option("--warmup-steps", type=click.IntRange(min=0), default=10)
@click.option("--repetitions", type=click.IntRange(min=1), default=1)
@click.option("--save-mesh", is_flag=True)
@click.option("--config-utilities-files", "-f", multiple=True)
@click.option("--config-utilities-yaml", "-c", multiple=True)
@click.option("--config-utilities-var", "-v", multiple=True)
def run(
    bag_path,
    output,
    config_utilities_files,
    config_utilities_yaml,
    config_utilities_var,
    **options,
):
    """Compare compressors using identical updates reconstructed from BAG_PATH."""
    output.mkdir(parents=True, exist_ok=False)
    args = configuration_args(
        config_utilities_files, config_utilities_yaml, config_utilities_var, options
    )
    hydra.set_glog_level(2, 0)
    hydra.init_config_context(args)
    metadata = {
        "bag_path": str(bag_path.resolve()),
        "options": options,
        "config_args": args,
        "platform": platform.platform(),
        "python": sys.version,
        "clock": "C++ std::chrono::steady_clock",
        "timing_scope": "compressor update including archival; mesh application separate",
    }
    (output / "metadata.json").write_text(json.dumps(metadata, indent=2, default=str))
    rows = []
    with BagReader(bag_path) as bag, (output / "frames.csv").open("w") as fout:
        loader = make_loader(bag, options)
        camera = hydra.make_camera(
            **loader.intrinsics,
            min_range=options["min_range"],
            max_range=options["max_range"],
        )
        writer = csv.DictWriter(fout, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for repeat in range(options["repetitions"]):
            rows.extend(run_trial(loader, camera, repeat, writer, output, options))
            fout.flush()
    summary = summarize(rows)
    (output / "summary.json").write_text(json.dumps(summary, indent=2))
    click.echo(json.dumps(summary, indent=2))


if __name__ == "__main__":
    run()
