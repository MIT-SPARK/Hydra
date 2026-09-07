#!/usr/bin/env python3
"""Locate ground samples lost during selected compression updates.

Use benchmark_mesh_compression.py --debug-frame for every update in a short
segment. Saved masks distinguish loss within an update from reference samples
not covered by any inspected snapshot. Missing snapshots are not observations.
Requires the same dependencies as evaluate_mesh_compression.py.
"""

import json
import pathlib

import click
import numpy as np
from evaluate_mesh_compression import (
    ground_surface,
    load_mesh,
    sample_surface,
    surface_distances,
)


def coverage(path, points, threshold):
    import spark_dsg

    raw = spark_dsg.Mesh.load(path)
    if not raw.num_faces():
        return np.zeros(len(points), dtype=bool)
    mesh, _ = load_mesh(path)
    return surface_distances(mesh, points, batch_size=64) <= threshold


@click.command()
@click.argument(
    "reference", type=click.Path(exists=True, dir_okay=False, path_type=pathlib.Path)
)
@click.argument(
    "snapshots", type=click.Path(exists=True, file_okay=False, path_type=pathlib.Path)
)
@click.option("--method", default="MeshCompression")
@click.option("--ground-height", type=float, required=True)
@click.option("--ground-band", type=click.FloatRange(min=0, min_open=True), default=0.1)
@click.option("--samples", type=click.IntRange(min=1), default=10000)
@click.option("--threshold", type=click.FloatRange(min=0, min_open=True), default=0.02)
@click.option("--output", type=click.Path(path_type=pathlib.Path), required=True)
def run(
    reference, snapshots, method, ground_height, ground_band, samples, threshold, output
):
    mesh, _ = load_mesh(reference)
    points = sample_surface(
        ground_surface(mesh, ground_height, ground_band), samples, 0
    )
    output.mkdir(parents=True, exist_ok=False)
    seen = np.zeros(samples, dtype=bool)
    rows = []
    frames = sorted(
        (p for p in snapshots.iterdir() if p.name.isdigit()), key=lambda p: int(p.name)
    )
    for frame in frames:
        before = coverage(frame / method / "before.sparkdsg", points, threshold)
        incoming = coverage(frame / "incoming.sparkdsg", points, threshold)
        after = coverage(frame / method / "after.sparkdsg", points, threshold)
        seen |= before | after
        lost = before & ~after
        gained = ~before & after
        rows.append(
            {
                "update": int(frame.name),
                "lost": int(lost.sum()),
                "gained": int(gained.sum()),
                "covered": int(after.sum()),
                "not_seen_in_snapshots": int((~seen).sum()),
            }
        )
        np.savez_compressed(
            output / f"{frame.name}.npz",
            points=points,
            before=before,
            incoming=incoming,
            after=after,
            lost=lost,
            gained=gained,
        )
    result = {
        "samples": samples,
        "threshold_m": threshold,
        "ground_height_m": ground_height,
        "method": method,
        "updates": rows,
    }
    (output / "history.json").write_text(json.dumps(result, indent=2))
    click.echo(json.dumps(result, indent=2))


if __name__ == "__main__":
    run()
