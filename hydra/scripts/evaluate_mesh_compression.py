#!/usr/bin/env python3
"""Compare compressed meshes to an unwindowed marching-cubes reference.

Requires spark_dsg, numpy, scipy, trimesh, and rtree. Sample surfaces uniformly by
triangle area, then measure exact distance to the other triangle surface (not its
vertices). No alignment, cropping, or distance clipping is applied. The reference
is a reconstruction baseline, not independently measured physical ground truth.
"""

import json
import pathlib

import click
import numpy as np
import spark_dsg
import trimesh


def load_mesh(path):
    """Weld exact duplicates and remove duplicate/zero-area faces for evaluation."""
    source = spark_dsg.Mesh.load(path)
    vertices, inverse = np.unique(
        source.get_vertices()[:3].T, axis=0, return_inverse=True
    )
    faces = inverse[source.get_faces().T]
    _, indices = np.unique(np.sort(faces, axis=1), axis=0, return_index=True)
    mesh = trimesh.Trimesh(vertices, faces[np.sort(indices)], process=False)
    valid = mesh.area_faces > 0
    zero_area = int(np.count_nonzero(~valid))
    mesh.update_faces(valid)
    mesh.remove_unreferenced_vertices()
    if not len(mesh.faces):
        raise ValueError(f"No nondegenerate triangles in {path}")
    counts = np.bincount(mesh.edges_unique_inverse)
    geometry = {
        "input_vertices": source.num_vertices(),
        "input_faces": source.num_faces(),
        "unique_surface_vertices": len(mesh.vertices),
        "unique_surface_faces": len(mesh.faces),
        "duplicate_faces": source.num_faces() - len(indices),
        "zero_area_faces": zero_area,
        "surface_area_m2": float(mesh.area),
        "boundary_edges": int(np.count_nonzero(counts == 1)),
        "nonmanifold_edges": int(np.count_nonzero(counts > 2)),
    }
    return mesh, geometry


def check_timestamps(reference_path, candidate_path):
    """Reject mismatched reconstruction updates when benchmark manifests exist."""
    manifests = [
        path.parent.parent / f"{path.parent.name}_timestamps.npy"
        for path in (reference_path, candidate_path)
    ]
    if not all(path.is_file() for path in manifests):
        return None
    reference, candidate = (np.load(path) for path in manifests)
    if not np.array_equal(reference, candidate):
        raise click.ClickException(
            f"Different reconstruction timestamps: {candidate_path}"
        )
    return len(reference)


def sample_surface(mesh, count, seed):
    """Use deterministic, area-weighted uniform barycentric samples."""
    rng = np.random.default_rng(seed)
    indices = rng.choice(len(mesh.faces), count, p=mesh.area_faces / mesh.area)
    triangles = mesh.triangles[indices]
    uv = rng.random((count, 2))
    reflect = uv.sum(axis=1) > 1
    uv[reflect] = 1 - uv[reflect]
    return (
        triangles[:, 0]
        + uv[:, :1] * (triangles[:, 1] - triangles[:, 0])
        + uv[:, 1:] * (triangles[:, 2] - triangles[:, 0])
    )


def surface_distances(mesh, points, batch_size=1000):
    """Bound temporary candidate storage while querying triangle surfaces."""
    result = []
    for offset in range(0, len(points), batch_size):
        _, distances, _ = trimesh.proximity.closest_point(
            mesh, points[offset : offset + batch_size]
        )
        result.append(distances)
    return np.concatenate(result)


def distance_summary(distances):
    return {
        "rmse_m": float(np.sqrt(np.mean(distances**2))),
        "mean_m": float(np.mean(distances)),
        "median_m": float(np.median(distances)),
        "p95_m": float(np.percentile(distances, 95)),
        "max_m": float(np.max(distances)),
    }


def compare_meshes(reference, candidate, reference_points, count, seed, thresholds):
    accuracy = surface_distances(reference, sample_surface(candidate, count, seed))
    completeness = surface_distances(candidate, reference_points)
    scores = {}
    for threshold in thresholds:
        precision = float(np.mean(accuracy <= threshold))
        recall = float(np.mean(completeness <= threshold))
        total = precision + recall
        scores[str(threshold)] = {
            "precision": precision,
            "completeness": recall,
            "f1": 2 * precision * recall / total if total else 0.0,
        }
    return {
        "candidate_to_reference": distance_summary(accuracy),
        "reference_to_candidate": distance_summary(completeness),
        "symmetric_rmse_m": float(
            np.sqrt((np.mean(accuracy**2) + np.mean(completeness**2)) / 2)
        ),
        "thresholds_m": scores,
    }


@click.command(context_settings={"show_default": True})
@click.argument(
    "reference_path",
    type=click.Path(exists=True, dir_okay=False, path_type=pathlib.Path),
)
@click.option(
    "--candidate",
    type=(str, click.Path(exists=True, dir_okay=False, path_type=pathlib.Path)),
    multiple=True,
    required=True,
    help="NAME PATH for each candidate mesh.",
)
@click.option("--output", required=True, type=click.Path(path_type=pathlib.Path))
@click.option("--samples", default=100000, type=click.IntRange(min=1))
@click.option("--seed", default=0, type=int)
@click.option(
    "--threshold",
    "thresholds",
    multiple=True,
    default=(0.01, 0.02, 0.05, 0.1),
    type=click.FloatRange(min=0, min_open=True),
)
def run(reference_path, candidate, output, samples, seed, thresholds):
    """Evaluate candidate meshes in the same world frame as REFERENCE_PATH."""
    reference, geometry = load_mesh(reference_path)
    reference_points = sample_surface(reference, samples, seed)
    result = {
        "reference": str(reference_path.resolve()),
        "reference_geometry": geometry,
        "samples_per_direction": samples,
        "seed": seed,
        "distance": "area-weighted surface samples to closest triangle, meters",
        "trimesh_version": trimesh.__version__,
        "candidates": {},
    }
    for name, path in candidate:
        click.echo(f"Evaluating {name}", err=True)
        matching_updates = check_timestamps(reference_path, path)
        mesh, geometry = load_mesh(path)
        metrics = compare_meshes(
            reference, mesh, reference_points, samples, seed, thresholds
        )
        result["candidates"][name] = {
            "path": str(path.resolve()),
            "matching_reconstruction_updates": matching_updates,
            "geometry": geometry,
            **metrics,
        }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2))
    click.echo(json.dumps(result, indent=2))


if __name__ == "__main__":
    run()
