"""Check mesh-quality measurements against analytic triangle surfaces."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest

trimesh = pytest.importorskip("trimesh")
pytest.importorskip("rtree")
script = Path(__file__).parents[2] / "scripts" / "evaluate_mesh_compression.py"
spec = importlib.util.spec_from_file_location("mesh_quality", script)
quality = importlib.util.module_from_spec(spec)
spec.loader.exec_module(quality)


def square():
    return trimesh.Trimesh(
        [[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]],
        [[0, 1, 2], [0, 2, 3]],
        process=False,
    )


def test_distance_is_to_triangle_surface():
    distances = quality.surface_distances(square(), [[0.25, 0.25, 2], [2, 0.5, 0]])
    np.testing.assert_allclose(distances, [2, 1], atol=1e-12)


def test_parallel_surfaces_have_known_rmse():
    reference = square()
    candidate = square()
    candidate.vertices[:, 2] += 0.1
    points = quality.sample_surface(reference, 100, 0)
    metrics = quality.compare_meshes(reference, candidate, points, 100, 0, [0.05, 0.2])
    assert metrics["symmetric_rmse_m"] == pytest.approx(0.1)
    assert metrics["thresholds_m"]["0.05"]["completeness"] == 0.0
    assert metrics["thresholds_m"]["0.2"]["completeness"] == 1.0


def test_missing_half_surface_reduces_completeness():
    reference = square()
    candidate = square()
    candidate.update_faces([True, False])
    points = quality.sample_surface(reference, 10000, 1)
    metrics = quality.compare_meshes(reference, candidate, points, 1000, 1, [1e-8])
    scores = metrics["thresholds_m"]["1e-08"]
    assert scores["precision"] == 1.0
    assert scores["completeness"] == pytest.approx(0.5, abs=0.02)


def test_exact_vertex_welding_preserves_surface(tmp_path):
    import spark_dsg

    mesh = spark_dsg.Mesh(False, False, False, False)
    vertices = np.zeros((6, 6))
    vertices[:3] = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]] * 2).T
    mesh.set_vertices(vertices)
    mesh.set_faces(np.array([[0, 1, 2], [5, 4, 3]]).T)
    path = tmp_path / "duplicates.sparkdsg"
    mesh.save(path)
    surface, stats = quality.load_mesh(path)
    assert len(surface.vertices) == 3
    assert len(surface.faces) == 1
    assert stats["duplicate_faces"] == 1
    assert surface.area == pytest.approx(0.5)
