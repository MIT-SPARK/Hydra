"""Verify synchronous reconstruction and persistent mesh stitching."""

import hydra_python as hydra
import numpy as np
import pytest
import spark_dsg
import yaml


def make_pipeline(method, comparison=None):
    settings = {
        "default_num_threads": 1,
        "map_window": {"type": "spatial", "max_radius_m": 1.0},
        "reconstruction": {
            "volumetric_map": {"voxel_size": 0.1, "truncation_distance": 0.3},
            "mesh": {"integrator_threads": 1},
        },
        "mesh_compression": {"type": method, "resolution": 0.01},
    }
    settings["comparison_compression"] = {"type": "Uninitialized Virtual Config"}
    if comparison:
        settings["comparison_compression"] = {"type": comparison, "resolution": 0.01}
    hydra.init_config_context(["-c", yaml.safe_dump(settings)])
    camera = hydra.make_camera(20.0, 20.0, 9.5, 9.5, 20, 20)
    return hydra.ReconstructionPipeline(camera)


def step(pipeline, stamp, x):
    rgb = np.full((20, 20, 3), 128, dtype=np.uint8)
    depth = np.full((20, 20), 2.0, dtype=np.float32)
    return pipeline.step(stamp, [1.0, 0.0, 0.0, 0.0], [x, 0.0, 0.0], rgb, depth)


@pytest.mark.parametrize("method", ["MeshCompression", "DeltaCompression"])
def test_reconstruction_retains_archived_mesh(method, tmp_path):
    pipeline = make_pipeline(method)
    assert step(pipeline, 1_000_000_000, 0.0)
    first = pipeline.compression_stats[0]
    assert first.method == method
    assert first.mesh_vertices > 0
    assert first.mesh_faces > 0
    assert step(pipeline, 2_000_000_000, 10.0)
    second = pipeline.compression_stats[0]
    assert first.timestamp_ns == 1_000_000_000
    assert second.timestamp_ns == 2_000_000_000
    assert second.archived_vertices > 0
    assert second.mesh_vertices > first.mesh_vertices
    assert second.mesh_faces > first.mesh_faces
    assert second.mesh_vertices > second.active_vertices
    assert second.compression_ms >= 0.0
    assert pipeline.mesh.num_vertices() == second.mesh_vertices
    pipeline.save(tmp_path / method)
    saved = spark_dsg.Mesh.load(tmp_path / method / "mesh.sparkdsg")
    assert saved.num_vertices() == pipeline.mesh.num_vertices()
    assert saved.num_faces() == pipeline.mesh.num_faces()


def test_paired_compression_uses_same_packets_and_alternates_order():
    pipeline = make_pipeline("MeshCompression", "DeltaCompression")
    for index in range(3):
        stamp = (index + 1) * 1_000_000_000
        assert step(pipeline, stamp, 10.0 * index)
        new, old = pipeline.compression_stats
        assert new.timestamp_ns == old.timestamp_ns == stamp
        assert new.updated_blocks == old.updated_blocks
        assert new.archived_blocks == old.archived_blocks
        assert new.execution_order == index % 2
        assert old.execution_order == 1 - index % 2
        assert new.mesh_faces > 0
        assert old.mesh_faces > 0


def test_unwindowed_marching_cubes_without_compression(tmp_path):
    disabled = {"type": "Uninitialized Virtual Config"}
    hydra.init_config_context(
        [
            "-c",
            yaml.safe_dump(
                {
                    "default_num_threads": 1,
                    "map_window": disabled,
                    "mesh_compression": disabled,
                    "comparison_compression": disabled,
                    "reconstruction": {"mesh": {"integrator_threads": 1}},
                }
            ),
        ]
    )
    pipeline = hydra.ReconstructionPipeline(hydra.make_camera(20, 20, 9.5, 9.5, 20, 20))
    assert step(pipeline, 1_000_000_000, 0.0)
    first = pipeline.mesh
    assert first.num_faces() > 0
    assert step(pipeline, 2_000_000_000, 10.0)
    assert not pipeline.compression_stats
    mesh = pipeline.mesh
    assert mesh.num_faces() > first.num_faces()
    points = mesh.get_vertices()[:3].T
    assert points[:, 0].min() < 0
    assert points[:, 0].max() > 10
    pipeline.save(tmp_path / "reference")
    saved = spark_dsg.Mesh.load(tmp_path / "reference" / "mesh.sparkdsg")
    np.testing.assert_array_equal(saved.get_vertices(), mesh.get_vertices())
    np.testing.assert_array_equal(saved.get_faces(), mesh.get_faces())
