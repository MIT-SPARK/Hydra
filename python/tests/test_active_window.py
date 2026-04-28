"""Smoke tests for active-window map query bindings."""

import numpy as np
import pytest
import hydra_python as hydra
from hydra_python._hydra_bindings import HydraReconstruction, CameraConfig

# Minimal reconstruction config: no mesh viz, semantics enabled, k=2
_CONFIG = """
visualize_mesh: false
reconstruction:
  volumetric_map:
    voxel_size: 0.1
    voxels_per_side: 8
    with_semantics: true
  tsdf:
    semantic_integrator:
      type: FirstKSemanticIntegrator
      k: 2
      min_weight: 0.0
      max_weight: 0.0
label_space:
  total_labels: 4
label_names:
  0: unknown
  1: wall
  2: floor
  3: ceiling
"""

H, W = 32, 32
FX = FY = 200.0
CX, CY = W / 2.0, H / 2.0
DEPTH_M = 1.5  # flat wall at 1.5 m


@pytest.fixture(scope="module")
def reconstruction():
    sensor = CameraConfig(
        width=W, height=H,
        fx=FX, fy=FY,
        cx=CX, cy=CY,
        min_range=0.1, max_range=5.0,
    )
    recon = HydraReconstruction.from_config(_CONFIG, sensor)

    # Feed one frame: robot at origin, looking along +X
    depth = np.full((H, W), DEPTH_M, dtype=np.float32)
    labels = np.ones((H, W), dtype=np.int32)  # label 1 = "wall"
    pos = np.array([0.0, 0.0, 0.0])
    # identity quaternion: w, x, y, z
    quat = np.array([1.0, 0.0, 0.0, 0.0])

    recon.step(0, pos, quat, depth, labels, None)
    return recon


def test_map_accessible(reconstruction):
    m = reconstruction.map
    assert m is not None


def test_voxel_and_block_size(reconstruction):
    m = reconstruction.map
    assert abs(m.voxel_size - 0.1) < 1e-6
    assert abs(m.block_size - 0.8) < 1e-6  # voxel_size * voxels_per_side


def test_active_window_bounds_returns_tuple(reconstruction):
    result = reconstruction.map.get_active_window_bounds()
    assert result is not None, "No blocks allocated after feeding a frame"
    mn, mx = result
    assert mn.shape == (3,)
    assert mx.shape == (3,)
    assert np.all(mx > mn)


def test_active_block_indices_nonempty(reconstruction):
    indices = reconstruction.map.get_active_block_indices()
    assert len(indices) > 0
    # each element is a 3-tuple of ints
    bx, by, bz = indices[0]
    assert isinstance(bx, int)


def test_label_names(reconstruction):
    names = reconstruction.map.get_label_names()
    assert names[0] == "unknown"
    assert names[1] == "wall"


def test_voxel_semantics_at_observed_point(reconstruction):
    # The wall is ~1.5 m along X from origin — sample a voxel inside it
    pos = np.array([1.5, 0.0, 0.0], dtype=np.float32)
    result = reconstruction.map.get_voxel_semantics_at(pos)
    # May be None if the exact voxel wasn't hit, but shouldn't raise
    if result is not None:
        assert "mle_label" in result
        assert "weights" in result
        assert "label_ids" in result
        assert len(result["weights"]) == len(result["label_ids"])


def test_voxel_semantics_at_empty_returns_none(reconstruction):
    # Far from the observed scene
    pos = np.array([100.0, 100.0, 100.0], dtype=np.float32)
    result = reconstruction.map.get_voxel_semantics_at(pos)
    assert result is None
