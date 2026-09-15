"""Structured evaluation results exposed through the main binding module."""

import numpy as np
import pytest
from hydra_python import eval as evaluation


def test_room_scores():
    """Voxel sets map to structured metrics with stable overlap ordering."""
    result = evaluation.score_rooms(
        {7: {(0, 0, 0), (1, 0, 0)}}, {9: {(1, 0, 0), (2, 0, 0)}}
    )
    assert result.is_valid
    assert result.total_recall == pytest.approx(0.5)
    assert result.total_precision == pytest.approx(0.5)
    assert result.gt_sizes == [2]
    assert result.est_sizes == [2]
    np.testing.assert_array_equal(result.overlaps, [[1.0]])


def test_empty_room_scores():
    """Empty comparisons produce zero scores rather than native assertions."""
    result = evaluation.score_rooms({}, {})
    assert not result.is_valid
    assert result.total_recall == 0
    assert result.total_precision == 0
    assert result.overlaps.shape == (0, 0)
    result = evaluation.score_rooms({0: {(0, 0, 0)}}, {})
    assert result.recalls == [0.0]
    assert result.overlaps.shape == (1, 0)


def test_room_annotations():
    """Ground truth annotation geometry is available without subprocesses."""
    rooms = evaluation.RoomGeometry.from_yaml("""
3:
  - {center: [0, 0, 0], extents: [2, 2, 2], rotation: {w: 1, x: 0, y: 0, z: 0}}
""")
    assert rooms.get_room_ids() == [3]
    assert rooms.find_room_index(np.zeros(3, dtype=np.float32)) == 3
    assert rooms.find_room_index(np.ones(3, dtype=np.float32) * 10) is None


@pytest.fixture
def ground_truth(tmp_path):
    """One observed voxel centered at (0.5, 0.5, 0.5), with zero distance."""
    from pathlib import Path

    tsdf = Path(__file__).parent / "resources" / "eval_tsdf.layer"
    rooms = tmp_path / "rooms.yaml"
    rooms.write_text("""
0:
  - {center: [0.5, 0.5, 0.5], extents: [1, 1, 1], rotation: {w: 1, x: 0, y: 0, z: 0}}
""")
    config = tmp_path / "gvd.yaml"
    config.write_text("use_tsdf_for_surface: true\nmin_distance_m: 1.0\n")
    return str(tsdf), str(rooms), str(config)


def test_evaluators_reuse_ground_truth(ground_truth, tmp_path):
    """Native evaluators accept Spark DSG graphs repeatedly and agree with file APIs."""
    import spark_dsg as dsg

    tsdf, rooms, config = ground_truth
    room_eval = evaluation.RoomEvaluator.from_file(
        evaluation.RoomEvaluatorConfig(), rooms, tsdf
    )
    place_eval = evaluation.PlaceEvaluator.from_file(config, tsdf, max_distance_m=3.0)
    assert room_eval is not None
    assert place_eval is not None
    graph = dsg.SceneGraph()
    place = dsg.NodeSymbol("p", 0)
    room = dsg.NodeSymbol("R", 0)
    attrs = dsg.PlaceNodeAttributes()
    attrs.position = np.full(3, 0.5)
    attrs.distance = 0.0
    assert graph.add_node(dsg.DsgLayers.PLACES, place, attrs)
    assert graph.add_node(dsg.DsgLayers.ROOMS, room, dsg.RoomNodeAttributes())
    assert graph.insert_edge(room, place)
    graph_path = str(tmp_path / "graph.json")
    graph.save(graph_path)

    for _ in range(2):
        scores = room_eval.eval(graph)
        assert scores.total_recall == pytest.approx(1.0)
        assert scores.total_precision == pytest.approx(1.0)
        assert room_eval.eval_file(graph_path).total_recall == scores.total_recall
        place_scores = place_eval.eval(graph, min_basis=0)
        assert place_scores.is_valid
        assert place_scores.num_valid == 1
        assert place_scores.node_order == [place.value]
        assert place_scores.gvd_distance_errors == [0.0]
        assert place_eval.eval_file(graph_path, min_basis=0).num_valid == 1

    assert not place_eval.eval(graph, min_basis=0, layer_id="missing").is_valid
    graph.remove_node(place)
    assert room_eval.eval(graph).total_recall == 0.0
    assert len(room_eval.get_room_indices()[0]) == 1
