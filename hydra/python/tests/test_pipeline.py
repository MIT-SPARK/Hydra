"""Run isolation and artifact destinations through the Python pipeline."""

import json

import hydra_python as hydra
import numpy as np
import pytest
import spark_dsg as dsg
import yaml

pytestmark = pytest.mark.skipif(
    hydra.BINDINGS_ERROR is not None, reason="Hydra bindings are not built"
)


def make_pipeline(
    path, robot_id, label, name, *, log_keyframes=False, num_threads=1, labelspace=None
):
    hydra.init_config_context(
        [
            "hydra",
            "--config-utilities-yaml",
            json.dumps(
                {
                    "robot_id": robot_id,
                    "log_path": "" if path is None else str(path),
                    **(
                        {"default_num_threads": num_threads}
                        if num_threads is not None
                        else {}
                    ),
                    "default_verbosity": 0,
                    "labelspace": {
                        "type": "from_config",
                        "total_semantic_labels": label + 1,
                        "invalid_labels": [0],
                        "label_names": [{"label": label, "name": name}],
                    },
                    "active_window": {
                        "tsdf": {
                            "semantic_integrator": {"type": "MLESemanticIntegrator"}
                        }
                    },
                    "frontend": {
                        "enable_mesh_objects": False,
                        "pose_graph_tracker": {"min_pose_separation": 0.0},
                        "keyframe_selector": {
                            "type": "KeyframeSelector",
                            "log_keyframes": log_keyframes,
                        },
                    },
                    "backend": {
                        "pgmo": {
                            "covariance": {
                                "odom": 0.01,
                                "loop_close": 0.05,
                                "prior": 0.01,
                                "mesh_mesh": 0.01,
                                "pose_mesh": 0.01,
                            }
                        }
                    },
                }
            ),
        ]
    )
    context = hydra.RunContext(labelspace=labelspace)
    sensor = hydra.make_camera(40.0, 40.0, 23.5, 23.5, 48, 48)
    return hydra.HydraPipeline(sensor, context, show_config=False), context


def step(pipeline, label, stamp=1_000_000_000):
    return pipeline.step(
        stamp,
        np.array([1.0, 0.0, 0.0, 0.0]),
        np.zeros(3),
        np.full((48, 48, 3), 127, dtype=np.uint8),
        np.full((48, 48), 2.0, dtype=np.float32),
        np.full((48, 48), label, dtype=np.int32),
    )


def saved_mesh(path):
    graph = dsg.DynamicSceneGraph.load(str(path / "frontend/dsg_with_mesh.json"))
    mesh = graph.mesh
    assert mesh is not None
    assert mesh.num_vertices() > 0
    return mesh


@pytest.mark.parametrize("python_labelspace", [False, True])
def test_sequential_labelspaces(tmp_path, python_labelspace):
    """Different runs use their own semantic dimensions, names, and robot IDs."""
    results = []
    contexts = []
    for index, (robot_id, label, name) in enumerate(
        [(0, 1, "wall"), (2, 4, "chair"), (0, 1, "wall")]
    ):
        path = tmp_path / str(index)
        expected_name = f"model_{name}" if python_labelspace else name
        expected_total = label + (2 if python_labelspace else 1)
        if index == 2:
            context = contexts[0]
            sensor = hydra.make_camera(40.0, 40.0, 23.5, 23.5, 48, 48)
            pipeline = hydra.HydraPipeline(sensor, context, show_config=False)
        else:
            labels = None
            if python_labelspace:
                labels = hydra.Labelspace()
                labels.total_labels = expected_total
                labels.invalid_labels = {0}
                labels.label_names = {label: expected_name}

            configured_label = 0 if python_labelspace else label
            pipeline, context = make_pipeline(
                path, robot_id, configured_label, name, labelspace=labels
            )
            contexts.append(context)
            if labels is not None:
                labels.total_labels = 1
                labels.label_names = {}

        assert context.robot_id == robot_id
        assert context.labelspace.total_labels == expected_total
        assert context.labelspace.label_names == {label: expected_name}
        assert step(pipeline, label)
        assert step(pipeline, label, 2_000_000_000)
        pipeline.stop()
        pipeline.save(path)
        graph = dsg.DynamicSceneGraph.load(str(path / "frontend/dsg_with_mesh.json"))
        agents = list(graph.get_layer(2, ord("a") + robot_id).nodes)
        assert agents
        assert {node.id.category for node in agents} == {chr(ord("a") + robot_id)}
        mesh = saved_mesh(path)
        assert set(mesh.get_labels()) == {label}
        results.append(mesh.num_vertices())
        with (path / "hydra_config.yaml").open() as stream:
            config = yaml.safe_load(stream)
        assert config["robot_id"] == robot_id
        assert config["labelspace"]["total_semantic_labels"] == expected_total
        assert config["labelspace"]["label_names"] == [
            {"label": label, "name": expected_name}
        ]
        del pipeline

    assert results[0] == results[2]


def test_save_override(tmp_path):
    """A save override leaves the configured run directory as the default."""
    base, override = tmp_path / "base", tmp_path / "override"
    pipeline, _ = make_pipeline(base, 0, 1, "wall", log_keyframes=True)
    assert step(pipeline, 1)
    pipeline.stop()
    pipeline.save(override)
    assert not (base / "hydra_config.yaml").exists()
    assert (base / "frontend/keyframes/1000000000.zip").is_file()
    assert not (override / "frontend/keyframes").exists()
    override_mesh = saved_mesh(override)
    pipeline.save()
    assert saved_mesh(base).num_vertices() == override_mesh.num_vertices()


def test_threaded_run_and_explicit_output(tmp_path):
    """Draining a threaded run saves its mesh even without a base log directory."""
    pipeline, _ = make_pipeline(None, 1, 2, "wall", log_keyframes=True)
    pipeline.start()
    step(pipeline, 2)
    pipeline.stop()
    pipeline.save()
    pipeline.save(tmp_path)
    assert set(saved_mesh(tmp_path).get_labels()) == {2}
    assert not (tmp_path / "frontend/keyframes").exists()


def test_reconstruction_restores_context_settings(tmp_path):
    """A reconstruction run restores its context before later runs inherit defaults."""
    contexts = []
    for threads in (1, 2):
        hydra.init_config_context(
            ["hydra", "-c", json.dumps({"default_num_threads": threads})]
        )
        contexts.append(hydra.RunContext())

    sensor = hydra.make_camera(40.0, 40.0, 23.5, 23.5, 48, 48)
    reconstruction = hydra.ReconstructionPipeline(sensor, contexts[0])
    assert reconstruction.step(
        1_000_000_000,
        np.array([1.0, 0.0, 0.0, 0.0]),
        np.zeros(3),
        np.full((48, 48, 3), 127, dtype=np.uint8),
        np.full((48, 48), 2.0, dtype=np.float32),
    )
    assert reconstruction.mesh.num_vertices() > 0
    del reconstruction

    pipeline, _ = make_pipeline(tmp_path, 0, 1, "wall", num_threads=None)
    pipeline.stop()
    pipeline.save()
    with (tmp_path / "hydra_config.yaml").open() as stream:
        config = yaml.safe_load(stream)
    assert config["default_num_threads"] == 1


@pytest.mark.parametrize("dataset,horizon", [("habitat", 15.0), ("sidpac", 100.0)])
def test_dataset_deformation_settings(tmp_path, dataset, horizon):
    """Dataset deformation tuning reaches the constructed frontend and saved config."""
    sensor = hydra.make_camera(40.0, 40.0, 23.5, 23.5, 48, 48)
    pipeline = hydra.load_pipeline(sensor, dataset, "ade20k_mp3d", output_path=tmp_path)
    assert step(pipeline, 1)
    pipeline.stop()
    pipeline.save()
    assert saved_mesh(tmp_path).num_vertices() > 0
    with (tmp_path / "hydra_config.yaml").open() as stream:
        config = yaml.safe_load(stream)
    assert config["frontend"]["deformation_graph_builder"]["horizon_s"] == horizon
    assert config["frontend"]["deformation_graph_builder"]["resolution"] == 2.5
