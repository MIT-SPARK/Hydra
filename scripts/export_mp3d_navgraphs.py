"""Habitat-specific simulator."""

import json
import multiprocessing as mp
import pathlib

import click
import habitat_sim
import networkx as nx
import numpy as np
import tqdm


def _make_sensor(sensor_type, width=640, height=360, hfov=90.0, z_offset=0.0):
    spec = habitat_sim.CameraSensorSpec()
    spec.uuid = str(sensor_type)
    spec.sensor_type = sensor_type
    spec.resolution = [height, width]
    spec.position = [0.0, z_offset, 0.0]
    spec.orientation = [0.0, 0.0, 0.0]
    spec.hfov = hfov
    spec.sensor_subtype = habitat_sim.SensorSubType.PINHOLE
    return spec


def _make_habitat_config(scene, z_offset=0.0, agent_radius=0.1):
    sim_cfg = habitat_sim.SimulatorConfiguration()
    path = scene.parent.parent
    json_path = path / "mp3d.scene_dataset_config.json"
    sim_cfg.scene_dataset_config_file = str(json_path)
    sim_cfg.create_renderer = False
    sim_cfg.gpu_device_id = 0
    sim_cfg.scene_id = str(scene)
    sim_cfg.enable_physics = True
    sim_cfg.allow_sliding = False

    sensor_specs = [
        _make_sensor(x, z_offset=z_offset)
        for x in [
            habitat_sim.SensorType.COLOR,
            habitat_sim.SensorType.DEPTH,
            habitat_sim.SensorType.SEMANTIC,
        ]
    ]

    camera_spec = sensor_specs[0]
    height, width = camera_spec.resolution
    focal_length = width / (2.0 * np.tan(float(camera_spec.hfov) * np.pi / 360.0))
    camera_info = {
        "fx": float(focal_length),
        "fy": float(focal_length),
        "cx": float(width / 2.0),
        "cy": float(height / 2.0),
        "width": int(width),
        "height": int(height),
    }

    agent_cfg = habitat_sim.agent.AgentConfiguration(
        height=z_offset,
        radius=agent_radius,
        sensor_specifications=sensor_specs,
        action_space={},
        body_type="cylinder",
    )

    return habitat_sim.Configuration(sim_cfg, [agent_cfg]), camera_info


def _build_navgraph(sim, inflation_radius=0.15, threshold=0.01):
    pathfinder = habitat_sim.nav.PathFinder()
    settings = habitat_sim.NavMeshSettings()
    settings.agent_radius = inflation_radius
    success = sim.recompute_navmesh(pathfinder, settings)
    if not success:
        raise RuntimeError("Failed to make navmesh")

    faces = pathfinder.build_navmesh_vertices()
    G = nx.Graph()

    vertices = []
    face_to_vertices = {}
    for face_idx, face_vertex in enumerate(faces):
        curr_pos = np.array(face_vertex).reshape((3, 1))
        matches_prev = False

        for vertex_idx, vertex in enumerate(vertices):
            dist = np.linalg.norm(curr_pos - vertex)
            if dist < threshold:
                matches_prev = True
                face_to_vertices[face_idx] = vertex_idx
                break

        if matches_prev:
            continue

        face_to_vertices[face_idx] = len(vertices)
        G.add_node(len(vertices), pos=curr_pos.tolist())
        vertices.append(curr_pos)

    vertices = np.squeeze(np.array(vertices))
    for i in range(0, len(faces), 3):
        v1 = face_to_vertices[i]
        v2 = face_to_vertices[i + 1]
        v3 = face_to_vertices[i + 2]

        w1 = float(np.linalg.norm(vertices[v1, :] - vertices[v2, :]))
        w2 = float(np.linalg.norm(vertices[v2, :] - vertices[v3, :]))
        w3 = float(np.linalg.norm(vertices[v3, :] - vertices[v1, :]))

        G.add_edge(v1, v2, weight=w1)
        G.add_edge(v2, v3, weight=w2)
        G.add_edge(v3, v1, weight=w3)

    return G


def _export(mesh_path, output):
    # mesh_path, output = data
    mesh_path = pathlib.Path(mesh_path).expanduser().absolute()
    mesh_name = mesh_path.stem
    config, _ = _make_habitat_config(mesh_path)
    sim = habitat_sim.Simulator(config)

    G = _build_navgraph(sim)
    mesh_output = output / f"{mesh_name}.json"
    with mesh_output.open("w") as fout:
        json.dump(nx.node_link_data(G), fout)


@click.command()
@click.argument("mesh_paths", type=click.Path(exists=True), nargs=-1)
@click.option("--output", "-o", type=click.Path(), default=None)
def main(mesh_paths, output):
    if output is None:
        output = pathlib.Path(".").absolute()
    else:
        output = pathlib.Path(output).expanduser().absolute()

    output.mkdir(parents=True, exist_ok=True)

    for mesh_path in tqdm.tqdm(mesh_paths):
        proc = mp.Process(target=_export, args=(mesh_path, output))
        proc.start()
        proc.join()


if __name__ == "__main__":
    main()
