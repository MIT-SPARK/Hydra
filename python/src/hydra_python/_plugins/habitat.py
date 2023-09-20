"""Habitat-specific simulator."""
from scipy.spatial.transform import Rotation as R
from typing import Union
import spark_dsg.mp3d
import hydra_python as hydra
import networkx as nx
import numpy as np
import habitat_sim
import pathlib
import magnum
import random
import os


MISSING_ADE_LABELS = [29, 33]


def _get_index(vertices, pos):
    distances = np.linalg.norm(vertices - np.squeeze(pos), axis=1)
    return np.argmin(distances)


def _compute_path_distance(G, nodes):
    dist = 0.0
    for i in range(len(nodes) - 1):
        v1 = G.nodes[nodes[i]]["pos"]
        v2 = G.nodes[nodes[i + 1]]["pos"]
        dist += np.linalg.norm(v1 - v2)

    return dist


def _build_navgraph(sim, pathfinder, settings, threshold):
    success = sim.recompute_navmesh(pathfinder, settings)
    if not success:
        raise RuntimeError("Failed to make navmesh")

    faces = pathfinder.build_navmesh_vertices()
    G = nx.Graph()

    vertices = []
    for face_vertex in faces:
        curr_pos = np.array(face_vertex).reshape((3, 1))
        matches_prev = False

        for vertex in vertices:
            dist = np.linalg.norm(curr_pos - vertex)
            if dist < threshold:
                matches_prev = True
                break

        if matches_prev:
            continue

        G.add_node(len(vertices), pos=curr_pos)
        vertices.append(curr_pos)

    vertices = np.squeeze(np.array(vertices))
    for i in range(0, len(faces), 3):
        v1 = _get_index(vertices, faces[i + 0])
        v2 = _get_index(vertices, faces[i + 1])
        v3 = _get_index(vertices, faces[i + 2])

        w1 = np.linalg.norm(vertices[v1, :] - vertices[v2, :])
        w2 = np.linalg.norm(vertices[v2, :] - vertices[v3, :])
        w3 = np.linalg.norm(vertices[v3, :] - vertices[v1, :])

        G.add_edge(v1, v2, weight=w1)
        G.add_edge(v2, v3, weight=w2)
        G.add_edge(v3, v1, weight=w3)

    largest_cc = max(nx.connected_components(G), key=len)
    return G.subgraph(largest_cc)


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


def _set_logging():
    log_settings = [
        "Default=Quiet",
        "Metadata=Quiet",
        "Assets=Quiet",
        "Physics=Quiet",
        "Nav=Quiet",
        "Scene=Quiet",
        "Sensor=Quiet",
        "Gfx=Quiet",
    ]
    os.environ["HABITAT_SIM_LOG"] = ":".join(log_settings)
    try:
        habitat_sim._ext.habitat_sim_bindings.core.LoggingContext.reinitialize_from_env()
    except AttributeError:
        print("Failed to setup logging")


def _transform_from_habitat(h_q_c, p_h):
    w_R_h = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
    p_w = np.squeeze(w_R_h @ p_h.reshape((3, 1)))

    h_R_c = R.from_quat(h_q_c).as_matrix()
    w_R_c = w_R_h @ h_R_c @ w_R_h.T
    w_q_c = R.from_matrix(w_R_c).as_quat()
    return w_q_c, p_w


def _transform_to_habitat(w_q_c, p_w):
    h_R_w = np.array([[0, -1, 0], [0, 0, 1], [-1, 0, 0]])
    p_h = np.squeeze(h_R_w @ p_w.reshape((3, 1)))

    w_R_c = R.from_quat(w_q_c).as_matrix()
    h_R_c = h_R_w @ w_R_c @ h_R_w.T
    h_q_c = R.from_matrix(h_R_c).as_quat()
    return h_q_c, p_h


def _transform_to_body(w_q_c, p_c):
    b_R_c = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    w_R_c = R.from_quat(w_q_c).as_matrix()
    w_R_b = w_R_c @ b_R_c.T
    w_q_b = R.from_matrix(w_R_b).as_quat()
    # note that b_T_c doesn't have translation...
    return w_q_b, p_c


def _camera_point_from_habitat(p_ah, z_offset=1.5):
    p_bh = np.array(p_ah)
    # z offset is relative to ENU
    p_bh[1] += z_offset
    bw_R_bh = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
    p_bw = np.squeeze(bw_R_bh @ p_bh.reshape((3, 1)))

    # bw_R_c = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    # p_c = np.squeeze(bw_R_c.T @ p_bw.reshape((3, 1)))
    return p_bw


class HabitatInterface:
    """Class handling interfacing with habitat."""

    def __init__(self, scene: Union[str, pathlib.Path]):
        """Initialize the simulator."""
        scene = pathlib.Path(scene).expanduser().resolve()
        # TODO(nathan) expose some of this via the data interface
        _set_logging()
        config, camera_info = _make_habitat_config(scene)
        self._house_path = scene.parent / f"{scene.stem}.house"
        self._camera_info = camera_info
        self._sim = habitat_sim.Simulator(config)
        self._make_instance_labelmap()

        self._obs = None
        self._labels = None

    def _make_instance_labelmap(self):
        object_to_cat_map = {
            c.id: c.category.index() for c in self._sim.semantic_scene.objects
        }

        mpcat_to_ade = np.arange(41)
        for label in sorted(MISSING_ADE_LABELS):
            mpcat_to_ade[label:] -= 1
            mpcat_to_ade[label] = 0

        ade_to_mpcat = {}
        for mpcat_idx, ade_idx in enumerate(mpcat_to_ade):
            if ade_idx not in ade_to_mpcat:
                ade_to_mpcat[ade_idx] = mpcat_idx

        category_map = np.array(
            [mpcat_to_ade[idx] for _, idx in object_to_cat_map.items()]
        )
        self._labelmap = hydra.LabelConverter(category_map)

        name_mapping = {}
        for c in self._sim.semantic_scene.categories:
            name_mapping[c.index()] = c.name()

        keys = sorted([x for x in ade_to_mpcat])
        names = [name_mapping[ade_to_mpcat[idx]] for idx in keys]
        self._colormap = hydra.SegmentationColormap.from_names(names=names)

    def get_full_trajectory(
        self,
        inflation_radius=0.1,
        threshold=1.0e-3,
        seed=None,
        z_offset=1.0,
        add_reverse=False,
        max_room_distance=5.0,
        **kwargs,
    ):
        """Get a trajectory that explores the entire scene."""
        pathfinder = habitat_sim.nav.PathFinder()
        settings = habitat_sim.NavMeshSettings()
        settings.agent_radius = inflation_radius
        G = _build_navgraph(self._sim, pathfinder, settings, threshold)
        components = list(nx.connected_components(G))
        if len(components) > 1:
            print("Warning: {len(components)} components found in navgraph!")
            components = sorted(components, lambda x: len(x), reverse=True)

        mp3d_info = spark_dsg.mp3d.load_mp3d_info(self._house_path)
        rooms = spark_dsg.mp3d.get_rooms_from_mp3d_info(mp3d_info, angle_deg=-90)

        node_sequence = []
        for room in rooms:
            best_node = None
            best_distance = None
            for x in components[0]:
                pos = _camera_point_from_habitat(G.nodes[x]["pos"], z_offset=z_offset)
                dist = np.linalg.norm(pos - room.centroid)
                if not best_distance or dist < best_distance:
                    best_node = x
                    best_distance = dist

            if best_node is None:
                continue

            if best_distance > max_room_distance:
                pos = _camera_point_from_habitat(
                    G.nodes[best_node]["pos"], z_offset=z_offset
                )
                print(f"No node for {room.get_id()} @ {np.squeeze(room.centroid)}")
                print(f"Closest: {best_node} @ {np.squeeze(pos)}")
                continue

            node_sequence.append(best_node)

        if len(node_sequence) <= 1:
            return None

        if seed is not None:
            random.seed(seed)

        random.shuffle(node_sequence)
        if add_reverse:
            node_sequence = node_sequence + node_sequence[::-1]

        b_R_c = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
        first_pos_habitat = G.nodes[node_sequence[0]]["pos"]
        first_pos_cam = _camera_point_from_habitat(first_pos_habitat, z_offset=z_offset)

        traj = hydra.Trajectory.rotate(first_pos_cam, body_R_camera=b_R_c, **kwargs)
        for i in range(len(node_sequence) - 1):
            start = node_sequence[i]
            end = node_sequence[i + 1]
            nodes = nx.shortest_path(G, source=start, target=end, weight="weight")
            if len(nodes) <= 1:
                continue

            pos_habitat = [G.nodes[x]["pos"] for x in nodes]
            pos_cam = [
                _camera_point_from_habitat(p, z_offset=z_offset) for p in pos_habitat
            ]
            new_traj = hydra.Trajectory.from_positions(
                np.array(pos_cam), body_R_camera=b_R_c, **kwargs
            )

            traj += new_traj
            traj += hydra.Trajectory.rotate(
                np.array(pos_cam[-1]), body_R_camera=b_R_c, **kwargs
            )

        return traj

    def get_random_trajectory(
        self,
        target_length_m=100.0,
        inflation_radius=0.1,
        threshold=1.0e-3,
        seed=None,
        z_offset=0.5,
    ):
        """Get a trajectory as sequence of segments between random areas in a scene."""
        pathfinder = habitat_sim.nav.PathFinder()
        settings = habitat_sim.NavMeshSettings()
        settings.agent_radius = inflation_radius
        G = _build_navgraph(self._sim, pathfinder, settings, threshold)

        if seed is not None:
            random.seed(seed)

        node_sequence = [x for x in G]
        random.shuffle(node_sequence)

        path = []
        total_length = 0.0
        for i in range(len(node_sequence) - 1):
            start = node_sequence[i]
            end = node_sequence[i + 1]
            nodes = nx.shortest_path(G, source=start, target=end, weight="weight")
            total_length += _compute_path_distance(G, nodes)
            path += nodes[:-1]

            if total_length > target_length_m:
                break

        positions_habitat = [G.nodes[x]["pos"] for x in path]
        positions_camera = [
            _camera_point_from_habitat(p, z_offset=z_offset) for p in positions_habitat
        ]
        b_R_c = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
        return hydra.Trajectory.from_positions(
            np.array(positions_camera), body_R_camera=b_R_c
        )

    def set_pose(self, timestamp, world_T_camera):
        """Set pose of the agent directly."""
        w_q_c = R.from_matrix(world_T_camera[:3, :3]).as_quat()
        w_q_b, p_b = _transform_to_body(w_q_c, world_T_camera[:3, 3])
        h_q_c, p_h = _transform_to_habitat(w_q_b, p_b)

        new_state = habitat_sim.AgentState()
        new_state.position = magnum.Vector3(p_h[0], p_h[1], p_h[2])
        new_state.rotation = h_q_c
        self._sim.agents[0].set_state(new_state)
        self._obs = self._sim.get_sensor_observations()
        self._labels = None

    @property
    def colormap(self):
        """Get colormap between labels and semantic colors."""
        return self._colormap

    @property
    def camera_info(self):
        """Get camera properties."""
        return self._camera_info

    @property
    def depth(self):
        """Get current depth observation if available."""
        if self._obs is None:
            return None

        return self._obs[str(habitat_sim.SensorType.DEPTH)]

    @property
    def labels(self):
        """Get current semantics observation if available."""
        if self._obs is None:
            return None

        if self._labels is None:
            instance_labels = self._obs[str(habitat_sim.SensorType.SEMANTIC)]
            self._labels = self._labelmap(instance_labels).astype(np.int32)

        return self._labels

    @property
    def rgb(self):
        """Get current RGB observation if available."""
        if self._obs is None:
            return None

        return self._obs[str(habitat_sim.SensorType.COLOR)][:, :, :3]


hydra.DataInterface.register(HabitatInterface)
