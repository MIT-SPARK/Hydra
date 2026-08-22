"""Habitat-specific simulator."""

import pathlib
from typing import Union

import numpy as np
from scipy.spatial.transform import Rotation as R


def _make_sensor(sensor_type, width=640, height=480, hfov=90.0, z_offset=0.0):
    import habitat_sim

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
    import habitat_sim

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
    return p_bw


class HabitatInterface:
    """Class handling interfacing with habitat."""

    def __init__(self, scene: Union[str, pathlib.Path], reindex_labels=True):
        """Initialize the simulator."""
        import habitat_sim

        self._scene = pathlib.Path(scene).expanduser().resolve()
        config, camera_info = _make_habitat_config(self._scene)
        self._camera_info = camera_info
        self._sim = habitat_sim.Simulator(config)

        def _parse_instance(obj_id):
            parts = obj_id.split("_")
            return int(parts[-1])

        try:
            scene = self.sim.semantic_scene
            label_map = {
                _parse_instance(c.id): c.category.index() for c in scene.objects
            }
            name_map = {c.index(): c.name() for c in scene.categories}
            self._labelmap = LabelConverter.from_mapping(label_map, name_map)
            if reindex_labels:
                self._labelmap = self._labelmap.reindex(self.mpcat_to_ade)
        except Exception as e:
            print(f"Loading label conversion failed: {e}")
            self._labelmap = None

        self._obs = None
        self._labels = None
        self._color_name = str(habitat_sim.SensorType.COLOR)
        self._depth_name = str(habitat_sim.SensorType.DEPTH)
        self._label_name = str(habitat_sim.SensorType.SEMANTIC)

    @property
    def sim(self):
        """Return the underlying simulator."""
        return self._sim

    @property
    def house_path(self):
        """Get path to corresponding house file."""
        return self._scene.parent / f"{self._scene.stem}.house"

    @property
    def mpcat_to_ade(self):
        """Get remapping from mpcat40 to ade20k."""
        # labels 29 and 33 don't show up in ade20k
        return {
            29: 0,
            30: 29,
            31: 30,
            32: 31,
            33: 0,
            34: 32,
            35: 33,
            36: 34,
            37: 35,
            38: 36,
            39: 37,
            40: 38,
        }

    def set_pose(self, timestamp, world_T_camera):
        """Set pose of the agent directly."""
        import habitat_sim
        import magnum

        w_q_c = R.from_matrix(world_T_camera[:3, :3]).as_quat()
        p_c = world_T_camera[:3, 3]
        w_q_b, p_b = _transform_to_body(w_q_c, p_c)
        h_q_c, p_h = _transform_to_habitat(w_q_b, p_b)

        new_state = habitat_sim.AgentState()
        new_state.position = magnum.Vector3(p_h[0], p_h[1], p_h[2])
        new_state.rotation = h_q_c
        self._sim.agents[0].set_state(new_state)
        self._obs = self._sim.get_sensor_observations()
        self._labels = None

    @property
    def sensor(self):
        """Get camera properties."""
        return self._camera_info

    @property
    def depth(self):
        """Get current depth observation if available."""
        return None if self._obs is None else self._obs[self._depth_name]

    @property
    def labels(self):
        """Get current semantics observation if available."""
        if self._obs is None:
            return None

        if self._labels is not None:
            return self._labels

        instances = self._obs.get(self._label_name)
        if instances is None or self._labelmap is None:
            return instances

        self._labels = self._labelmap(instances).astype(np.int32)
        return self._labels

    @property
    def rgb(self):
        """Get current RGB observation if available."""
        return None if self._obs is None else self._obs[self._color_name][:, :, :3]
