"""Helper functions for working with trajectories."""

import pathlib

import spark_dsg as dsg
from scipy.spatial.transforms import Rotation
from spark_dataset_interfaces import Pose, Trajectory


def trajectory_from_scene_graph(filepath, agent_prefix="a"):
    """Construct a trajectory from a previous scene graph."""
    filepath = pathlib.Path(filepath).expanduser().absolute()
    if not filepath.exists():
        return None

    G = dsg.DynamicSceneGraph.load(filepath)
    agents = G.get_layer(dsg.DsgLayers.AGENTS, agent_prefix)

    times = []
    poses = []
    for node in agents.nodes:
        q = node.attributes.world_R_body
        q = [q.x, q.y, q.z, q.w]
        times.append(node.attributes.timestamp)
        poses.append(Pose(Rotation.from_quat(q), node.attributes.position))

    return Trajectory(times, poses)
