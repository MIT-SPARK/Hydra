"""Class that generates a set of poses for the simulator."""
from scipy.spatial.transform import Rotation as Rot
from scipy.spatial.transform import Slerp
import spark_dsg as dsg
import numpy as np
import pathlib
import csv


def _interp_pose(start, end, alpha):
    new_pose = np.zeros(7)
    new_pose[:3] = alpha * (end[:3] - start[:3]) + start[:3]

    # scipy requires xyzw order
    q_start = np.array([start[4], start[5], start[6], start[3]])
    q_end = np.array([end[4], end[5], end[6], end[3]])

    slerp = Slerp([0, 1], Rot.from_quat([q_start, q_end]))
    q_mid = slerp([alpha])[0].as_quat()

    # xyzw -> wxyz
    new_pose[3] = q_mid[3]
    new_pose[4] = q_mid[0]
    new_pose[5] = q_mid[1]
    new_pose[6] = q_mid[2]
    return new_pose


def _pose_from_components(pos, yaw, body_R_camera):
    pose = np.zeros(7)
    pose[:3] = np.squeeze(pos)
    # we assume yaw around z-axis in ENU
    world_q_body = np.array([0.0, 0.0, np.sin(yaw / 2.0), np.cos(yaw / 2.0)])
    world_q_camera = (
        Rot.from_quat(world_q_body) * Rot.from_matrix(body_R_camera)
    ).as_quat()
    pose[3] = world_q_camera[3]
    pose[4] = world_q_camera[0]
    pose[5] = world_q_camera[1]
    pose[6] = world_q_camera[2]
    return pose


class Trajectory:
    """Represents pre-computed trajectory."""

    def __init__(self, times, poses):
        """
        Initialize the trajectory.

        Requires a set of N poses and N timestamps (in nanoseconds)

        Args:
            times (np.ndarray): N x 1 array of np.uint64 timestamps in nanoseconds
            trajectory_array (np.ndarray): N x 7 with columns x, y, z, qw, qx, qy, qz
        """
        self._times = np.squeeze(times)
        self._poses = poses

        N_times = self._times.size
        N_poses = self._poses.shape[0]
        if self._times.size != self._poses.shape[0]:
            raise ValueError(f"# of times ({N_times}) != # of poses ({N_poses})")

        if self._poses.shape[1] != 7:
            raise ValueError(f"invalid pose matrix: {self._poses.shape[1]} != 7")

    @classmethod
    def from_csv(cls, filename, skip_first=True):
        """Construct a trajectory from a saved csv file."""
        filepath = pathlib.Path(filename).resolve()
        if not filepath.exists():
            raise ValueError("invalid trajectory csv path: {filepath}")

        times = []
        poses = []
        with filepath.open("r") as fin:
            reader = csv.reader(fin)
            if skip_first:
                next(reader)

            for row in reader:
                times.append(int(row[0]))
                poses.append([float(x) for x in row[1:]])

        return cls(np.array(times, dtype=np.int64), np.array(poses))

    @classmethod
    def from_scene_graph(cls, filename, agent_prefix="a"):
        """Construct a trajectory from a previous scene graph."""
        filepath = pathlib.Path(filename).resolve()
        if not filepath.exists():
            raise ValueError("invalid scene graph: {filepath}")

        G = dsg.DynamicSceneGraph.load(str(filepath))
        agents = G.get_dynamic_layer(dsg.DsgLayers.AGENTS, agent_prefix)
        times = np.zeros(agents.num_nodes(), dtype=np.int64)
        poses = np.zeros((agents.num_nodes(), 7))

        for index, node in enumerate(agents.nodes):
            times[index] = node.timestamp
            poses[index, :3] = node.attributes.position
            poses[index, 3] = node.attributes.world_R_body.w
            poses[index, 4] = node.attributes.world_R_body.x
            poses[index, 5] = node.attributes.world_R_body.y
            poses[index, 6] = node.attributes.world_R_body.z

        return cls(times, poses)

    @classmethod
    def from_positions(
        cls,
        positions,
        body_R_camera=None,
        reinterp_distance=0.2,
        reinterp_angle=0.2,
        start_time_s=0.0,
        dt=0.2,
    ):
        """Construct a trajectory from a list of positions."""
        yaw = np.zeros(positions.shape[0])
        for i in range(positions.shape[0] - 1):
            diff = positions[i + 1] - positions[i]
            yaw[i] = np.arctan2(diff[1], diff[0])

        # last segment has no orientation change
        yaw[-1] = yaw[-2]

        b_R_c = np.eye(3) if body_R_camera is None else body_R_camera

        poses = []
        for i in range(positions.shape[0] - 1):
            pose_start = _pose_from_components(positions[i, :], yaw[i], b_R_c)
            pose_end = _pose_from_components(positions[i + 1, :], yaw[i + 1], b_R_c)

            poses.append(pose_start)

            dist = np.linalg.norm(pose_start[:3] - pose_end[:3])
            angle_dist = abs(yaw[i + 1] - yaw[i])
            num_intermediate_pos = int(np.ceil(dist / reinterp_distance) - 1)
            num_intermediate_yaw = int(np.ceil(angle_dist / reinterp_angle) - 1)
            num_intermediate = max(num_intermediate_pos, num_intermediate_yaw)
            for i in range(num_intermediate):
                # we want slerp ratio to be 0 at start (0)
                # and 1 at end (num_intermediate)
                ratio = (i + 1) / (num_intermediate + 1)
                poses.append(_interp_pose(pose_start, pose_end, ratio))

            poses.append(pose_end)

        poses = np.array(poses)
        times_s = dt * np.arange(poses.shape[0]) + start_time_s
        times_ns = (1.0e9 * times_s).astype(np.uint64)
        return cls(times_ns, poses)

    def save(self, filename):
        filepath = pathlib.Path(filename).expanduser().absolute()
        with filepath.open("w") as fout:
            writer = csv.writer(fout)
            writer.writerow(["timestamp_ns", "px", "py", "pz", "qw", "qx", "qy", "qz"])
            for index in range(self._times.size):
                writer.writerow(
                    [
                        self._times[index],
                        self._poses[index, 0],
                        self._poses[index, 1],
                        self._poses[index, 2],
                        self._poses[index, 3],
                        self._poses[index, 4],
                        self._poses[index, 5],
                        self._poses[index, 6],
                    ]
                )

    def __iter__(self):
        self._index = 0
        return self

    def __next__(self):
        if self._index >= self._times.size:
            raise StopIteration

        self._index += 1
        return (
            self._times[self._index - 1],
            self._poses[self._index - 1, :3],
            self._poses[self._index - 1, 3:],
        )

    def __len__(self):
        return self._times.size

    def get_path_length(self):
        """Compute total path length."""
        total_length = 0.0
        for i in range(self._poses.shape[0] - 1):
            total_length += np.linalg.norm(self._poses[i, :3] - self._poses[i + 1, :3])

        return total_length
