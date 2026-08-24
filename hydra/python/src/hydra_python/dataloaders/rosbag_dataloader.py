"""Dataloader that wraps a rosbag."""

import logging
import pathlib

import numpy as np
from ianvs.bag_reader import BagReader  # type: ignore

from hydra_python.trajectory import Pose, Trajectory


def _find_camera_info(bag, topic):
    logging.info(f"Looking for camera info @ '{topic}'")
    for _, msg, _ in bag.read_messages([topic]):
        return msg

    return None


def _parse_camera_info(msg):
    return {
        "width": msg.width,
        "height": msg.height,
        "fx": msg.k[0],
        "fy": msg.k[4],
        "cx": msg.k[2],
        "cy": msg.k[5],
    }


def _tf_to_pose(tf):
    pos = [tf.translation.x, tf.translation.y, tf.translation.z]
    q = [tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w]
    return Pose.from_flattened(np.array(pos + q))


def parse_message_timestamp(msg):
    return int(msg.header.stamp.sec * 1.0e9) + msg.header.stamp.nanosec


def load_trajectory_from_bag(
    bag: BagReader, map_frame: str, body_frame: str, progress: bool = False
):
    poses = []
    timestamps = []
    for _, msg, _ in bag.read_messages(["/tf"], progress=progress):
        for x in msg.transforms:
            if map_frame != x.header.frame_id or body_frame != x.child_frame_id:
                continue

            poses.append(_tf_to_pose(x.transform))
            timestamps.append(parse_message_timestamp(x))

    # TODO(nathan) think about jointly sorting
    return Trajectory(timestamps, poses)


class RosbagDataLoader:
    """Class for loading rosbags."""

    def __init__(
        self,
        bag: BagReader,
        trajectory: Trajectory,
        rgb_topic: str,
        other_topics: list[str] | None,
        body_frame: str | None = None,
        rgb_info_topic: str | None = None,
        min_separation_s: float = 0.0,
        threshold_us: int = 0,
        is_bgr: bool = True,
        start_time_ns: int | None = None,
        progress: bool = False,
    ):
        """
        Construct a rosbag dataset interface.

        Note:
            threshold_us should be no more than half the expected frame period
        """
        self._bag = bag
        self._rgb_topic = rgb_topic
        if rgb_info_topic is None:
            self._rgb_info_topic = str(pathlib.Path(rgb_topic).parent / "camera_info")
        else:
            self._rgb_info_topic = rgb_info_topic

        self._topics = [self._rgb_topic] + (other_topics or [])
        self._trajectory = trajectory
        self._threshold_us = threshold_us
        self._is_bgr = is_bgr
        self._start_time_ns = start_time_ns
        self._progress = progress

        msg = _find_camera_info(self._bag, self._rgb_info_topic)
        if msg is None:
            raise ValueError(f"Could not find camera info for '{self._rgb_topic}'")

        self._camera_info = _parse_camera_info(msg)

        sensor_frame = msg.header.frame_id
        self._body_T_sensor = Pose()
        if body_frame is not None:
            tf_msg = bag.lookup_static_transform(body_frame, sensor_frame)
            self._body_T_sensor = _tf_to_pose(tf_msg)
            logging.info("Loaded body_T_sensor: {self._body_T_sensor}")

    @property
    def intrinsics(self):
        """Get camera info."""
        return self._camera_info

    def __iter__(self):
        """Return the iterator object."""

        abs_start_time = 0
        if self._start_time_ns is not None:
            abs_start_time = self._bag.start_time + self._start_time_ns

        thresh_ns = int(1.0e3 * self._threshold_us)
        msg_iter = self._bag.read_synced_messages(
            self._topics,
            max_diff_ns=thresh_ns,
            start_time_ns=abs_start_time,
            progress=self._progress,
        )

        for messages in msg_iter:
            time = parse_message_timestamp(messages[self._rgb_topic])
            if self._trajectory is not None:
                pose = self._trajectory.pose(time)
                if pose is None:
                    continue

                pose = pose @ self._body_T_sensor

            yield time, pose, [messages[t] for t in self._topics]
