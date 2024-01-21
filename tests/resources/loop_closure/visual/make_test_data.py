#!/usr/bin/env python3
"""Use reconstruction pipeline to make a higher quality mesh."""
import click
import rosbag
import pathlib
import cv2
import warnings
import yaml
import sys
from cv_bridge import CvBridge


def _load_poses(bag, buffer_length_s=1.0e6):
    import tf2_py
    import rospy

    tf_buffer = tf2_py.BufferCore(rospy.Duration(buffer_length_s))
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        for _, tf_msg, _ in bag.read_messages(topics=["/tf_static"]):
            for msg in tf_msg.transforms:
                tf_buffer.set_transform_static(msg, "rosbag")

        for _, tf_msg, _ in bag.read_messages(topics=["/tf"]):
            for msg in tf_msg.transforms:
                tf_buffer.set_transform(msg, "rosbag")

    return tf_buffer


def _load_transform(tf_buffer, frame_a, frame_b, stamp):
    """Lookup a transform between two frames."""
    try:
        a_T_b = tf_buffer.lookup_transform_core(frame_a, frame_b, stamp)
    except Exception as e:
        click.secho(f"tf error: {e}", fg="red")
        click.secho(f"frames: {{\n{tf_buffer.all_frames_as_yaml()}}}", fg="red")
        sys.exit(1)
    return a_T_b.transform


def _export_pose(a_T_b):
    q = a_T_b.rotation
    t = a_T_b.translation
    return {
        "q": {"w": q.w, "x": q.x, "y": q.y, "z": q.z},
        "t": {"x": t.x, "y": t.y, "z": t.z},
    }


def _parse_images(bag, topic, depth_topic, odom_frame, frame_indices):
    tf_buffer = _load_poses(bag)

    bridge = CvBridge()
    frame_idx = 0
    frames = []
    times = []
    num_images = bag.get_message_count(topic_filters=[topic])
    click.secho(f"{topic} has {num_images} messages", fg="green")
    for index, msg_tuple in enumerate(bag.read_messages(topics=[topic])):
        if frame_idx >= len(frame_indices):
            break

        if index != frame_indices[frame_idx]:
            continue

        msg = msg_tuple[1]
        click.secho(f"frame {index} @ {msg.header.stamp.to_nsec()} [ns]", fg="green")
        img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        world_T_camera = _load_transform(
            tf_buffer, odom_frame, msg.header.frame_id, msg.header.stamp
        )
        frames.append((img, world_T_camera))
        times.append(msg.header.stamp.to_nsec())
        frame_idx += 1

    depth_idx = 0
    depth_frames = []
    for _, msg, _ in bag.read_messages(topics=[depth_topic]):
        if depth_idx >= len(times):
            break

        if msg.header.stamp.to_nsec() != times[depth_idx]:
            continue

        click.secho(f"depth @ {msg.header.stamp.to_nsec()} [ns]", fg="green")
        img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        depth_frames.append(img)
        depth_idx += 1

    assert len(frames) == len(depth_frames)
    return [(x[0], y, x[1]) for x, y in zip(frames, depth_frames)]


@click.command()
@click.argument("rosbag_path", type=click.Path(exists=True))
@click.argument("frame_indices", nargs=-1, type=int)
@click.option("-t", "--topic", default="/tesse/left_cam/rgb/image_raw")
@click.option("-d", "--depth-topic", default="/tesse/depth_cam/mono/image_raw")
@click.option("-f", "--odom-frame", default="world")
@click.option("-o", "--output", default=None)
@click.option("--flip-color/--no-flip-color", default=True)
def main(
    rosbag_path, frame_indices, topic, depth_topic, odom_frame, output, flip_color
):
    """Run reconstruction against a ROS bag."""
    rosbag_path = pathlib.Path(rosbag_path).expanduser().absolute()
    if output is None:
        output_path = pathlib.Path(".").absolute()
    else:
        output_path = pathlib.Path(output).expanduser().absolute()

    click.secho(f"Opening {rosbag_path}...", fg="green")
    with rosbag.Bag(str(rosbag_path), "r") as bag:
        click.secho("Opened bag", fg="green")
        images = _parse_images(bag, topic, depth_topic, odom_frame, frame_indices)

        pose_export = {}
        for index, posed_img in enumerate(images):
            rgb, depth, pose = posed_img
            pose_export[index] = _export_pose(pose)
            filename = str(output_path / f"img_{index:03d}.png")
            if flip_color:
                cv2.imwrite(filename, rgb[:, :, ::-1])
            else:
                cv2.imwrite(filename, rgb)

            cv2.imwrite(str(output_path / f"depth_{index:03d}.tiff"), depth)

        with (output_path / "image_poses.yaml").open("w") as fout:
            fout.write(yaml.dump(pose_export, default_flow_style=True))


if __name__ == "__main__":
    main()
