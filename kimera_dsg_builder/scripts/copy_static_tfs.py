#!/usr/bin/env python
"""Copy static tfs from one uhumans2 bag to a launch file."""
import rosbag
import argparse
import sys


def main():
    """Run some stuff."""
    parser = argparse.ArgumentParser(
        description="utiltiy to read a static tf from a bag."
    )
    parser.add_argument("bag_file", type=str, help="bag file to use")
    parser.add_argument("output", type=str, help="file to output to")
    args = parser.parse_args()

    tf_message = None
    with rosbag.Bag(args.bag_file, "r") as bag:
        for topic, msg, t in bag.read_messages(topics=["/tf_static"]):
            tf_message = msg

    if tf_message is None:
        print("Could not find static tf in {}".format(args.bag_file))
        sys.exit(1)

    with open(args.output, "w") as fout:
        fout.write("<launch>\n\n")

        for idx, msg in enumerate(tf_message.transforms):
            fout.write('    <node pkg="tf2_ros" type="static_transform_publisher"')
            name = "bag_static_tf_{}".format(idx)
            fout.write(' name="{}"'.format(name))
            fout.write(
                ' args="{x} {y} {z} {qx} {qy} {qz} {qw} {parent} {child}"/>\n\n'.format(
                    x=msg.transform.translation.x,
                    y=msg.transform.translation.y,
                    z=msg.transform.translation.z,
                    qx=msg.transform.rotation.x,
                    qy=msg.transform.rotation.y,
                    qz=msg.transform.rotation.z,
                    qw=msg.transform.rotation.w,
                    parent=msg.header.frame_id,
                    child=msg.child_frame_id,
                )
            )

        fout.write("</launch>\n")


if __name__ == "__main__":
    main()
