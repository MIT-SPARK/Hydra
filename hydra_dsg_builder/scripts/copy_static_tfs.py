# Copyright 2022, Massachusetts Institute of Technology.
# All Rights Reserved
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Research was sponsored by the United States Air Force Research Laboratory and
# the United States Air Force Artificial Intelligence Accelerator and was
# accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
# and conclusions contained in this document are those of the authors and should
# not be interpreted as representing the official policies, either expressed or
# implied, of the United States Air Force or the U.S. Government. The U.S.
# Government is authorized to reproduce and distribute reprints for Government
# purposes notwithstanding any copyright notation herein.
#
#
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
