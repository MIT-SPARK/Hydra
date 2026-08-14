#!/usr/bin/env python3
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
"""Copy static tfs from one uhumans2 bag to a launch file."""

import argparse

import rosbag

LINE_STR = "{ts},{x},{y},{z},{qw},{qx},{qy},{qz}\n"


def main():
    """Run some stuff."""
    parser = argparse.ArgumentParser(
        description="utiltiy to read a odom information from a bag."
    )
    parser.add_argument("bag_file", type=str, help="bag file to use")
    parser.add_argument("topic", type=str, help="bag file to use")
    parser.add_argument("output", type=str, help="file to output to")
    args = parser.parse_args()

    with open(args.output, "w") as fout:
        fout.write("#timestamp_kf,x,y,z,qw,qx,qy,qz\n")

        with rosbag.Bag(args.bag_file, "r") as bag:
            for topic, msg, t in bag.read_messages(topics=[args.topic]):
                fout.write(
                    LINE_STR.format(
                        ts=msg.header.stamp.to_nsec(),
                        x=msg.pose.pose.position.x,
                        y=msg.pose.pose.position.y,
                        z=msg.pose.pose.position.z,
                        qx=msg.pose.pose.orientation.x,
                        qy=msg.pose.pose.orientation.y,
                        qz=msg.pose.pose.orientation.z,
                        qw=msg.pose.pose.orientation.w,
                    )
                )


if __name__ == "__main__":
    main()
