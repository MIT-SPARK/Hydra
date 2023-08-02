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
import json
import click
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np


@click.command()
@click.argument("input_file", type=click.File("r"))
@click.option("-n", "--index", default=-1, type=int)
@click.option("-s", "--save", default=False, is_flag=True)
def main(input_file, index, save):
    results = json.loads(input_file.read())
    filtration = results["results"][index][1]
    distances = np.array([x[0] for x in filtration])
    components = np.array([x[1] for x in filtration])

    barcodes = [x[1] for x in results["barcodes"][index][1]]
    barcodes = sorted(barcodes, key=lambda x: x[1] - x[0], reverse=True)

    points = np.zeros((len(barcodes), 2))
    for i, lifetime in enumerate(barcodes):
        # lifetimes are [start, end), starting with greatest distance at 0
        start_idx = len(distances) - lifetime[1] + 1
        end_idx = len(distances) - lifetime[0]
        start_distance = distances[start_idx]
        end_distance = distances[end_idx]

        points[i, 0] = start_distance
        points[i, 1] = end_distance

    print(np.min(points))
    print(np.max(points))

    sns.set()
    sns.set_style("white")
    color = sns.color_palette()[0]

    fig, ax = plt.subplots(1, 2)
    ax[0].plot(distances, components, marker="+")
    ax[0].set_xlabel("Dilation Distance [m]")
    ax[0].set_ylabel("Number of Components")

    min_dist = min(distances)
    max_dist = max(distances)
    print(min_dist, max_dist)

    ax[1].plot([min_dist, max_dist], [min_dist, max_dist], c=color)
    ax[1].fill(
        [min_dist, max_dist, min_dist, min_dist],
        [min_dist, max_dist, max_dist, min_dist],
        c=color + (0.3,),
    )
    ax[1].scatter(points[:, 0], points[:, 1], color=color, marker="o")
    ax[1].set_xlabel("Birth [m]")
    ax[1].set_ylabel("Death [m]")

    if save:
        plt.savefig("filtration.pdf")

    plt.show()


if __name__ == "__main__":
    main()
