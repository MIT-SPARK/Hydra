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
import pathlib
import shutil
import tqdm


@click.command()
@click.argument("input_file", type=click.File("r"))
@click.option("-o", "--output", default=None)
def main(input_file, output):
    sns.set()
    sns.set_style("white")

    if output is None:
        output_path = pathlib.Path("filtration_plots")
    else:
        output_path = pathlib.Path(output).expanduser().absolute()

    if output_path.exists():
        click.confirm(f"{output_path} exists! okay to delete", abort=True)
        shutil.rmtree(output_path)

    output_path.mkdir(parents=True)

    results = json.loads(input_file.read())
    contents = results["contents"]
    for idx, timestep in tqdm.tqdm(enumerate(contents), total=len(contents)):
        start, stop = timestep["start"], timestep["end"]
        distance = timestep["threshold"]
        filtration = timestep["filtration"]
        components = np.array([x["c"] for x in filtration])
        distances = np.array([x["d"] for x in filtration])

        fig, ax = plt.subplots()
        ax.plot(distances, components, marker="+", color="g")
        ax.plot(distances[:start], components[:start], marker="+", color="r")
        ax.plot(distances[stop:], components[stop:], marker="+", color="r")
        ax.set_ylim([0, 10])
        ax.set_xlim([0, 2.5])
        limits = ax.get_ylim()
        ax.vlines(distance, limits[0], limits[1], colors="k", linestyle="--")
        ax.set_xlabel("Dilation Distance [m]")
        ax.set_ylabel("Number of Components")
        fig.set_size_inches([8, 5])
        plt.tight_layout()
        plt.savefig(str(output_path / f"timestep_{idx:06d}.png"))
        plt.close("all")


if __name__ == "__main__":
    main()
