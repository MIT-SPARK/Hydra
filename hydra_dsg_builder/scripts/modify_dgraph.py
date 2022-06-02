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
#!/usr/bin/env python3
"""Modify deformation graph file."""
import argparse
import pathlib
import yaml
import sys
import re


POSE_FINDER = re.compile(r"^NODE(_TEMP)?.*?$", flags=re.MULTILINE)
BETWEEN_FINDER = re.compile(r"^BETWEEN(_TEMP)?.*?$", flags=re.MULTILINE)
VALENCE_FINDER = re.compile(r"^DEDGE(_TEMP)?.*?$", flags=re.MULTILINE)
PRIOR_FINDER = re.compile(r"^PRIOR.*?$", flags=re.MULTILINE)
VERTEX_FINDER = re.compile(r"^VERTEX.*?$", flags=re.MULTILINE)
KEY_MASK = 0xFF << 56
VERTEX_KEYS = set(["s", "t", "u", "v", "w", "x", "y", "z"])


def remap_prior_covariance(config, match):
    if "prior" not in config:
        return match.group(0)

    parts = match.group(0).split(" ")
    parts[9] = str(100 * config["prior"])
    parts[15] = str(100 * config["prior"])
    parts[20] = str(100 * config["prior"])
    parts[24] = str(config["prior"])
    parts[27] = str(config["prior"])
    parts[29] = str(config["prior"])
    return " ".join(parts)


def remap_between_covariance(config, match):
    """Remap the covariance of a between factor."""
    parts = match.group(0).split(" ")
    if match.group(1) is None:
        label = chr((int(parts[1]) & KEY_MASK) >> 56)
        if label in VERTEX_KEYS:
            key = "mesh_mesh"
        else:
            idx_1 = int(parts[1]) & ~KEY_MASK
            idx_2 = int(parts[2]) & ~KEY_MASK
            key = "loop_close" if abs(idx_1 - idx_2) > 1 else "odom"
    else:
        key = "place_edge"

    if key not in config:
        return match.group(0)

    parts[10] = str(100 * config[key])
    parts[16] = str(100 * config[key])
    parts[21] = str(100 * config[key])
    parts[25] = str(config[key])
    parts[28] = str(config[key])
    parts[30] = str(config[key])
    return " ".join(parts), key


def remap_valence_covariance(config, match):
    """Remap the covariance of a valence factor."""
    if match.group(1) is None:
        key = "pose_mesh"
    else:
        key = "place_mesh"

    if key not in config:
        return match.group(0)

    parts = match.group(0).split(" ")
    parts[13] = str(config[key])
    parts[16] = str(config[key])
    parts[18] = str(config[key])
    return " ".join(parts)


def main():
    """Run everything."""
    parser = argparse.ArgumentParser(description="modify dgraph file")
    parser.add_argument("filepath", help="file to use")
    parser.add_argument("--config", "-c", help="config to use", nargs="?", default=None)
    parser.add_argument("--output", "-o", help="output name", nargs="?", default=None)
    parser.add_argument(
        "--namespace",
        "-n",
        help="config namespace",
        nargs="?",
        default="pgmo/covariance",
    )
    args = parser.parse_args()

    filepath = pathlib.Path(args.filepath).expanduser().absolute()
    if not filepath.exists():
        print(f"ERROR: {filepath} does not exist")
        sys.exit(1)

    if args.config is None:
        covars = {}
    else:
        configpath = pathlib.Path(args.config).expanduser().absolute()
        if not configpath.exists():
            print(f"ERROR: {configpath} does not exist")
            sys.exit(1)

        with configpath.open("r") as fin:
            covars = yaml.load(fin.read(), Loader=yaml.SafeLoader)

    parts = args.namespace.split("/")
    for part in parts:
        if part in covars:
            covars = covars[part]
        else:
            covars = {}
            break

    config = {k: 1.0 / v for k, v in covars.items()}
    print("Precisions:")
    for k, v in config.items():
        print(f"  - {k}: {v}")

    print("")
    with filepath.open("r") as fin:
        contents = fin.read()

    if args.output is None:
        output_path = filepath.parent / f"{filepath.stem}_updated{filepath.suffix}"
    else:
        output_path = pathlib.Path(args.output).expanduser().absolute()

    print(f"Writing to {output_path}\n")

    num_temp_nodes = 0
    num_nodes = 0
    num_priors = 0
    num_lc = 0
    num_valence = 0
    num_temp_valence = 0
    num_odom = 0
    num_place_edges = 0
    num_mesh_edges = 0
    num_vertices = 0
    with output_path.open("w") as fout:
        for match in POSE_FINDER.finditer(contents):
            fout.write(f"{match.group(0)}\n")
            if match.group(1) is None:
                num_nodes += 1
            else:
                num_temp_nodes += 1

        for match in VERTEX_FINDER.finditer(contents):
            fout.write(f"{match.group(0)}\n")
            num_vertices += 1

        for match in PRIOR_FINDER.finditer(contents):
            new_line = remap_prior_covariance(config, match)
            fout.write(f"{new_line}\n")
            num_priors += 1

        for match in BETWEEN_FINDER.finditer(contents):
            new_line, key = remap_between_covariance(config, match)
            fout.write(f"{new_line}\n")
            if key == "odom":
                num_odom += 1
            elif key == "mesh_mesh":
                num_mesh_edges += 1
            elif key == "place_edge":
                num_place_edges += 1
            else:
                num_lc += 1

        for match in VALENCE_FINDER.finditer(contents):
            new_line = remap_valence_covariance(config, match)
            fout.write(f"{new_line}\n")
            if match.group(1) is None:
                num_valence += 1
            else:
                num_temp_valence += 1

    print("Wrote the following:")
    print(f"  - Nodes:          {num_nodes}")
    print(f"  - Nodes (temp):   {num_temp_nodes}")
    print(f"  - Vertices:       {num_vertices}")
    print(f"  - Priors:         {num_priors}")
    print(f"  - Loop-Closures:  {num_lc}")
    print(f"  - Valence:        {num_valence}")
    print(f"  - Valence (temp): {num_temp_valence}")
    print(f"  - Odometry:       {num_odom}")
    print(f"  - Mesh to Mesh:   {num_mesh_edges}")
    print(f"  - Place to Place: {num_place_edges}")


if __name__ == "__main__":
    main()
