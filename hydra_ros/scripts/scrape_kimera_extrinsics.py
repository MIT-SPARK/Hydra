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
from scipy.spatial.transform.rotation import Rotation as R
import numpy as np
import pathlib
import yaml
import sys


def main():
    """Scrape TF from kimera config and write to hydra format."""
    if len(sys.argv) < 3:
        print("missing required arguments!", file=sys.stderr)
        print(
            "usage: scrape_kimera_extrinsics.py PATH_TO_KIMERA_CONIFG PATH_TO_OUTPUT",
            file=sys.stderr,
        )
        sys.exit(1)

    input_path = pathlib.Path(sys.argv[1]).expanduser().absolute()
    output_path = pathlib.Path(sys.argv[2]).expanduser().absolute()

    with input_path.open("r") as fin:
        contents = [x for x in fin]
        # skip first line
        if "%YAML:1.0" in contents[0]:
            contents_str = "\n".join(contents[1:])
        else:
            contents_str = "\n".join(contents)
        config = yaml.load(contents_str, Loader=yaml.Loader)

    body_T_sensor = np.array([float(x) for x in config["T_BS"]["data"]]).reshape((4, 4))
    body_t_sensor = body_T_sensor[:3, 3]
    body_R_sensor = R.from_matrix(body_T_sensor[:3, :3]).as_quat().tolist()

    to_write = {
        "body_t_camera": body_t_sensor.tolist(),
        "body_R_camera": {
            "w": body_R_sensor[3],
            "x": body_R_sensor[0],
            "y": body_R_sensor[1],
            "z": body_R_sensor[2],
        },
    }
    with output_path.open("w") as fout:
        fout.write(yaml.dump(to_write))


if __name__ == "__main__":
    main()
