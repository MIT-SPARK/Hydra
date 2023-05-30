#!/usr/bin/env python3
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
