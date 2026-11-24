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
"""Module containing python utilites for loading a valid config."""

import json
import logging
import pathlib

from hydra_python._hydra_bindings import (
    Camera,
    CameraConfig,
    ExtrinsicsConfig,
    HydraPipeline,
    RunContext,
    init_config_context,
)


def get_config_path(dataset=None):
    """Return config directory for this package."""
    package_path = pathlib.Path(__file__).absolute().parent
    install_path = package_path / "config"
    if install_path.exists():
        return install_path

    devel_path = package_path.parent.parent.parent / "config"
    if devel_path.exists():
        return devel_path

    logging.error("failed to find config directory of package!")
    raise RuntimeError("invalid config path")


def load_configs(
    dataset: str,
    labelspace: str = "ade20k_mp3d",
    config_path=None,
    bounding_box_type: str = "AABB",
):
    """Build config_utilities arguments for the dataset and labelspace."""
    config_path = (
        get_config_path()
        if config_path is None
        else pathlib.Path(config_path).resolve()
    )
    dataset_path = config_path / "datasets" / f"{dataset}.yaml"
    labelspace_path = config_path / "label_spaces" / f"{labelspace}_label_space.yaml"
    for path in (dataset_path, labelspace_path):
        if not path.is_file():
            raise FileNotFoundError(path)

    return [
        "hydra",
        "--config-utilities-file",
        str(dataset_path),
        "--config-utilities-file",
        f"{labelspace_path}@labelspace",
        "--config-utilities-yaml",
        json.dumps({"frontend": {"objects": {"bounding_box_type": bounding_box_type}}}),
    ]


def make_camera(
    fx,
    fy,
    cx,
    cy,
    width,
    height,
    min_range=0.1,
    max_range=5.0,
    name="camera",
    extrinsics=None,
):
    config = CameraConfig()
    config.min_range = min_range
    config.max_range = max_range
    config.width = width
    config.height = height
    config.fx = fx
    config.fy = fy
    config.cx = cx
    config.cy = cy
    config.extrinsics = ExtrinsicsConfig() if extrinsics is None else extrinsics
    return Camera(config, name)


def load_pipeline(
    sensor,
    config_name,
    labelspace,
    config_path=None,
    output_path=None,
    config_verbosity=0,
    zmq_url=None,
):
    """Load Hydra pipeline from configs."""
    args = load_configs(config_name, labelspace=labelspace, config_path=config_path)
    if output_path is not None:
        args.extend(
            [
                "--config-utilities-yaml",
                json.dumps({"log_path": str(output_path)}),
            ]
        )

    if zmq_url is not None:
        args.extend(
            [
                "--config-utilities-yaml",
                json.dumps(
                    {"backend": {"sinks": [{"type": "ZmqSink", "url": zmq_url}]}}
                ),
            ]
        )

    init_config_context(args)
    context = RunContext()
    return HydraPipeline(sensor, context, show_config=config_verbosity > 0)
