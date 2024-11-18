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
import logging
import pathlib
import pprint

import yaml

from hydra_python._hydra_bindings import HydraPipeline, set_glog_dir


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


def update_nested(contents, other):
    """Merge two config trees recursively."""
    for key, value in other.items():
        if key not in contents:
            contents[key] = {}

        if isinstance(value, dict):
            update_nested(contents[key], value)
        else:
            contents[key] = value


def load_configs(
    dataset: str, labelspace: str = "ade20k_mp3d", bounding_box_type: str = "AABB"
):
    """
    Load various configs to construct the Hydra pipeline.

    dataset_name: Dataset name to load config from
    labelspace_name: Labelspace name to use
    bounding_box_type: Type of bounding box to use

    Returns:
        (Optional[PythonConfig]) Pipline config or none if invalid
    """
    config_path = get_config_path()
    dataset_path = config_path / "datasets" / f"{dataset}.yaml"
    labelspace_path = config_path / "label_spaces" / f"{labelspace}_label_space.yaml"
    if not dataset_path.exists():
        logging.error(f"invalid dataset path: {dataset_path}")
        return None

    if not labelspace_path.exists():
        logging.error(f"invalid labelspace path: {labelspace_path}")
        return None

    contents = {}
    with dataset_path.open("r") as fin:
        update_nested(contents, yaml.safe_load(fin.read()))

    with labelspace_path.open("r") as fin:
        update_nested(contents, yaml.safe_load(fin.read()))

    overrides = {
        "frontend": {
            "type": "GraphBuilder",
            "objects": {"bounding_box_type": bounding_box_type},
        },
        "backend": {"type": "BackendModule"},
        "reconstruction": {
            "type": "ReconstructionModule",
            "show_stats": False,
            "pose_graphs": {"make_pose_graph": True},
        },
        "lcd": {"lcd_use_bow_vectors": False},
    }
    update_nested(contents, overrides)
    return contents


def load_pipeline(
    data,
    config_name,
    labelspace,
    output_path=None,
    config_verbosity=0,
    place_feature_strategy=None,
    zmq_url=None,
):
    """Load Hydra pipeline from configs."""
    contents = load_configs(config_name, labelspace=labelspace)
    if not contents:
        return None

    if output_path:
        update_nested(contents, {"log_path": str(output_path)})
        glog_dir = output_path / "logs"
        if not glog_dir.exists():
            glog_dir.mkdir(parents=True)

        set_glog_dir(glog_dir)

    if place_feature_strategy is not None:
        update_nested(
            contents,
            {
                "frontend": {
                    "view_database": {"view_selection_method": place_feature_strategy}
                }
            },
        )

    logging.debug(pprint.pformat(contents, sort_dicts=False))
    pipeline = HydraPipeline.from_config(
        yaml.safe_dump(contents),
        data.sensor,
        robot_id=0,
        config_verbosity=config_verbosity,
        zmq_url="" if zmq_url is None else zmq_url
    )

    return pipeline
