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
"""Quick script to convert semantic color configuration to a label space."""
import click
import pathlib
import yaml


HELP_TEXT = """n: normal (default)
o: object
d: dynamic
s: surface
x: invalid
?: help """


def _get_output_name(label_grouping_file, name):
    if name:
        return f"{name}_label_space.yaml"

    group_name = label_grouping_file.stem
    return f"{group_name}_label_space.yaml"


def _get_output_directory(output_dir):
    config_dir = pathlib.Path(__file__).absolute().parent.parent / "config"
    label_space_path = config_dir / "label_spaces"
    if not output_dir:
        return label_space_path

    output_path = pathlib.Path(output_dir).expanduser().absolute()
    if not output_path.exists():
        click.secho(
            f"{output_path} does not exist: defaulting to {label_space_path}!",
            fg="yellow",
        )
        return label_space_path

    return output_path


def _format_list(name, values, collapse=True, **kwargs):
    indent = kwargs.get("indent", 0)
    prefix = " " * indent + name if indent > 0 else name
    if collapse:
        indent += 2

    args = {
        k: v for k, v in kwargs.items() if k != "indent" and k != "default_flow_style"
    }
    args["indent"] = indent
    value_str = yaml.dump(values, default_flow_style=collapse, **args)
    return f"{prefix}: {value_str}"


def _get_choice(class_name, skip_label, prev_choice=None):
    prompt_kwargs = {
        "type": click.Choice(["n", "o", "d", "s", "x", "?"]),
        "show_choices": True,
        "prompt_suffix": " >>> ",
        "default": "n",
    }

    if prev_choice in skip_label:
        click.secho(f"{class_name} →  {prev_choice} (prev)", fg="green")
        return prev_choice

    prompt_prefix = f"{class_name}"
    if prev_choice is not None:
        prompt_prefix += f" <previously: {prev_choice}>"
        prompt_kwargs["default"] = prev_choice

    value = click.prompt(prompt_prefix, **prompt_kwargs)
    while value == "?":
        click.secho(HELP_TEXT, fg="green")
        value = click.prompt(prompt_prefix, **prompt_kwargs)

    return value


def _load_prev(output_path):
    prev_config_map = {}
    if not output_path.exists():
        return prev_config_map

    with output_path.open("r") as fin:
        contents = yaml.load(fin.read(), Loader=yaml.SafeLoader)

    for prev_dynamic in contents.get("dynamic_labels", []):
        prev_config_map[int(prev_dynamic)] = "d"

    for prev_invalid in contents.get("invalid_labels", []):
        prev_config_map[int(prev_invalid)] = "x"

    if "frontend" not in contents:
        return prev_config_map

    prev_objects = contents["frontend"].get("objects", {}).get("labels", [])
    for prev_object in prev_objects:
        prev_config_map[int(prev_object)] = "o"

    prev_surfaces = contents["frontend"].get("surface_places", {}).get("labels", [])
    for prev_surface in prev_surfaces:
        prev_config_map[int(prev_surface)] = "s"

    return prev_config_map


@click.command()
@click.argument("label_grouping_file", type=click.Path(exists=True))
@click.option(
    "-p",
    "--measurement_probability",
    default=0.9,
    help="semantic measurement probability",
)
@click.option("-n", "--name", default=None, help="label space name")
@click.option("-o", "--output-dir", help="directory to output to")
@click.option("-f", "--force-overwrite", help="don't load previous labelspace")
@click.option(
    "-s",
    "--skip-label",
    multiple=True,
    default=["x", "d", "o"],
    help="label types to skip on second pass",
)
def main(
    label_grouping_file,
    measurement_probability,
    name,
    output_dir,
    force_overwrite,
    skip_label,
):
    """Parse and export labelspace."""
    label_grouping_file = pathlib.Path(label_grouping_file).expanduser().absolute()
    output_name = _get_output_name(label_grouping_file, name)
    output_dir = _get_output_directory(output_dir)
    output_path = output_dir / output_name
    click.echo(f"output: {output_path}")

    prev_config = {}
    if not force_overwrite:
        prev_config = _load_prev(output_path)

    with label_grouping_file.open("r") as fin:
        config = yaml.load(fin.read(), Loader=yaml.SafeLoader)

    invalid_labels = []
    surface_labels = []
    dynamic_labels = []
    object_labels = []
    output_names = []
    for class_id, group in enumerate(config["groups"]):
        class_name = group["name"]
        output_names.append({"label": class_id, "name": class_name})

        value = _get_choice(
            class_name, skip_label, prev_choice=prev_config.get(class_id)
        )
        if value == "o":
            object_labels.append(class_id)
        if value == "d":
            dynamic_labels.append(class_id)
        if value == "s":
            surface_labels.append(class_id)
        if value == "x":
            invalid_labels.append(class_id)

    with output_path.open("w") as fout:
        fout.write("---\n")
        fout.write(
            yaml.dump({"semantic_measurement_probability": measurement_probability})
        )
        fout.write(yaml.dump({"total_semantic_labels": len(config["groups"])}))
        fout.write(_format_list("dynamic_labels", dynamic_labels))
        fout.write(_format_list("invalid_labels", invalid_labels))
        fout.write("frontend:\n")
        fout.write("  objects:\n")
        fout.write(_format_list("labels", object_labels, indent=4))
        fout.write("  surface_places:\n")
        fout.write(_format_list("labels", surface_labels, indent=4))
        fout.write("label_names:\n")
        for name in output_names:
            fout.write("  - " + yaml.dump(name, default_flow_style=True))


if __name__ == "__main__":
    main()
