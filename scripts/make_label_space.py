"""Quick script to convert semantic color configuration to a label space."""
import click
import pathlib
import yaml


HELP_TEXT = """n: normal (default)
o: object
d: dynamic
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


def _get_choice(class_name):
    prompt_kwargs = {
        "type": click.Choice(["n", "o", "d", "x", "?"]),
        "show_choices": True,
        "prompt_suffix": " >>> ",
        "default": "n",
    }

    value = click.prompt(f"{class_name}", **prompt_kwargs)
    while value == "?":
        click.secho(HELP_TEXT, fg="green")
        value = click.prompt(f"{class_name}", **prompt_kwargs)

    return value


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
def main(label_grouping_file, measurement_probability, name, output_dir):
    """Parse and export labelspace."""
    label_grouping_file = pathlib.Path(label_grouping_file).expanduser().absolute()
    output_name = _get_output_name(label_grouping_file, name)
    output_dir = _get_output_directory(output_dir)
    output_path = output_dir / output_name
    click.echo(f"output: {output_path}")

    with label_grouping_file.open("r") as fin:
        config = yaml.load(fin.read(), Loader=yaml.SafeLoader)

    invalid_labels = []
    dynamic_labels = []
    object_labels = []
    output_names = []
    for class_id, group in enumerate(config["groups"]):
        class_name = group["name"]
        output_names.append({"label": class_id, "name": class_name})

        value = _get_choice(class_name)
        if value == "o":
            object_labels.append(class_id)
        if value == "d":
            dynamic_labels.append(class_id)
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
        fout.write("objects:\n")
        fout.write(_format_list("labels", object_labels, indent=2))
        fout.write("label_names:\n")
        for name in output_names:
            fout.write("  - " + yaml.dump(name, default_flow_style=True))


if __name__ == "__main__":
    main()
