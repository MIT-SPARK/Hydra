#!/usr/bin/env python3
"""Script to check source file for license info."""
import difflib
import pathlib
import click


def get_filetype(extension):
    """Get filetype based on extension."""
    if extension in [".cpp", ".h"]:
        return "cpp"
    elif extension == ".py":
        return "python"
    else:
        return None


@click.command()
@click.argument("input_files", type=click.File("r"), nargs=-1)
@click.option("--verbose", "-v", is_flag=True, help="show files checked")
def main(input_files, verbose):
    """Check if header of file matches license."""
    parent_dir = pathlib.Path(__file__).absolute().parent
    headers = {}
    with (parent_dir / "cpp-header.txt").open("r") as fin:
        headers["cpp"] = fin.read()

    with (parent_dir / "python-header.txt").open("r") as fin:
        headers["python"] = fin.read()

    for input_file in input_files:
        ext = pathlib.Path(input_file.name).suffix
        filetype = get_filetype(ext)
        if not filetype:
            click.secho(f"unknown extension for {input_file.name}: {ext}", fg="yellow")
            continue

        target = headers[filetype]
        num_lines = len(target.split("\n")) - 1

        try:
            lines = [next(input_file) for _ in range(num_lines)]
        except StopIteration:
            click.secho(f"{input_file.name}: missing", fg="red")
            continue

        if filetype == "python":
            if lines and "#!" in lines[0]:
                lines = lines[1:]
                try:
                    lines.append(next(input_file))
                except StopIteration:
                    click.secho(f"{input_file.name}: missing", fg="red")
                    continue

        header = "".join(lines)
        if header != target:
            click.secho(f"{input_file.name}: bad", fg="red")
            if verbose:
                diff = difflib.context_diff(
                    header.splitlines(keepends=True),
                    target.splitlines(keepends=True),
                    fromfile=input_file.name,
                    tofile="license",
                )
                diff_str = "".join(diff)
                click.secho(f"{diff_str}", fg="red")
        elif verbose:
            click.secho(f"{input_file.name}: good", fg="green")


if __name__ == "__main__":
    main()
