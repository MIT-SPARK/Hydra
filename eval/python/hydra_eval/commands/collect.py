"""Command to run hydra and collect results."""

import pathlib
import pprint
import shutil

import click
from hydra_eval.result_collection import ExperimentManager, ResultManager


@click.group(name="collect")
def cli():
    """Run and manage experimental results."""
    pass


@cli.command(name="run")
@click.argument("output_path", type=click.Path())
@click.argument("experiments", type=click.Path(exists=True), nargs=-1)
@click.option("--skip-existing", "-s", is_flag=True)
def run(output_path, experiments, skip_existing):
    """Collect a new set of Hydra results."""
    with ResultManager(output_path) as manager:
        for experiment_path in experiments:
            experiment = ExperimentManager.from_file(manager, experiment_path)
            if manager is None:
                continue

            experiment.run(skip_existing)


@cli.command(name="list")
@click.argument("result_path", type=click.Path(exists=True))
def list(result_path):
    """List all results in the result collection."""
    with ResultManager(result_path) as manager:
        for result in manager.results:
            pprint.pprint(result)


@cli.command(name="find")
@click.argument("result_path", type=click.Path(exists=True))
@click.argument("experiment_name", type=str)
@click.argument("trial_name", type=str)
def find(result_path, experiment_name, trial_name):
    """Find all matching results for a given trial."""
    with ResultManager(result_path) as manager:
        header = f"Results for {trial_name}:{experiment_name}:"
        print(header)
        print("-" * len(header))
        results = manager.find_trials(experiment_name, trial_name)
        for result in results:
            pprint.pprint(result)


@cli.command(name="drop")
@click.argument("result_path", type=click.Path(exists=True))
@click.argument("condition", type=str)
def drop(result_path, condition):
    """Drop all results matching condition from collection."""
    with ResultManager(result_path) as manager:
        print("Would drop:")
        print("-----------")
        for result in manager.find(condition):
            pprint.pprint(result)

        click.confirm("Drop results?", abort=True)
        manager.drop(condition)


@cli.command(name="export")
@click.argument("result_path", type=click.Path(exists=True))
@click.argument("output_path", type=click.Path())
@click.argument("condition", type=str)
def export(result_path, output_path, condition):
    """Save results matching condition to a directory."""
    output_path = pathlib.Path(output_path).expanduser().absolute()
    if output_path.exists():
        click.confirm(f"Remove existing '{output_path}'?", abort=True)
        shutil.rmtree(output_path)

    output_path.mkdir(parents=True)
    with ResultManager(result_path) as manager:
        for result in manager.find(condition):
            hydra_path = result.path / "hydra"
            required = [
                "hydra_config.yaml",
                "frontend/dsg_with_mesh.json",
                "backend/deformation_graph.dgrf",
            ]
            required = [hydra_path / x for x in required]
            missing = [str(x) for x in required if not x.exists()]
            if len(missing) > 0:
                result_name = f"{result.name}:{result.trial_name}"
                missing_str = pprint.pformat(missing)
                click.secho(
                    f"Missing files for {result_name}:\n{missing_str}",
                    fg="yellow",
                )
                continue

            exp_path = output_path / result.name / result.trial_name
            exp_path.mkdir(parents=True)
            dest_paths = [exp_path / x.relative_to(hydra_path) for x in required]
            for src, dest in zip(required, dest_paths):
                dest.parent.mkdir(exist_ok=True, parents=True)
                shutil.copy2(src, dest)
