"""Command to run hydra and collect results."""

import pprint

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
@click.argument("output_path", type=click.Path(exists=True))
def list(output_path):
    """Collect a new set of Hydra results."""
    with ResultManager(output_path) as manager:
        for result in manager.results:
            pprint.pprint(result)


@cli.command(name="find")
@click.argument("output_path", type=click.Path(exists=True))
@click.argument("experiment_name", type=str)
@click.argument("trial_name", type=str)
def find(output_path, experiment_name, trial_name):
    """Collect a new set of Hydra results."""
    with ResultManager(output_path) as manager:
        header = f"Results for {trial_name}:{experiment_name}:"
        print(header)
        print("-" * len(header))
        results = manager.find_trials(experiment_name, trial_name)
        for result in results:
            pprint.pprint(result)
