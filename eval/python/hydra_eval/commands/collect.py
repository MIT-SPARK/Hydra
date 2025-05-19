"""Command to run hydra and collect results."""

import click
from hydra_eval.result_collection import ExperimentManager, ResultManager


@click.group(name="collect")
def cli():
    """Display info about Hydra timing in various ways."""
    pass


@cli.command(name="run")
@click.argument("output_path", type=click.Path())
@click.argument("experiments", type=click.Path(exists=True), nargs=-1)
def run(output_path, experiments):
    """Collect a new set of Hydra results."""
    with ResultManager(output_path) as manager:
        for experiment_path in experiments:
            experiment = ExperimentManager.from_file(manager, experiment_path)
            if manager is None:
                continue

            experiment.run()


@cli.command(name="show")
@click.argument("output_path", type=click.Path())
def show(output_path):
    """Collect a new set of Hydra results."""
    with ResultManager(output_path) as manager:
        cur = manager.db.cursor()
        for row in cur.execute(
            "SELECT uuid, date, experiment_name FROM results ORDER BY date"
        ):
            print(row)
