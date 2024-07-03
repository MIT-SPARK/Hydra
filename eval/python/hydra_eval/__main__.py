"""Entry points for hydra."""

import hydra_eval.commands.timing as timing
import click
import logging


class ClickHandler(logging.Handler):
    """Logging handler to color output using click."""

    def emit(self, record):
        """Send log record to console with appropriate coloring."""
        msg = self.format(record)

        if record.levelno <= logging.DEBUG:
            click.secho(msg, fg="green")
            return

        if record.levelno <= logging.INFO:
            click.echo(msg)
            return

        if record.levelno <= logging.WARNING:
            click.secho(msg, fg="yellow", err=True)
            return

        click.secho(msg, fg="red", err=True)


def setup_log_file(result_dir):
    """Set logging to also use log file."""
    handler = logging.FileHandler(result_dir / "odsg_eval.log", mode="w")
    handler.setLevel(logging.DEBUG)
    formatter = logging.Formatter(
        "[%(levelname)s] %(asctime)s @ %(filename)s:%(lineno)s: %(message)s"
    )
    handler.setFormatter(formatter)
    logger = logging.getLogger()
    logger.addHandler(handler)


@click.group()
def cli():
    """Entry point target for subcommands."""
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)

    # TODO(nathan) pass config from commands
    handler = ClickHandler()
    handler.setLevel(logging.INFO)
    formatter = logging.Formatter("%(message)s")
    handler.setFormatter(formatter)

    logger.handlers.clear()
    logger.addHandler(handler)


cli.add_command(timing.cli)


if __name__ == "__main__":
    cli()
