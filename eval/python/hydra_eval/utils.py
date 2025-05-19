"""Various logging related utilities."""

import logging
import pathlib

import click


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


def get_logger(name="hydra_eval"):
    """Get logger for package."""
    return logging.getLogger(name)


def setup_log_file(result_dir, logger=None):
    """Set logging to also use log file."""
    result_dir = pathlib.Path(result_dir).expanduser().absolute()
    handler = logging.FileHandler(result_dir / "hydra_eval.log", mode="w")
    handler.setLevel(logging.DEBUG)
    formatter = logging.Formatter(
        "[%(levelname)s] %(asctime)s @ %(filename)s:%(lineno)s: %(message)s"
    )
    handler.setFormatter(formatter)
    if logger is None:
        logger = get_logger()

    logger.addHandler(handler)
