import contextlib
import pathlib
import shutil
import sqlite3
import time
import uuid
from dataclasses import dataclass
from typing import Any

import spark_config as sc

from hydra_eval.utils import get_logger


@dataclass
class ResultEntry:
    """Entry in the result table."""

    path: pathlib.Path
    name: str
    date: str


class ResultManager:
    """Class that manages sessions."""

    def __init__(self, output_path):
        self._output_path = pathlib.Path(output_path).expanduser().resolve()
        self._fields = "uuid, date, experiment_name"
        self._db = None

    def __enter__(self):
        self._output_path.mkdir(exist_ok=True, parents=True)
        try:
            self._db = sqlite3.connect(str(self._output_path / ".results.db"))
        except sqlite3.OperationalError as e:
            get_logger().critical(f"Failed to open '{self._output_path}': {e}")
            raise e

        self._db.execute(f"CREATE TABLE IF NOT EXISTS results({self._fields})")
        return self

    def __exit__(self, exc, exc_type, exc_str):
        """Close the result manager."""
        if self._db is not None:
            self._db.close()
            self._db = None

    @contextlib.contextmanager
    def open_result(self, name):
        if self._db is None:
            get_logger().critical("Result manager not initialized!")
            raise RuntimeError("DB not initialized!")

        ident = str(uuid.uuid4())
        result_path = self._output_path / ident
        result_path.mkdir()

        try:
            yield result_path
        except Exception as e:
            shutil.rmtree(result_path)
            raise e

        try:
            values = f"'{ident}', {int(time.time())}, '{name}'"
            self._db.execute(f"INSERT INTO results VALUES ({values})")
            self._db.commit()
        except Exception as e:
            shutil.rmtree(result_path)
            raise e

    @property
    def results(self):
        if self._db is None:
            get_logger().critical("Result manager not initialized!")
            raise RuntimeError("DB not initialized!")

        cur = self._db.cursor()
        for row in cur.execute(f"SELECT {self._fields} FROM results ORDER BY date"):
            yield ResultEntry(
                path=self._output_path / row[0],
                name=row[2],
                date=time.asctime(time.localtime(row[1])),
            )


@dataclass
class ExperimentManagerConfig(sc.Config):
    """Configuration for experiments."""

    name: str = ""
    place_method: Any = sc.config_field("place_model", default="Salad")
    descriptor_method: Any = sc.config_field("descriptor_model", required=False)

    @classmethod
    def load(cls, path: str):
        return sc.Config.load(ExperimentManagerConfig, path)


class ExperimentManager:
    """Class that manages experiments."""

    def __init__(self, result_manager, config):
        self._results = result_manager
        self._config = config

    @classmethod
    def from_file(cls, result_manager, config_path):
        logger = get_logger()
        config_path = pathlib.Path(config_path).expanduser().absolute()
        config = ExperimentManagerConfig.load(config_path)
        if config is None:
            logger.error(f"Failed to load config for experiment from '{config_path}'")
            return None

        if config.name == "":
            logger.error("Experiment name is required!")
            return None

        return cls(result_manager, config_path)

    def run(self):
        """Run all experiments."""
        with self._results.open_result(self._config.name) as result_path:
            print(result_path)


class TmuxpExperiment:
    def __init__(self, config):
        """Run an experiment from a tmuxp file."""
        self._config = config

    @property
    def config(self):
        """Get underlying config."""
        return self.config

    def run(self):
        pass
