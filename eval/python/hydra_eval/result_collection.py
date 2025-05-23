import contextlib
import pathlib
import shutil
import sqlite3
import time
import uuid
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import spark_config as sc
from ruamel.yaml import YAML

from hydra_eval.utils import get_logger

yaml = YAML(typ="safe")


@dataclass
class ResultEntry:
    """Entry in the result table."""

    path: pathlib.Path
    name: str
    trial_name: str
    date: str
    error_code: int

    @classmethod
    def from_row(cls, output_path, row):
        return cls(
            path=output_path / row[0],
            name=row[2],
            trial_name=row[3],
            date=time.asctime(time.localtime(row[1])),
            error_code=row[4],
        )


class ResultManager:
    """Class that manages sessions."""

    def __init__(self, output_path):
        self._output_path = pathlib.Path(output_path).expanduser().resolve()
        self._fields = "uuid, date, experiment_name, trial_name, error_code"
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
    def open_result(self, name, trial_name):
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
            values = f"'{ident}', {int(time.time())}, '{name}', '{trial_name}', -1"
            self._db.execute(f"INSERT INTO results VALUES ({values})")
            self._db.commit()
        except Exception as e:
            shutil.rmtree(result_path)
            raise e

    def find_trials(self, exp, trial):
        cond = f"experiment_name='{exp}' AND trial_name='{trial}'"
        return self.find(cond)

    def find(self, cond):
        query = self._db.execute(f"SELECT * FROM results WHERE {cond}")
        return [ResultEntry.from_row(self._output_path, x) for x in query.fetchall()]

    def drop(self, cond):
        to_remove = self.find(cond)
        get_logger().info(f"Dropping {len(to_remove)} entries!")
        for x in to_remove:
            if x.path.exists():
                shutil.rmtree(x.path)

        with self._db:
            self._db.execute(f"DELETE FROM results WHERE {cond}")

    def set_error_code(self, uuid, code):
        cur = self._db.cursor()
        cur.execute(
            f"UPDATE results SET error_code = {code} WHERE results.uuid = '{uuid}'"
        )
        self._db.commit()

    @property
    def results(self):
        if self._db is None:
            get_logger().critical("Result manager not initialized!")
            raise RuntimeError("DB not initialized!")

        cur = self._db.cursor()
        for row in cur.execute(f"SELECT {self._fields} FROM results ORDER BY date"):
            yield ResultEntry.from_row(self._output_path, row)


@dataclass
class TrialConfig(sc.Config):
    """Configuration for a single trial."""

    name: str = ""
    executable: Any = sc.config_field("exec")
    args: Dict[str, str] = field(default_factory=dict)


@dataclass
class ExperimentConfig(sc.Config):
    """Configuration for experiments."""

    name: str = ""
    trials: List[TrialConfig] = field(default_factory=list)

    @classmethod
    def load(cls, path: str):
        return sc.Config.load(ExperimentConfig, path)


class ExperimentManager:
    """Class that manages experiments."""

    def __init__(self, result_manager, config):
        self._results = result_manager
        self._config = config

    @classmethod
    def from_file(cls, result_manager, config_path):
        logger = get_logger()
        config_path = pathlib.Path(config_path).expanduser().absolute()
        config = ExperimentConfig.load(config_path)
        if config is None:
            logger.error(f"Failed to load config for experiment from '{config_path}'")
            return None

        if config.name == "":
            logger.error("Experiment name is required!")
            return None

        return cls(result_manager, config)

    def run(self, skip_existing):
        """Run all experiments."""
        for trial in self._config.trials:
            if skip_existing:
                existing = self._result.find_trials(self._config.name, trial.name)
                if len(existing) > 0:
                    get_logger().info(
                        f"Skipping existing {self._config.name}:{trial.name}"
                    )
                    continue

            with self._results.open_result(self._config.name, trial.name) as output:
                executable = trial.executable.create(trial.name, output, trial.args)
                ret = executable.run()

            self._results.set_error_code(output.stem, ret)


class LaunchExec:
    """Construct a subprocess command for tmuxp."""

    def __init__(self, config, name, result_path, args):
        """Set up subprocess command via config."""
        import launch

        ros_logs = result_path / "roslogs"
        ros_logs.mkdir(parents=True)
        launch.logging.launch_config.log_dir = str(ros_logs)

        self.config = config
        has_path = self.config.path is not None
        has_yaml = self.config.contents is not None
        if has_path and has_yaml:
            get_logger().error("Cannot specify both file and contents!")
            return

        if not has_path and not has_yaml:
            get_logger().error("Must specify either file or contents!")
            return

        rendered_path = result_path / f"{name}.launch.yaml"
        if has_path:
            norm_path = pathlib.Path(self.config.path).expanduser().resolve()
            shutil.copy2(norm_path, rendered_path)
        else:
            with rendered_path.open("w") as fout:
                yaml.dump(config.contents, fout)

        arg_list = list(args.items()) + [("output_path", str(result_path))]
        self._desc = launch.LaunchDescription(
            [
                launch.actions.IncludeLaunchDescription(
                    launch.launch_description_sources.AnyLaunchDescriptionSource(
                        str(rendered_path)
                    ),
                    launch_arguments=arg_list,
                )
            ]
        )

    def run(self):
        """Run command."""
        import launch

        serv = launch.LaunchService()
        serv.include_launch_description(self._desc)
        return serv.run()


@sc.register_config("exec", "launch", constructor=LaunchExec)
@dataclass
class LaunchExec(sc.Config):
    """Config for a tmuxp executable."""

    path: Optional[str] = None
    contents: Optional[Any] = None
