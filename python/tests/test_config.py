"""Test config collation."""
import hydra_python as hydra
import yaml


def test_simple_config():
    """Test that merging configs works as expected."""
    original_config = "{a: 5, b: 2.0}"
    config = hydra.PythonConfig()
    config.add_yaml(original_config)
    config.add_yaml(original_config, config_ns="test")
    expected = {"test": {"a": 5, "b": 2.0}, "a": 5, "b": 2.0}
    result = yaml.safe_load(str(config))
    assert expected == result


def test_simple_config_from_file(tmp_path):
    """Test that merging config files works as expected."""
    original_config = {"a": 5, "b": 2.0}
    config_path = tmp_path / "config.yaml"
    with config_path.open("w") as fout:
        fout.write(yaml.dump(original_config))

    config = hydra.PythonConfig()
    config.add_file(config_path)
    config.add_file(config_path, config_ns="test")
    expected = {"test": {"a": 5, "b": 2.0}, "a": 5, "b": 2.0}
    result = yaml.safe_load(str(config))
    assert expected == result


def test_joint_overlay(tmp_path):
    """Test that we can pull from multiple sources."""
    original_config = {"a": 5, "b": 2.0}
    config_path = tmp_path / "config.yaml"
    with config_path.open("w") as fout:
        fout.write(yaml.dump(original_config))

    del original_config["a"]
    original_config["b"] = 10.0
    original_config["foo"] = [1, 2, 3]

    overlay = hydra.ConfigOverlay(
        config_ns="test", contents=yaml.dump(original_config), filepath=config_path
    )
    config = hydra.PythonConfig([overlay])
    expected = {"test": {"a": 5, "b": 10.0, "foo": [1, 2, 3]}}
    result = yaml.safe_load(str(config))
    assert expected == result
