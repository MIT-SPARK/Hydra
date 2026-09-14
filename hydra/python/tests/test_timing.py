"""Timing file and standalone CLI behavior."""

import numpy as np
from click.testing import CliRunner
from hydra_python import timing
from hydra_python.commands.main import cli


def test_empty_and_single_sample(tmp_path):
    """Timing files preserve two columns even with zero or one sample."""
    tmp_path = tmp_path / "timing"
    tmp_path.mkdir()
    (tmp_path / "empty_timing_raw.csv").write_text("timestamp(ns),elapsed(s)\n")
    (tmp_path / "single_timing_raw.csv").write_text(
        "timestamp(ns),elapsed(s)\n1000000000,0.002\n"
    )
    data = timing.collect_timing_info(tmp_path)
    assert data["empty"].shape == (0, 2)
    np.testing.assert_array_equal(data["single"], [[1e9, 0.002]])
    result = CliRunner().invoke(cli, ["timing", "show", str(tmp_path)])
    assert result.exit_code == 0, result.output
    assert "2.000" in result.output
    assert "single" in result.output


def test_collation_timestamp_tolerance():
    """Collation matches nanosecond timestamps and omits unmatched samples."""
    data = {
        "a": np.array([[1e9, 0.1], [2e9, 0.2]]),
        "b": np.array([[1e9 + 10, 0.3], [2e9 + 100, 0.4]]),
    }
    stamps, elapsed = timing.collate_timers(data, ["a", "b"], max_diff_ns=20)
    np.testing.assert_array_equal(stamps, [1e9])
    np.testing.assert_allclose(elapsed, [0.4])
    data["b"] = np.empty((0, 2))
    assert timing.collate_timers(data, ["a", "b"])[0].size == 0
