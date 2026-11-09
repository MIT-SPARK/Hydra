"""hydra_python.image_store needs neither the compiled bindings nor OpenCV."""

import subprocess
import sys

import hydra_python
import pytest


def _run(code):
    return subprocess.run(
        [sys.executable, "-c", code], capture_output=True, text=True, check=False
    )


def test_image_store_without_bindings():
    """Without the bindings, importing hydra_python warns once and the image store
    works (the bindings are hidden even when they are built)."""
    out = _run(
        """
import sys, warnings
sys.modules["hydra_python._hydra_bindings"] = None  # import -> ImportError
with warnings.catch_warnings(record=True) as caught:
    warnings.simplefilter("always")
    import hydra_python
    from hydra_python.image_store import ImageStore, make_id
names = [type(w.message).__name__ for w in caught]
assert names.count("BindingsNotBuiltWarning") == 1, names
assert hydra_python.BINDINGS_ERROR is not None
assert make_id(1, 2) == (1 << 48) | 2
assert "cv2" not in {k for k, v in sys.modules.items() if v is not None}
"""
    )
    assert out.returncode == 0, out.stderr


def test_python_m_image_store():
    out = _run(
        "import runpy, sys; sys.argv = ['x', '--help']; "
        "runpy.run_module('hydra_python.image_store', run_name='__main__')"
    )
    assert out.returncode == 0, out.stderr
    for command in ("index", "check", "info", "export"):
        assert command in out.stdout


def test_hydra_cli_has_image_store():
    """``hydra image-store ...`` (the hydra CLI also imports matplotlib/seaborn)."""
    pytest.importorskip("matplotlib")
    pytest.importorskip("seaborn")
    from click.testing import CliRunner
    from hydra_python.commands.main import cli

    result = CliRunner().invoke(cli, ["image-store", "--help"])
    assert result.exit_code == 0, result.output
    assert "index" in result.output and "check" in result.output


def test_bindings_state_is_exposed():
    assert hasattr(hydra_python, "BINDINGS_ERROR")
