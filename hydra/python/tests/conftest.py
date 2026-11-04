"""Test fixtures for python binding tests."""

import hydra_python as hydra
import pytest


@pytest.fixture(autouse=True)
def setup_glog():
    """Set glog to be verbose (when the bindings are built)."""
    if hydra.BINDINGS_ERROR is None:
        hydra.set_glog_level(log_level=0, verbosity=0)
