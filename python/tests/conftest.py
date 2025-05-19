"""Test fixtures for python binding tests."""

import hydra_python as hydra
import pytest


@pytest.fixture(autouse=True)
def setup_glog():
    """Set glog to be verbose."""
    hydra.set_glog_level(min_log_level=0, verbosity=0)
