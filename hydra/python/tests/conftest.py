"""Test fixtures for python binding tests."""

import hydra_python as hydra
import pytest


@pytest.fixture(autouse=True)
def setup_logging():
    """Configure logging (when the bindings are built)."""
    if hydra.BINDINGS_ERROR is None:
        hydra.set_log_to_stderr()
        hydra.set_log_level("info")
    yield
    if hydra.BINDINGS_ERROR is None:
        hydra.set_log_to_stderr()
