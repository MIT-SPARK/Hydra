import pathlib

import pytest

import hydra_image_store

CPP_SCHEMA = (
    pathlib.Path(__file__).resolve().parents[2]
    / "hydra/resources/image_store/schema.sql"
)


@pytest.mark.skipif(not CPP_SCHEMA.exists(), reason="not in a Hydra checkout")
def test_schema_matches_hydra():
    """The packaged schema is a copy of Hydra's canonical schema."""
    packaged = pathlib.Path(hydra_image_store.__file__).with_name("schema.sql")
    assert packaged.read_text() == CPP_SCHEMA.read_text()
