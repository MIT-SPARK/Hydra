import pathlib
import struct

import numpy as np
import pytest
from hydra_image_store.codec import SAM3_TRACK_BASE

from hydra_image_store import (
    blob_to_transform,
    id_robot,
    id_seq,
    make_id,
    make_node_id,
    node_label,
    node_symbol,
    rle_decode,
    rle_encode,
    sam3_track_id,
    schema_sql,
    transform_to_blob,
)

CANONICAL_SCHEMA = (
    pathlib.Path(__file__).resolve().parents[2]
    / "hydra/resources/image_store/schema.sql"
)


@pytest.mark.skipif(
    not CANONICAL_SCHEMA.exists(), reason="canonical schema not available"
)
def test_packaged_schema_is_byte_identical_to_canonical():
    assert schema_sql().encode() == CANONICAL_SCHEMA.read_bytes()


def test_transform_blob_is_little_endian_row_major():
    T = np.arange(16, dtype=float).reshape(4, 4)
    blob = transform_to_blob(T)
    assert blob == struct.pack("<16d", *range(16))
    np.testing.assert_array_equal(blob_to_transform(blob), T)
    assert blob_to_transform(None) is None
    with pytest.raises(ValueError):
        blob_to_transform(b"\0" * 8)


def test_ids_and_symbols():
    fid = make_id(3, 42)
    assert fid == (3 << 48) | 42 and id_robot(fid) == 3 and id_seq(fid) == 42
    nid = make_node_id("O", 12)
    assert nid == 5692549928996306944 + 12
    assert node_symbol(nid) == ("O", 12) and node_label(nid) == "O_12"
    assert sam3_track_id(nid) == SAM3_TRACK_BASE | 12


@pytest.mark.parametrize("first", [False, True])
def test_rle_round_trip(first):
    rng = np.random.default_rng(0)
    mask = rng.random((17, 23)) > 0.6
    mask[0, 0] = first
    counts = rle_encode(mask)
    assert sum(counts) == mask.size
    assert (counts[0] == 0) == first  # always starts with a background run
    np.testing.assert_array_equal(rle_decode(counts, mask.shape), mask)
    np.testing.assert_array_equal(
        rle_decode(str(counts).replace(" ", ""), mask.shape), mask
    )


def test_rle_wrong_size_raises():
    with pytest.raises(ValueError):
        rle_decode([3, 4], (2, 2))
