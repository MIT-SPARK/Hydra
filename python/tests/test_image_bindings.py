"""Test python image conversion."""
from hydra_python._hydra_bindings import (
    _get_color_image,
    _get_depth_image,
    _get_label_image,
)
import numpy as np
import pytest


def _get_img(dtype):
    return np.arange(50, dtype=dtype).reshape((5, 10))


def test_label_parsing():
    """Test that we always get an int32 image out when parsing labels."""
    for dtype in [np.uint8, np.int8, np.uint16, np.int16, np.uint32, np.int32]:
        labels_in = np.arange(50, dtype=dtype).reshape((5, 10))
        labels_out = np.array(_get_label_image(labels_in))
        assert labels_out.shape == (5, 10)
        assert labels_out.dtype == np.int32
        assert (labels_out == labels_in.astype(np.int32)).all()

    bad_labels = np.arange(150, dtype=np.uint8).reshape((5, 10, 3))
    assert _get_label_image(bad_labels) is None

    bad_labels = np.arange(150, dtype=np.float32).reshape((5, 30))
    assert _get_label_image(bad_labels) is None


def test_depth_parsing():
    """Test that we convert depth properly."""
    depth_in = np.arange(50, dtype=np.float32).reshape((5, 10))
    depth_out = np.array(_get_depth_image(depth_in))
    assert depth_out.shape == (5, 10)
    assert depth_out.dtype == np.float32
    assert depth_out == pytest.approx(depth_in)

    depth_in = np.arange(50, dtype=np.uint16).reshape((5, 10)) + 1
    depth_out = np.array(_get_depth_image(depth_in))
    assert depth_out.shape == (5, 10)
    assert depth_out.dtype == np.float32
    assert depth_out == pytest.approx(depth_in * 1.0e-3)

    bad_depth = np.arange(150, dtype=np.uint8).reshape((5, 10, 3))
    assert _get_depth_image(bad_depth) is None

    bad_depth = np.arange(150, dtype=np.int32).reshape((5, 30))
    assert _get_depth_image(bad_depth) is None


def test_mono_parsing():
    """Test that mono images are parsed into rgb correctly."""
    expected = np.stack(
        (_get_img(np.uint8), _get_img(np.uint8), _get_img(np.uint8)), axis=-1
    )

    mono_in = _get_img(np.float32) / 255.0
    color_out = np.array(_get_color_image(mono_in))
    assert color_out.shape == (5, 10, 3)
    assert color_out.dtype == np.uint8
    assert (expected == color_out).all()

    mono_in = _get_img(np.uint8)
    color_out = np.array(_get_color_image(mono_in))
    assert color_out.shape == (5, 10, 3)
    assert color_out.dtype == np.uint8
    assert (expected == color_out).all()

    mono_in = _get_img(np.uint16) * 255
    color_out = np.array(_get_color_image(mono_in))
    assert color_out.shape == (5, 10, 3)
    assert color_out.dtype == np.uint8
    assert (expected == color_out).all()


def test_rgb_parsing():
    """Test that rgb images are parsed correctly."""
    expected = np.stack(
        (_get_img(np.uint8) + 1, _get_img(np.uint8) + 2, _get_img(np.uint8) + 3),
        axis=-1,
    )
    rgb_out = np.array(_get_color_image(expected))
    assert rgb_out.shape == (5, 10, 3)
    assert rgb_out.dtype == np.uint8
    assert (expected == rgb_out).all()

    rgb_in = np.stack(
        (
            (_get_img(np.float32) + 1) / 255.0,
            (_get_img(np.float32) + 2) / 255.0,
            (_get_img(np.float32) + 3) / 255.0,
        ),
        axis=-1,
    )
    rgb_out = np.array(_get_color_image(rgb_in))
    assert rgb_out.shape == (5, 10, 3)
    assert rgb_out.dtype == np.uint8
    assert (expected == rgb_out).all()


def test_invalid_color_parsing():
    """Test that we catch invalid images."""
    assert _get_color_image(_get_img(np.uint32)) is None
    assert _get_color_image(np.stack([_get_img(np.uint8)] * 2, axis=-1)) is None
    assert _get_color_image(np.stack([_get_img(np.uint8)] * 4, axis=-1)) is None
    assert _get_color_image(np.stack([_get_img(np.uint16)] * 3, axis=-1)) is None


def test_invalid_tensor():
    """Test that we catch invalid shapes."""
    four_dim_tensor = np.arange(64, dtype=np.uint8).reshape((4, 4, 2, 2))
    assert _get_color_image(four_dim_tensor) is None
    assert _get_label_image(four_dim_tensor) is None
    assert _get_depth_image(four_dim_tensor) is None


def test_unsqueeze_tensor():
    """Test that images with arbitrary empty axes gets squeezed."""
    expected = np.stack(
        (_get_img(np.uint8) + 1, _get_img(np.uint8) + 2, _get_img(np.uint8) + 3),
        axis=-1,
    )
    assert (_get_color_image(expected) == expected).all()
    valid_image = expected[
        np.newaxis,
        :,
        np.newaxis,
        np.newaxis,
        :,
        np.newaxis,
        :,
        np.newaxis,
        np.newaxis,
        np.newaxis,
    ]
    assert (_get_color_image(valid_image) == expected).all()
