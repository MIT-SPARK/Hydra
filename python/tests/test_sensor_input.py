"""Test sensor input validation."""
import hydra_python as hydra
import numpy as np
import pytest


def _get_img(w, h, dtype):
    return np.arange(w * h, dtype=dtype).reshape(w, h)


def test_image_input():
    """Test that get valid and invalid inputs depending on images supplied."""
    # we need either color or labels
    packet = hydra.PythonSensorInput.from_images(0, _get_img(5, 10, np.uint16))
    assert not packet

    # mismatched shapes mean no conversion
    packet = hydra.PythonSensorInput.from_images(
        0, _get_img(5, 10, np.uint16), colors=_get_img(10, 5, np.uint8)
    )
    assert not packet

    # mismatched shapes mean no conversion
    packet = hydra.PythonSensorInput.from_images(
        0, _get_img(5, 10, np.uint16), labels=_get_img(10, 5, np.int32)
    )
    assert not packet

    # valid labels and depth -> valid packet
    packet = hydra.PythonSensorInput.from_images(
        0, _get_img(5, 10, np.uint16), labels=_get_img(5, 10, np.int32)
    )
    assert packet

    # matching color and depth -> valid packet
    packet = hydra.PythonSensorInput.from_images(
        0, _get_img(5, 10, np.uint16), colors=_get_img(5, 10, np.uint8)
    )
    assert packet


def test_cloud_input():
    """Test that get valid and invalid inputs depending on pointcloud data supplied."""
    # we need either color or labels
    packet = hydra.PythonSensorInput.from_points(0, np.zeros((3, 10), np.float64))
    assert not packet

    # no points also fails
    packet = hydra.PythonSensorInput.from_points(
        0, np.zeros((3, 0), np.float64), colors=np.zeros((3, 0), np.uint8)
    )
    assert not packet

    # mismatched dimensions not supported
    with pytest.raises(TypeError):
        packet = hydra.PythonSensorInput.from_points(0, np.zeros((2, 10), np.float64))

    # mismatched numbers of labels means no conversion
    packet = hydra.PythonSensorInput.from_points(
        0, np.zeros((3, 10), np.float64), labels=np.zeros((1, 5), np.int32)
    )
    assert not packet

    # mismatched numbers of colors means no conversion
    packet = hydra.PythonSensorInput.from_points(
        0, np.zeros((3, 10), np.float64), colors=np.zeros((3, 5), np.uint8)
    )
    assert not packet

    # matching labels and points -> valid packet
    packet = hydra.PythonSensorInput.from_points(
        0, np.zeros((3, 10), np.float64), labels=np.zeros((1, 10), np.int32)
    )
    assert packet

    # matching colors and points -> valid packet
    packet = hydra.PythonSensorInput.from_points(
        0, np.zeros((3, 10), np.float64), colors=np.zeros((3, 10), np.uint8)
    )
    assert packet
