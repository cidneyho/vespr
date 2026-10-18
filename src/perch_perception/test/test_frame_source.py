"""Tests for the camera-independent part of the frame source layer."""

import numpy as np

from fake_source import ListSource, make_frame
from perch_perception.frame_source import depth_to_meters


def test_depth_to_meters_applies_the_depth_scale():
    raw = np.array([[1000, 2500]], dtype=np.uint16)

    meters = depth_to_meters(raw, depth_scale=0.001)

    np.testing.assert_allclose(meters, [[1.0, 2.5]])


def test_depth_to_meters_keeps_zero_as_unknown():
    raw = np.array([[0, 1000]], dtype=np.uint16)

    meters = depth_to_meters(raw, depth_scale=0.001)

    assert meters[0, 0] == 0.0


def test_depth_to_meters_returns_float32():
    raw = np.array([[1000]], dtype=np.uint16)

    assert depth_to_meters(raw, depth_scale=0.001).dtype == np.float32


def test_a_frame_unpacks_as_the_agreed_tuple():
    frame = make_frame(timestamp=1.5)

    image, depth, intrinsics, timestamp, imu = frame

    assert timestamp == 1.5
    assert imu is None


def test_iterating_a_source_yields_its_frames_in_order():
    source = ListSource([make_frame(1.0), make_frame(2.0)])

    timestamps = [frame.timestamp for frame in source]

    assert timestamps == [1.0, 2.0]


def test_a_source_is_closed_when_its_with_block_exits():
    source = ListSource([])

    with source:
        assert not source.closed

    assert source.closed
