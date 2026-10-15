"""Tests for the RealSense frame sources.

The bag tests replay a synthetic bag (see synthetic_bag.py), so they need
pyrealsense2 but no camera. They are skipped where pyrealsense2 is missing.
"""

import numpy as np
import pytest

pytest.importorskip('pyrealsense2')

import synthetic_bag as bag  # noqa: E402
from perch_perception.frame_source import Intrinsics  # noqa: E402
from perch_perception.realsense_source import (  # noqa: E402
    BagSource,
    default_fps,
    require_imu_support,
)


@pytest.fixture(scope='module')
def frames(tmp_path_factory):
    """Every frame BagSource yields from the synthetic bag."""
    path = tmp_path_factory.mktemp('bags') / 'synthetic.bag'
    bag.write_synthetic_bag(path)
    with BagSource(path) as source:
        return list(source)


def test_bag_source_yields_every_recorded_frame_once(frames):
    # The fixture keeps all frames in a list. This fails if the source hands
    # out librealsense's buffers without copying, because librealsense then
    # runs out of buffers and drops frames.
    assert len(frames) == bag.FRAME_COUNT


def test_bag_source_depth_is_in_meters(frames):
    # Frame i was recorded FIRST_DEPTH_RAW + i * DEPTH_STEP_RAW raw units away.
    expected = [1.0 + 0.1 * i for i in range(bag.FRAME_COUNT)]

    measured = [frame.depth[1, 0] for frame in frames]

    np.testing.assert_allclose(measured, expected, rtol=1e-5)


def test_bag_source_keeps_missing_depth_as_zero(frames):
    assert np.all(frames[0].depth[0, :] == 0.0)


def test_bag_source_image_is_the_left_ir_frame(frames):
    image = frames[3].image

    assert image.shape == (bag.HEIGHT, bag.WIDTH)
    assert image.dtype == np.uint8
    assert np.all(image == 3)


def test_bag_source_timestamps_are_in_seconds(frames):
    period = 1.0 / bag.FPS
    expected = [5.0 + i * period for i in range(bag.FRAME_COUNT)]

    np.testing.assert_allclose([frame.timestamp for frame in frames], expected)


def test_bag_source_reports_the_recorded_intrinsics(frames):
    assert frames[0].intrinsics == Intrinsics(
        width=bag.WIDTH, height=bag.HEIGHT,
        fx=bag.FX, fy=bag.FY, cx=bag.CX, cy=bag.CY,
        distortion_model='brown_conrady', distortion_coeffs=(0.0,) * 5,
    )


def test_bag_source_has_no_imu_sample_when_the_bag_has_no_imu(frames):
    assert all(frame.imu is None for frame in frames)


def test_bag_source_names_a_missing_file(tmp_path):
    missing = tmp_path / 'nope.bag'

    with pytest.raises(FileNotFoundError, match='nope.bag'):
        BagSource(missing)


def test_default_fps_is_30_on_usb3():
    assert default_fps('3.2') == 30


def test_default_fps_drops_to_10_on_usb2():
    assert default_fps('2.1') == 10


def test_imu_is_refused_on_macos():
    with pytest.raises(RuntimeError, match='macOS'):
        require_imu_support('darwin')


def test_imu_is_allowed_on_linux():
    require_imu_support('linux')
