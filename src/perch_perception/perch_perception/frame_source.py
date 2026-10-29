"""Frame sources: where the pipeline gets its camera frames.

Plain Python and NumPy only. Nothing here imports pyrealsense2, so every
module downstream of a source stays independent of the camera SDK. The
RealSense-specific sources live in realsense_source.py.

Units are meters and seconds. The camera frame is the OpenCV one (X right,
Y down, Z forward) with its origin at the optical center of the image stream.
"""

from abc import ABC, abstractmethod
from collections.abc import Iterator
from dataclasses import dataclass
from typing import NamedTuple

import numpy as np


@dataclass(frozen=True)
class Intrinsics:
    """Pinhole camera parameters of one stream, in pixels.

    fx, fy are the focal lengths and (cx, cy) is the principal point, the
    pixel where the optical axis meets the image.
    """

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    distortion_model: str
    distortion_coeffs: tuple[float, ...]


@dataclass(frozen=True)
class ImuSample:
    """One accelerometer and gyroscope reading."""

    accel: tuple[float, float, float]  # m/s^2
    gyro: tuple[float, float, float]  # rad/s
    timestamp: float  # seconds


class Frame(NamedTuple):
    """What a source yields for every camera frame.

    A NamedTuple, so it unpacks as (image, depth, intrinsics, timestamp, imu)
    and also reads by name.
    """

    image: np.ndarray  # H x W uint8, left infrared
    depth: np.ndarray  # H x W float32 meters; 0 means unknown, not 0 m
    intrinsics: Intrinsics  # of the image stream; depth shares them
    timestamp: float  # seconds
    imu: ImuSample | None  # None when the source has no IMU


def depth_to_meters(raw_depth, depth_scale):
    """Convert raw 16-bit depth to float32 meters.

    depth_scale is the camera's meters-per-unit factor (0.001 on a D435i).
    A raw 0 means "no depth here" and stays 0 after scaling, so callers
    still filter zeros before taking any statistic.
    """
    return raw_depth.astype(np.float32) * np.float32(depth_scale)


class FrameSource(ABC):
    """Base class for anything that produces frames: camera, bag, video."""

    @abstractmethod
    def frames(self) -> Iterator[Frame]:
        """Yield frames until the source ends."""

    def close(self):  # noqa: B027  (an intentional no-op default)
        """Release the camera or file. Safe to call more than once.

        The default does nothing, for sources that hold nothing to release.
        """

    def __iter__(self):
        return self.frames()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
