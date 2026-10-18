"""RealSense frame sources: the live D435i and recorded .bag files.

This is the only module in the package that imports pyrealsense2. Both
sources convert RealSense frames to NumPy arrays and the plain dataclasses
from frame_source.py before yielding them, so nothing downstream ever sees
a RealSense type.

The image is the left infrared stream. Depth is computed in the left
imager's frame, so an image pixel and the depth pixel at the same (row,
column) are the same point in the scene. No alignment step is needed.
"""

import os
import sys

import numpy as np
import pyrealsense2 as rs

from perch_perception.frame_source import (
    Frame,
    FrameSource,
    ImuSample,
    Intrinsics,
    depth_to_meters,
)

LEFT_IR = 1  # RealSense numbers the infrared imagers 1 (left) and 2 (right)
USB3_FPS = 30
USB2_FPS = 10  # the most a D435i delivers at 848x480 over USB 2


_context = None


def _shared_context():
    """Return the one librealsense context this process uses.

    A context is librealsense's handle on the USB system. On macOS the
    process froze when camera objects were destroyed in the wrong order, and
    the sequence that tested reliable keeps the context alive throughout. So
    one is created on first use and never destroyed.
    """
    global _context
    if _context is None:
        _context = rs.context()
    return _context


def default_fps(usb_type):
    """Pick a frame rate the USB connection can carry.

    usb_type is the camera's USB type descriptor, for example '3.2' or
    '2.1'. Anything that is not USB 3 gets the USB 2 rate.
    """
    return USB3_FPS if usb_type.startswith('3') else USB2_FPS


def require_imu_support(platform):
    """Raise if IMU streaming cannot work on this platform.

    platform is sys.platform. On macOS, librealsense crashes the whole
    process when the motion streams start, and Python cannot catch that,
    so it is refused up front.
    """
    if platform == 'darwin':
        raise RuntimeError(
            'IMU streaming is not supported on macOS: librealsense crashes '
            'when the motion streams start. Use the Jetson for IMU.')


def _has_motion_streams(device):
    """Return True if the camera offers accelerometer or gyroscope streams."""
    for sensor in device.query_sensors():
        for profile in sensor.get_stream_profiles():
            if profile.stream_type() in (rs.stream.accel, rs.stream.gyro):
                return True
    return False


def _intrinsics_from_rs(rs_intrinsics):
    return Intrinsics(
        width=rs_intrinsics.width,
        height=rs_intrinsics.height,
        fx=rs_intrinsics.fx,
        fy=rs_intrinsics.fy,
        cx=rs_intrinsics.ppx,  # RealSense calls the principal point ppx, ppy
        cy=rs_intrinsics.ppy,
        distortion_model=rs_intrinsics.model.name,
        distortion_coeffs=tuple(rs_intrinsics.coeffs),
    )


def _imu_sample(frameset):
    """Return the IMU reading that arrived with this frameset, or None.

    This is the latest accel and gyro reading librealsense attached to the
    frameset. They are not interpolated to the image timestamp.
    """
    accel_frame = frameset.first_or_default(rs.stream.accel)
    gyro_frame = frameset.first_or_default(rs.stream.gyro)
    if not accel_frame or not gyro_frame:
        return None
    accel = accel_frame.as_motion_frame().get_motion_data()
    gyro = gyro_frame.as_motion_frame().get_motion_data()
    return ImuSample(
        accel=(accel.x, accel.y, accel.z),
        gyro=(gyro.x, gyro.y, gyro.z),
        timestamp=gyro_frame.get_timestamp() / 1000.0,
    )


class _RealSenseSource(FrameSource):
    """The frame loop shared by the live camera and bag replay."""

    timeout_ms = 5000  # how long to wait for the next frame
    _running = False

    def _start(self, config):
        self._pipeline = rs.pipeline()
        self._profile = self._pipeline.start(config)
        self._running = True
        device = self._profile.get_device()
        # Raw depth is 16-bit integers; depth_scale converts them to meters.
        self._depth_scale = device.first_depth_sensor().get_depth_scale()
        ir_profile = self._profile.get_stream(rs.stream.infrared, LEFT_IR)
        self._intrinsics = _intrinsics_from_rs(
            ir_profile.as_video_stream_profile().get_intrinsics())

    def _finished(self):
        """Return True if running out of frames is the normal end."""
        return False

    def frames(self):
        last_frame_number = None
        while True:
            arrived, frameset = self._pipeline.try_wait_for_frames(self.timeout_ms)
            if not arrived:
                if self._finished():
                    return
                raise RuntimeError(
                    f'no frames arrived for {self.timeout_ms / 1000:g} s')

            depth_frame = frameset.get_depth_frame()
            ir_frame = frameset.get_infrared_frame(LEFT_IR)
            if not depth_frame or not ir_frame:
                continue  # for example a frameset that only carries IMU data
            # librealsense can hand out the same depth frame twice, such as
            # when a new IMU reading arrives between two camera frames.
            if depth_frame.get_frame_number() == last_frame_number:
                continue
            last_frame_number = depth_frame.get_frame_number()

            # get_data() exposes librealsense's own buffer, and librealsense
            # only has a small pool of them. Copy the pixels out so the
            # buffer goes back to the pool; a caller that keeps uncopied
            # frames makes librealsense drop new ones without any error.
            # depth_to_meters copies as part of the conversion.
            image = np.asanyarray(ir_frame.get_data()).copy()
            depth = depth_to_meters(
                np.asanyarray(depth_frame.get_data()), self._depth_scale)
            yield Frame(
                image=image,
                depth=depth,
                intrinsics=self._intrinsics,
                timestamp=depth_frame.get_timestamp() / 1000.0,  # ms to s
                imu=_imu_sample(frameset),
            )

    def close(self):
        if self._running:
            self._pipeline.stop()
            self._running = False
            # Dropping these references destroys the pipeline. That is the
            # moment librealsense finishes a .bag it was recording: without
            # it the file is left unindexed and cannot be replayed.
            self._profile = None
            self._pipeline = None


class LiveSource(_RealSenseSource):
    """Frames from a connected D435i.

    fps=None picks the rate from the USB connection (see default_fps).
    imu=True adds accel and gyro if the camera offers them. It is refused
    on macOS.
    record_to, a file path, also records every enabled stream to that .bag
    until the source is closed. Closing is what finishes the file.
    """

    def __init__(self, width=848, height=480, fps=None, imu=False, record_to=None):
        if imu:
            require_imu_support(sys.platform)

        devices = _shared_context().query_devices()
        if len(devices) == 0:
            raise RuntimeError('no RealSense camera is connected')
        device = devices[0]

        self.usb_type = ''
        if device.supports(rs.camera_info.usb_type_descriptor):
            self.usb_type = device.get_info(rs.camera_info.usb_type_descriptor)
        self.fps = default_fps(self.usb_type) if fps is None else fps
        self.imu_enabled = imu and _has_motion_streams(device)

        config = rs.config()
        config.enable_stream(
            rs.stream.depth, width, height, rs.format.z16, self.fps)
        config.enable_stream(
            rs.stream.infrared, LEFT_IR, width, height, rs.format.y8, self.fps)
        if self.imu_enabled:
            config.enable_stream(rs.stream.accel)
            config.enable_stream(rs.stream.gyro)
        if record_to is not None:
            # librealsense's own recorder: raw frames, including 16-bit
            # depth, go straight to the .bag with their timestamps.
            config.enable_record_to_file(str(record_to))
        self._start(config)


class BagSource(_RealSenseSource):
    """Frames replayed from a .bag recording. Needs no camera."""

    # Replay is not paced by a real clock, so the next frame is ready almost
    # at once. A short wait keeps the pause at the end of the file short.
    timeout_ms = 1000

    def __init__(self, path):
        path = str(path)
        if not os.path.isfile(path):
            raise FileNotFoundError(f'bag file not found: {path}')

        config = rs.config()
        config.enable_device_from_file(path, repeat_playback=False)
        self._start(config)
        self._playback = self._profile.get_device().as_playback()
        # Deliver every recorded frame as fast as we read them, where
        # real-time playback would drop frames if we fall behind.
        self._playback.set_real_time(False)

    def _finished(self):
        return self._playback.current_status() == rs.playback_status.stopped
