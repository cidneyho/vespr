"""Write a tiny RealSense .bag with known contents, without a camera.

librealsense can simulate a camera (a "software device") and record it with
the same recorder a real camera uses. The tests replay this bag, so they
exercise the real playback path on any machine that has pyrealsense2.

Frame i (counting from 0) holds:
  depth: FIRST_DEPTH_RAW + i * DEPTH_STEP_RAW everywhere, except row 0,
         which is 0 (no depth)
  left IR: the value i everywhere
  timestamp: FIRST_TIMESTAMP_MS + i frame periods
"""

import time

import numpy as np
import pyrealsense2 as rs

WIDTH, HEIGHT, FPS = 64, 48, 30
FRAME_COUNT = 40  # more than the 16 frames librealsense keeps in its pool
DEPTH_SCALE = 0.001  # meters per raw depth unit, same as a D435i
FIRST_DEPTH_RAW = 1000  # 1.0 m
DEPTH_STEP_RAW = 100  # each frame is 0.1 m farther
FIRST_TIMESTAMP_MS = 5000.0
FX, FY, CX, CY = 60.0, 61.0, 31.5, 23.5


def _intrinsics():
    intrinsics = rs.intrinsics()
    intrinsics.width, intrinsics.height = WIDTH, HEIGHT
    intrinsics.fx, intrinsics.fy = FX, FY
    intrinsics.ppx, intrinsics.ppy = CX, CY
    intrinsics.model = rs.distortion.brown_conrady
    intrinsics.coeffs = [0.0] * 5
    return intrinsics


def _video_stream(stream_type, index, uid, pixel_format, bytes_per_pixel):
    stream = rs.video_stream()
    stream.type, stream.index, stream.uid = stream_type, index, uid
    stream.width, stream.height, stream.fps = WIDTH, HEIGHT, FPS
    stream.fmt, stream.bpp = pixel_format, bytes_per_pixel
    stream.intrinsics = _intrinsics()
    return stream


def _send(sensor, profile, pixels, bytes_per_pixel, index):
    frame = rs.software_video_frame()
    frame.pixels = pixels
    frame.bpp = bytes_per_pixel
    frame.stride = WIDTH * bytes_per_pixel
    frame.timestamp = FIRST_TIMESTAMP_MS + index * 1000.0 / FPS
    frame.domain = rs.timestamp_domain.hardware_clock
    frame.frame_number = index + 1
    frame.profile = profile.as_video_stream_profile()
    sensor.on_video_frame(frame)


def write_synthetic_bag(path):
    """Record FRAME_COUNT depth + left IR frames to path."""
    device = rs.software_device()
    sensor = device.add_sensor('Stereo Module')
    depth_profile = sensor.add_video_stream(
        _video_stream(rs.stream.depth, 0, 0, rs.format.z16, 2))
    # Infrared index 1 is the left imager, as on the real camera.
    ir_profile = sensor.add_video_stream(
        _video_stream(rs.stream.infrared, 1, 1, rs.format.y8, 1))
    sensor.add_read_only_option(rs.option.depth_units, DEPTH_SCALE)

    recorder = rs.recorder(str(path), device)
    recording_sensor = recorder.query_sensors()[0]
    recording_sensor.open([depth_profile, ir_profile])
    # The recorder only writes frames that are being delivered somewhere,
    # so give them a queue to land in.
    recording_sensor.start(rs.frame_queue(FRAME_COUNT * 2))

    for i in range(FRAME_COUNT):
        depth = np.full((HEIGHT, WIDTH), FIRST_DEPTH_RAW + i * DEPTH_STEP_RAW,
                        dtype=np.uint16)
        depth[0, :] = 0
        ir = np.full((HEIGHT, WIDTH), i, dtype=np.uint8)
        _send(sensor, depth_profile, depth, 2, i)
        _send(sensor, ir_profile, ir, 1, i)
        time.sleep(1.0 / FPS)

    recording_sensor.stop()
    recording_sensor.close()
    # Dropping the recorder is what flushes and closes the file.
    del recorder
