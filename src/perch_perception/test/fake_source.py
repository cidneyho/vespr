"""In-memory frame sources for tests that need frames but no camera."""

import os
import time

import numpy as np

from perch_perception.frame_source import Frame, FrameSource, Intrinsics


def make_frame(timestamp, depth=None, shape=(2, 4)):
    """Build a small frame. depth defaults to all zeros (no valid depth)."""
    if depth is None:
        depth = np.zeros(shape, dtype=np.float32)
    intrinsics = Intrinsics(
        width=4, height=2, fx=60.0, fy=60.0, cx=1.5, cy=0.5,
        distortion_model='none', distortion_coeffs=(0.0,) * 5,
    )
    return Frame(
        image=np.zeros(shape, dtype=np.uint8),
        depth=depth,
        intrinsics=intrinsics,
        timestamp=timestamp,
        imu=None,
    )


class ListSource(FrameSource):
    """A source backed by a list, standing in for a camera."""

    def __init__(self, frames):
        self._frames = frames
        self.closed = False

    def frames(self):
        yield from self._frames

    def close(self):
        self.closed = True


# The functions below open sources inside a child process for the
# ProcessSource tests. They are module-level so the child can import them.

def open_list_source(count):
    """Open a source of count frames with timestamps 0, 1, 2, ..."""
    source = ListSource([make_frame(float(i)) for i in range(count)])
    source.fps = 10
    return source


def open_failing_source():
    raise RuntimeError('no RealSense camera is connected')


def open_slow_source(seconds):
    """Take seconds to open, like a camera that hangs while starting."""
    time.sleep(seconds)
    return open_list_source(1)


class MarkerSource(FrameSource):
    """Yields frames forever and writes a file when closed.

    The file stands in for a recording that is only finished on close.
    close_seconds makes close() slow, to imitate a camera that freezes.
    """

    fps = 10

    def __init__(self, marker_path, close_seconds):
        self._marker_path = marker_path
        self._close_seconds = close_seconds

    def frames(self):
        timestamp = 0.0
        while True:
            time.sleep(0.01)
            timestamp += 1.0
            yield make_frame(timestamp)

    def close(self):
        time.sleep(self._close_seconds)
        with open(self._marker_path, 'w') as marker:
            marker.write('closed')


def open_marker_source(marker_path, close_seconds=0.0):
    return MarkerSource(marker_path, close_seconds)


class DyingSource(FrameSource):
    """Yields two frames, then its whole process dies, like a crash."""

    fps = 10

    def frames(self):
        yield make_frame(0.0)
        yield make_frame(1.0)
        os._exit(3)


def open_dying_source():
    return DyingSource()
