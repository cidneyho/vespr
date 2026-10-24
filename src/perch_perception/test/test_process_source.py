"""Tests for ProcessSource, which runs a frame source in a child process.

These start real child processes. The sources they run come from
fake_source.py, so no camera is involved.
"""

import multiprocessing
import os
import signal
import threading

import numpy as np
import pytest

import fake_source
from perch_perception.process_source import ProcessSource


def test_frames_cross_over_from_the_child_process():
    with ProcessSource(fake_source.open_list_source, (3,)) as source:
        timestamps = [frame.timestamp for frame in source]

    assert timestamps == [0.0, 1.0, 2.0]


def test_frame_images_arrive_intact():
    with ProcessSource(fake_source.open_list_source, (1,)) as source:
        frame = next(iter(source))

    assert frame.image.shape == (2, 4)
    assert frame.image.dtype == np.uint8


def test_the_frame_rate_of_the_inner_source_is_reported():
    with ProcessSource(fake_source.open_list_source, (1,)) as source:
        assert source.fps == 10


def test_a_source_that_fails_to_open_raises_here_with_its_message():
    with pytest.raises(RuntimeError, match='no RealSense camera is connected'):
        ProcessSource(fake_source.open_failing_source)


def test_a_source_that_does_not_open_in_time_has_its_process_ended():
    # A child left running would keep the camera, and every later attempt
    # to open it would fail.
    before = {child.pid for child in multiprocessing.active_children()}

    with pytest.raises(RuntimeError, match='did not open'):
        ProcessSource(fake_source.open_slow_source, (60.0,), open_timeout=1)

    after = {child.pid for child in multiprocessing.active_children()}
    assert after - before == set()


def test_close_lets_the_inner_source_close_before_the_process_ends(tmp_path):
    marker = tmp_path / 'closed.txt'
    source = ProcessSource(fake_source.open_marker_source, (str(marker),))
    next(iter(source))

    source.close()

    assert marker.exists()


def test_closing_twice_is_harmless(tmp_path):
    source = ProcessSource(fake_source.open_marker_source, (str(tmp_path / 'm.txt'),))

    source.close()
    source.close()


def test_ctrl_c_does_not_stop_the_child_before_it_has_closed(tmp_path):
    # Ctrl+C in a terminal goes to the child process as well. If it died
    # there and then, a recording would be left unfinished.
    marker = tmp_path / 'closed.txt'
    source = ProcessSource(fake_source.open_marker_source, (str(marker),))
    frames = iter(source)
    next(frames)

    os.kill(source.pid, signal.SIGINT)
    next(frames)
    source.close()

    assert marker.exists()


def test_a_child_that_freezes_while_closing_is_ended_and_reported(tmp_path):
    source = ProcessSource(
        fake_source.open_marker_source, (str(tmp_path / 'm.txt'), 60.0), close_timeout=1)
    next(iter(source))

    with pytest.raises(RuntimeError, match='froze'):
        source.close()

    with pytest.raises(ProcessLookupError):
        os.kill(source.pid, 0)  # signal 0 only asks whether the process exists


def test_a_child_that_dies_is_reported():
    source = ProcessSource(fake_source.open_dying_source)

    with pytest.raises(RuntimeError, match='ended unexpectedly'):
        list(source)


def test_ctrl_c_while_the_source_opens_ends_the_child():
    # The recorder checks whether a .bag exists once opening fails, so the child that would
    # create it must be gone by then, however opening ended.
    # A real SIGINT, as Ctrl+C sends, once the child is running. It wakes the wait for the child.
    timer = threading.Timer(2.0, os.kill, (os.getpid(), signal.SIGINT))
    timer.start()
    try:
        with pytest.raises(KeyboardInterrupt):
            ProcessSource(fake_source.open_slow_source, (30,))
    finally:
        timer.cancel()

    assert multiprocessing.active_children() == []
