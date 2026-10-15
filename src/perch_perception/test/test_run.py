"""Tests for run.py, the command that reads a frame source end to end."""

import numpy as np
import pytest

from fake_source import ListSource, make_frame
from perch_perception import run


def test_depth_summary_ignores_zero_depth():
    depth = np.array([[0.0, 1.0, 3.0, 0.0]], dtype=np.float32)

    median_m, valid_fraction = run.depth_summary(depth)

    assert median_m == 2.0
    assert valid_fraction == 0.5


def test_depth_summary_has_no_median_when_all_depth_is_zero():
    depth = np.zeros((2, 4), dtype=np.float32)

    median_m, valid_fraction = run.depth_summary(depth)

    assert median_m is None
    assert valid_fraction == 0.0


def test_describe_reports_the_median_depth_and_valid_fraction():
    depth = np.array([[0.0, 1.5, 1.5, 1.5]], dtype=np.float32)

    line = run.describe(make_frame(timestamp=2.0, depth=depth))

    assert 'median depth 1.500 m' in line
    assert 'valid 75%' in line


def test_describe_says_so_when_there_is_no_valid_depth():
    line = run.describe(make_frame(timestamp=2.0))

    assert 'no valid depth' in line


def test_run_reads_every_frame():
    source = ListSource([make_frame(1.0), make_frame(2.0), make_frame(3.0)])

    assert run.run(source) == 3


def test_run_stops_at_max_frames():
    source = ListSource([make_frame(1.0), make_frame(2.0), make_frame(3.0)])

    assert run.run(source, max_frames=2) == 2


def test_run_prints_one_line_every_print_every_frames(capsys):
    source = ListSource([make_frame(float(i)) for i in range(5)])

    run.run(source, print_every=2)

    # Frames 0, 2 and 4.
    assert len(capsys.readouterr().out.splitlines()) == 3


def test_without_a_bag_flag_the_source_is_the_live_camera():
    assert run.parse_args([]).bag is None


def test_bag_flag_replays_a_recording(tmp_path, capsys):
    pytest.importorskip('pyrealsense2')
    import synthetic_bag

    path = tmp_path / 'synthetic.bag'
    synthetic_bag.write_synthetic_bag(path)

    exit_code = run.main(['--bag', str(path), '--max-frames', '5'])

    assert exit_code == 0
    assert 'read 5 frames' in capsys.readouterr().out
