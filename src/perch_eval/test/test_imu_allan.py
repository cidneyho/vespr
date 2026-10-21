"""Tests for imu_allan.py: its time base on a CSV in imu_log's format, and
its own --self-test.

Skipped without numpy, pandas and matplotlib, which the analysis needs.
"""

import csv
import json
import os
import re
import subprocess
import sys

import pytest

np = pytest.importorskip('numpy')
pytest.importorskip('pandas')
pytest.importorskip('matplotlib')

from perch_eval import imu_allan, imu_log  # noqa: E402  (after the skips)
from perch_eval.imu_log import Sample  # noqa: E402

PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

PERIOD_US = 4943  # the accel period overnight
SAMPLES = 3000
WRAP_AT = 1000  # the counter wraps before this sample
LOST = (500, 2000)  # lost inside the camera: no gap in frame_number


def write_recording(path):
    """Both streams, interleaved, as imu_log writes them: the 32-bit ts_ms
    counter wraps once and one sample is missing at each of LOST."""
    rng = np.random.default_rng(4)
    start_us = (1 << 32) - WRAP_AT * PERIOD_US
    rows = []
    for stream in imu_log.STREAMS:
        frame_number = 0
        for i in range(SAMPLES):
            if i in LOST:
                continue
            ts_us = (start_us + i * PERIOD_US) % (1 << 32)
            x, y, z = rng.normal(0.0, 0.01, 3)
            rows.append(Sample(stream, frame_number, None, ts_us / 1000, 'hardware_clock',
                               1_792_529_380_000_000_000 + i * PERIOD_US * 1000,
                               float(x), float(y), float(z) + 9.8))
            frame_number += 1
    rows.sort(key=lambda sample: sample.host_ns)
    with open(path, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(imu_log.CSV_FIELDS)
        for sample in rows:
            writer.writerow(imu_log.csv_row(sample))


def test_the_clock_line_reports_one_wrap_two_gaps_and_one_segment(tmp_path, capsys):
    recording = tmp_path / 'imu_test.csv'
    write_recording(recording)

    assert imu_allan.main([str(recording), '--out', str(tmp_path)]) == 0

    clock_lines = [line for line in capsys.readouterr().out.splitlines()
                   if line.strip().startswith('clock')]
    assert len(clock_lines) == 2  # one per stream
    for line in clock_lines:
        assert re.search(r'\b1 wraps unwrapped\b', line), line
        assert re.search(r'\b2 gaps \(2 samples\) bridged\b', line), line
        assert re.search(r'\bsegment 1 of 1 used\b', line), line

    result = json.loads((tmp_path / 'imu_test_allan.json').read_text())
    for name in imu_log.STREAMS:
        grid = result['sensors'][name]['grid']
        assert (grid['n_wraps'], grid['n_gaps_bridged'], grid['n_segments']) == (1, 2, 1)
        assert grid['n_grid'] == SAMPLES
        assert grid['period_us'] == PERIOD_US
    assert (tmp_path / 'imu_test_allan.png').exists()


def test_self_test_passes():
    result = subprocess.run(
        [sys.executable, '-m', 'perch_eval.imu_allan', '--self-test'],
        cwd=PACKAGE_ROOT, capture_output=True, text=True, timeout=300)

    assert result.returncode == 0, result.stdout + result.stderr
    assert 'self-test PASSED' in result.stdout
    assert 'FAIL' not in result.stdout.replace('FAILED', '')
