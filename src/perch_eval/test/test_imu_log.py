"""Tests for imu_log.py: the CSV rows, drop counting, the writer thread,
file names, clock statistics and a whole run against a fake camera.

No camera is used. FakeFrame stands in for a librealsense motion frame and
FakeCamera for the opened device.
"""

import csv
import io
import json
import os
import threading
import time
from datetime import datetime

import pytest

from perch_eval import imu_log
from perch_eval.imu_log import Sample

ACCEL = object()  # stands in for rs.stream.accel
GYRO = object()
FRAME_TIMESTAMP = object()  # stands in for rs.frame_metadata_value.frame_timestamp


def make_sample(stream='gyro', frame_number=1, hw=1000, host_ns=None, x=0.1, y=-0.2, z=9.8):
    return Sample(
        stream, frame_number, hw, 1792529380381.291, 'global_time',
        time.time_ns() if host_ns is None else host_ns, x, y, z)


def read_rows(path):
    with open(path, newline='') as file:
        return list(csv.reader(file))


# CSV rows

def test_csv_row_has_the_fields_in_order():
    sample = Sample('gyro', 42, 123456789, 1792529380381.291, 'global_time',
                    1792529380381291000, -0.006981316953897476, 0.0, 0.01745329238474369)

    line = io.StringIO()
    csv.writer(line).writerow(imu_log.csv_row(sample))

    assert line.getvalue() == (
        'gyro,42,123456789,1792529380381.291,global_time,1792529380381291000,'
        '-0.006981316953897476,0.0,0.01745329238474369\r\n')


def test_csv_row_leaves_a_missing_hardware_timestamp_blank():
    line = io.StringIO()
    csv.writer(line).writerow(imu_log.csv_row(make_sample(hw=None, host_ns=5)))

    assert line.getvalue().split(',')[2] == ''


def test_csv_header_matches_the_row():
    assert imu_log.CSV_FIELDS == (
        'stream', 'frame_number', 'hw_timestamp_us', 'ts_ms', 'ts_domain',
        'host_ns', 'x', 'y', 'z')
    assert len(imu_log.csv_row(make_sample())) == len(imu_log.CSV_FIELDS)


# Reading a frame

class FakeVector:
    def __init__(self, x, y, z):
        self.x, self.y, self.z = x, y, z


class FakeProfile:
    def __init__(self, stream):
        self._stream = stream

    def stream_type(self):
        return self._stream


class FakeDomain:
    name = 'global_time'


class FakeFrame:
    def __init__(self, stream, number, hw, ts, xyz):
        self._stream, self._number, self._hw, self._ts, self._xyz = stream, number, hw, ts, xyz

    def get_profile(self):
        return FakeProfile(self._stream)

    def supports_frame_metadata(self, field):
        return field is FRAME_TIMESTAMP and self._hw is not None

    def get_frame_metadata(self, field):
        assert field is FRAME_TIMESTAMP
        return self._hw

    def as_motion_frame(self):
        return self

    def get_motion_data(self):
        return FakeVector(*self._xyz)

    def get_frame_number(self):
        return self._number

    def get_timestamp(self):
        return self._ts

    def get_frame_timestamp_domain(self):
        return FakeDomain()


def test_motion_sample_copies_the_frame_fields():
    frame = FakeFrame(ACCEL, 7, 5000, 12.5, (0.1, -9.8, 0.2))

    sample = imu_log.motion_sample(frame, 99, ACCEL, FRAME_TIMESTAMP)

    assert sample == Sample('accel', 7, 5000, 12.5, 'global_time', 99, 0.1, -9.8, 0.2)


def test_motion_sample_names_the_gyro_and_handles_missing_metadata():
    frame = FakeFrame(GYRO, 8, None, 13.0, (0.0, 0.0, 0.0))

    sample = imu_log.motion_sample(frame, 100, ACCEL, FRAME_TIMESTAMP)

    assert sample.stream == 'gyro'
    assert sample.hw_timestamp_us is None


def test_enum_text_uses_the_name_or_the_last_part_of_the_text():
    assert imu_log.enum_text(FakeDomain()) == 'global_time'
    assert imu_log.enum_text('timestamp_domain.hardware_clock') == 'hardware_clock'


# Drop counting

def count(numbers):
    counter = imu_log.DropCounter()
    for number in numbers:
        counter.add(number)
    return counter


def test_consecutive_frame_numbers_are_no_drops():
    assert count([5, 6, 7, 8]).dropped == 0


def test_a_gap_counts_every_missing_frame():
    assert count([1, 2, 5, 6, 10]).dropped == 2 + 3


def test_repeats_and_restarts_are_not_drops():
    counter = count([1, 2, 2, 3, 1, 2])

    assert counter.dropped == 0
    assert counter.backwards == 2


def test_the_first_frame_number_is_not_a_gap():
    assert count([1000]).dropped == 0


# The writer thread

def test_writer_writes_every_sample_put_before_stop():
    file = io.StringIO()
    writer = imu_log.Writer(file)
    writer.begin(0)
    samples = [make_sample(stream=('accel', 'gyro')[i % 2], frame_number=i // 2, hw=i * 2500)
               for i in range(5000)]

    for sample in samples:
        writer.put(sample)
    writer.stop()

    rows = list(csv.reader(io.StringIO(file.getvalue())))
    assert rows[0] == list(imu_log.CSV_FIELDS)
    assert len(rows) == 1 + len(samples)
    assert rows[-1][:3] == ['gyro', str(samples[-1].frame_number), str(samples[-1].hw_timestamp_us)]
    assert writer.written == len(samples)
    assert writer.stats['accel'].count == writer.stats['gyro'].count == 2500
    assert writer.error is None


def test_writer_discards_samples_from_before_begin():
    file = io.StringIO()
    writer = imu_log.Writer(file)
    writer.put(make_sample(host_ns=10))  # warm-up, before begin is even called
    writer.put(make_sample(host_ns=99))
    writer.begin(100)
    writer.put(make_sample(host_ns=100, frame_number=2))
    writer.stop()

    rows = list(csv.reader(io.StringIO(file.getvalue())))
    assert len(rows) == 2
    assert rows[1][5] == '100'
    assert writer.warmup_samples == 2


def test_writer_flushes_to_disk_while_running(tmp_path):
    path = tmp_path / 'log.csv'
    with open(path, 'w', newline='') as file:
        writer = imu_log.Writer(file)
        writer.begin(0)
        writer.put(make_sample())
        deadline = time.monotonic() + 5
        while len(read_rows(path)) < 2 and time.monotonic() < deadline:
            time.sleep(0.05)
        rows_while_running = len(read_rows(path))
        writer.stop()

    assert rows_while_running == 2


def test_writer_records_an_error_instead_of_dying_silently():
    class FullDisk(io.StringIO):
        def write(self, text):
            if 'gyro' in text:
                raise OSError('No space left on device')
            return super().write(text)

    writer = imu_log.Writer(FullDisk())
    writer.begin(0)
    writer.put(make_sample())
    writer.stop()

    assert isinstance(writer.error, OSError)


# File names

def test_log_name_is_the_start_time():
    assert imu_log.log_name(datetime(2026, 10, 20, 22, 15, 0)) == 'imu_2026-10-20T22-15-00'


def test_default_folder_is_in_the_home_folder(monkeypatch):
    monkeypatch.delenv('SUDO_USER', raising=False)

    assert imu_log.default_folder() == os.path.join(
        os.path.expanduser('~'), 'vespr_recordings', 'imu')


# Clock statistics

def test_unwrapper_carries_over_a_32_bit_wrap():
    unwrap = imu_log.Unwrapper(32)
    span = 1 << 32

    values = [unwrap(v) for v in (span - 2000, span - 1000, 0, 1000)]

    assert values == [span - 2000, span - 1000, span, span + 1000]


def test_unwrapper_ignores_a_small_step_back():
    unwrap = imu_log.Unwrapper(32)

    assert [unwrap(v) for v in (5000, 4000, 6000)] == [5000, 4000, 6000]


def test_linear_fit_finds_an_exact_line():
    fit = imu_log.LinearFit()
    for i in range(1000):
        fit.add(i * 0.005, 3.0 + 20.0 * i * 0.005)

    assert fit.slope() == pytest.approx(20.0)
    assert fit.residual_rms() == pytest.approx(0.0, abs=1e-9)


def test_linear_fit_matches_numpy():
    np = pytest.importorskip('numpy')
    rng = np.random.default_rng(1)
    x = np.arange(20000) * 0.005
    y = 1e6 + 50.0 * x + rng.normal(0, 400.0, x.size)
    fit = imu_log.LinearFit()
    for xi, yi in zip(x, y):
        fit.add(float(xi), float(yi))

    slope, intercept = np.polyfit(x, y, 1)
    residual = np.sqrt(np.mean((y - (intercept + slope * x)) ** 2))
    assert fit.slope() == pytest.approx(slope, rel=1e-9)
    assert fit.residual_rms() == pytest.approx(residual, rel=1e-6)


def test_stream_stats_measure_rate_and_drift():
    stats = imu_log.StreamStats()
    host_start = 1_792_529_380_000_000_000
    for i in range(2001):  # 10 s at 200 Hz; the host clock runs 25 ppm fast
        hw = i * 5000
        stats.add(make_sample(frame_number=i, hw=hw, host_ns=host_start + round(hw * 1000 * 1.000025)))

    assert stats.mean_rate() == pytest.approx(200.0)
    assert stats.drift_ppm() == pytest.approx(25.0, rel=1e-3)
    assert stats.drift_residual_ms() < 1e-3
    assert stats.drops.dropped == 0


def test_window_rate_covers_only_the_samples_since_the_last_call():
    stats = imu_log.StreamStats()
    for i in range(201):  # 1 s at 200 Hz
        stats.add(make_sample(frame_number=i, hw=i * 5000))
    stats.window_rate()
    for i in range(201, 301):  # then 1 s at 100 Hz
        stats.add(make_sample(frame_number=i, hw=1_000_000 + (i - 200) * 10000))

    assert stats.window_rate() == pytest.approx(100.0)


def test_status_line_shows_every_field():
    streams = {
        'accel': {'samples': 12000, 'dropped': 0, 'rate': 200.004},
        'gyro': {'samples': 11990, 'dropped': 3, 'rate': None},
    }

    line = imu_log.status_line(3725, streams, 4, {'asic': 41.0, 'projector': None})

    assert line == (
        '01:02:05 | accel 12000 samples, 200.00 Hz, 0 dropped | '
        'gyro 11990 samples, n/a, 3 dropped | queue 4 | asic 41.0 C, projector n/a')


# A whole run against a fake camera

class FakeCamera:
    """Calls on_sample from its own thread, like librealsense's callback."""

    def __init__(self, interval=0.001):
        self.interval = interval
        self.sent = []
        self.stopped = False
        self._halt = threading.Event()
        self._thread = None

    def info(self):
        return {'name': 'fake D435i', 'serial': '348522076012', 'motion_streams': {}}

    def temperatures(self):
        return {'asic': 40.0, 'projector': 35.0}

    def start(self, on_sample):
        def send():
            number = 0
            while not self._halt.is_set():
                for stream in ('accel', 'gyro'):
                    sample = make_sample(stream=stream, frame_number=number, hw=number * 5000)
                    self.sent.append(sample)
                    on_sample(sample)
                number += 1
                time.sleep(self.interval)

        self._thread = threading.Thread(target=send)
        self._thread.start()

    def stop(self):
        self._halt.set()
        if self._thread is not None:
            self._thread.join()
        self.stopped = True


def test_run_log_writes_the_tail_and_the_sidecar(tmp_path):
    camera = FakeCamera()
    clock = iter([datetime(2026, 10, 20, 22, 15, 0)] + [datetime(2026, 10, 20, 22, 16, 0)] * 5)

    path = imu_log.run_log(
        camera, str(tmp_path), warmup=0.2, duration=0.5, rate=200,
        options={'rate': 200}, stop=threading.Event(), clock=lambda: next(clock),
        status_interval=0.15)

    assert path == str(tmp_path / 'imu_2026-10-20T22-15-00.csv')
    assert camera.stopped
    rows = read_rows(path)[1:]
    written = [(row[0], int(row[1])) for row in rows]
    sent = [(s.stream, s.frame_number) for s in camera.sent]
    # What was written is the end of what was sent, up to the last sample:
    # the warm-up is cut off and the tail is kept.
    assert written == sent[len(sent) - len(written):]
    assert 0 < len(written) < len(sent)

    sidecar = json.loads((tmp_path / 'imu_2026-10-20T22-15-00.json').read_text())
    assert sidecar['camera']['serial'] == '348522076012'
    assert sidecar['options'] == {'rate': 200}
    assert sidecar['started'] == '2026-10-20T22:15:00'
    assert sidecar['ended'] == '2026-10-20T22:16:00'
    assert sidecar['report']['rows_written'] == len(written)
    assert sidecar['report']['warmup_samples_discarded'] > 0

    status = (tmp_path / 'imu_2026-10-20T22-15-00.status.log').read_text()
    assert 'samples discarded' in status  # a warm-up status line
    assert 'final report' in status
    assert 'queue' in status


def test_run_log_stops_cleanly_on_the_stop_event(tmp_path):
    camera = FakeCamera()
    stop = threading.Event()
    threading.Timer(0.3, stop.set).start()

    path = imu_log.run_log(
        camera, str(tmp_path), warmup=0, duration=0, rate=200, options={},
        stop=stop, status_interval=60)

    rows = read_rows(path)[1:]
    assert camera.stopped
    assert len(rows) > 0
    last = camera.sent[-1]
    assert rows[-1][:2] == [last.stream, str(last.frame_number)]


def test_dry_run_is_one_minute_without_warm_up():
    args = imu_log.parse_args(['--dry-run'])

    assert (args.warmup, args.duration) == (0, 60)


def test_defaults():
    args = imu_log.parse_args([])

    assert args.rate == 200
    assert args.warmup == 1200
    assert args.duration == 0
    assert args.depth_fps == 30
    assert not args.depth
    assert args.librealsense.endswith(os.path.join('librealsense', 'build', 'Release'))
