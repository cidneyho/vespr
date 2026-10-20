"""Log the D435i's raw IMU samples to CSV for Allan variance characterization.

    python3 -m perch_eval.imu_log --dry-run                  # one minute, no warm-up
    python3 -m perch_eval.imu_log --depth --duration 10800   # the characterization run

Every accelerometer and gyroscope sample the camera sends becomes one CSV
row, both streams interleaved in arrival order. Nothing is merged, paired or
interpolated; that is left to the analysis.

The camera is opened here rather than through perch_perception's
FrameSource, because LiveSource keeps one IMU sample per camera frame and
discards the rest. This module opens the motion sensor directly and starts
it with a callback, so librealsense hands over every sample. The callback
only timestamps the arrival and puts the fields on a queue; a writer thread
drains the queue to disk. A slow callback is how librealsense drops frames.

pyrealsense2 is only imported inside open_camera, so this module and its
tests import on a machine without the camera SDK.

Files, named by the time the program started, in --out:

    imu_<start>.csv          one row per motion frame (see CSV_FIELDS)
    imu_<start>.json         camera, calibration, options, host, git commit, times, report
    imu_<start>.status.log   the warm-up and status lines and the final report
"""

import argparse
import csv
import json
import math
import os
import queue
import signal
import socket
import subprocess
import sys
import threading
import time
from collections import namedtuple
from datetime import datetime

STREAMS = ('accel', 'gyro')
CSV_FIELDS = (
    'stream', 'frame_number', 'hw_timestamp_us', 'ts_ms', 'ts_domain',
    'host_ns', 'x', 'y', 'z')
Sample = namedtuple('Sample', CSV_FIELDS)

STATUS_INTERVAL = 60  # seconds between status lines
FLUSH_INTERVAL = 1.0  # seconds between flushes of the CSV to disk

# ts_ms is wanted in the hardware_clock domain: the camera's own clock, in
# milliseconds with microsecond resolution. librealsense's default domain,
# global_time, maps it onto the host clock, and on the Jetson the first
# samples came in hardware_clock before it switched over. So
# global_time_enabled is set to 0 on the sensors before they start.
HARDWARE_CLOCK = 'hardware_clock'

# A ts_ms interval outside these multiples of the median period is a clock
# discontinuity rather than jitter. Dropped frames are counted separately,
# from gaps in frame_number.
DISCONTINUITY_BAND = (0.5, 1.5)

# The band is set from the period at --rate until a stream has this many
# intervals, and from their median after that.
MIN_INTERVALS = 10

# A discontinuity within this many seconds of a stream's first logged sample
# is the startup clock jump (see the README): noted, not warned about.
STARTUP_SECONDS = 1.0

TEMPERATURE_SUFFIX = '_temperature'  # options read for the status lines

DEPTH_WIDTH = 848
DEPTH_HEIGHT = 480
LEFT_IR = 1  # RealSense numbers the infrared imagers 1 (left) and 2 (right)

RATE_TOLERANCE = 0.02  # a measured rate further than 2 % from --rate is flagged

_STOP = object()  # tells the writer thread that no more samples will come


def _home():
    """Return the home folder of whoever runs this, also under sudo.

    Under sudo, '~' can mean root's home. sudo keeps the original user's
    name in SUDO_USER, so the home directory is looked up from that.
    """
    return os.path.expanduser('~' + os.environ.get('SUDO_USER', ''))


def default_folder():
    """Return ~/vespr_recordings/imu.

    The home folder, not the Desktop or Documents, for the same reason as
    perch_perception's clips: macOS can sync those two to iCloud Drive, and
    a multi-hour log is hundreds of megabytes.
    """
    return os.path.join(_home(), 'vespr_recordings', 'imu')


def default_librealsense():
    return os.path.join(_home(), 'librealsense', 'build', 'Release')


def hand_to_user(path):
    """Under sudo, give a file back to the person who ran sudo."""
    if 'SUDO_UID' in os.environ:
        os.chown(path, int(os.environ['SUDO_UID']), int(os.environ['SUDO_GID']))


def log_name(now):
    """Name a log by its start time, for example imu_2026-10-20T22-15-00."""
    return 'imu_' + now.strftime('%Y-%m-%dT%H-%M-%S')


def csv_row(sample):
    """Return the CSV fields of one sample. A missing hw_timestamp_us is blank."""
    hw = '' if sample.hw_timestamp_us is None else sample.hw_timestamp_us
    return [
        sample.stream, sample.frame_number, hw, sample.ts_ms, sample.ts_domain,
        sample.host_ns, sample.x, sample.y, sample.z]


def enum_text(value):
    """Return a librealsense enum as text, for example 'global_time'."""
    name = getattr(value, 'name', None)
    return name if isinstance(name, str) else str(value).rsplit('.', 1)[-1]


def motion_sample(frame, host_ns, accel_stream, frame_timestamp):
    """Copy the fields of one librealsense motion frame into a Sample.

    accel_stream is rs.stream.accel and frame_timestamp is
    rs.frame_metadata_value.frame_timestamp. They are passed in so this
    runs on fake frames without the SDK. Only plain numbers and text come
    out: keeping the frame itself would hold one of librealsense's buffers.
    """
    if frame.get_profile().stream_type() == accel_stream:
        stream = 'accel'
    else:
        stream = 'gyro'
    hw = None
    if frame.supports_frame_metadata(frame_timestamp):
        hw = frame.get_frame_metadata(frame_timestamp)
    data = frame.as_motion_frame().get_motion_data()
    return Sample(
        stream, frame.get_frame_number(), hw, frame.get_timestamp(),
        enum_text(frame.get_frame_timestamp_domain()), host_ns,
        data.x, data.y, data.z)


class DropCounter:
    """Counts frames missing from a stream, from gaps in its frame numbers.

    A frame number that does not move forward (a repeat, or the counter
    restarting) is counted in `backwards`, not as a drop.
    """

    def __init__(self):
        self.last = None
        self.dropped = 0
        self.backwards = 0

    def add(self, frame_number):
        if self.last is not None:
            if frame_number > self.last + 1:
                self.dropped += frame_number - self.last - 1
            elif frame_number <= self.last:
                self.backwards += 1
        self.last = frame_number


class Histogram:
    """Counts of integer values, with their exact median.

    The ts_ms intervals take only a handful of distinct values (whole
    microseconds), so counting them keeps the median of hours of samples
    in a few entries.
    """

    def __init__(self):
        self.counts = {}
        self.n = 0

    def add(self, value):
        self.counts[value] = self.counts.get(value, 0) + 1
        self.n += 1

    def median(self):
        if self.n == 0:
            return None
        low_index, high_index = (self.n - 1) // 2, self.n // 2
        seen = 0
        low = None
        for value in sorted(self.counts):
            seen += self.counts[value]
            if low is None and seen > low_index:
                low = value
            if seen > high_index:
                return (low + value) / 2


class LinearFit:
    """Least-squares line y = a + b x, updated one point at a time.

    Uses running means and co-moments (Welford), which stay accurate over
    millions of points where plain sums of squares lose precision.
    """

    def __init__(self):
        self.n = 0
        self._mean_x = 0.0
        self._mean_y = 0.0
        self._sxx = 0.0
        self._sxy = 0.0
        self._syy = 0.0

    def add(self, x, y):
        self.n += 1
        dx = x - self._mean_x
        dy = y - self._mean_y
        self._mean_x += dx / self.n
        self._mean_y += dy / self.n
        self._sxx += dx * (x - self._mean_x)
        self._sxy += dx * (y - self._mean_y)
        self._syy += dy * (y - self._mean_y)

    def slope(self):
        if self.n < 2 or self._sxx == 0:
            return None
        return self._sxy / self._sxx

    def residual_rms(self):
        """Root mean square of the distances from the points to the line."""
        if self.n < 3 or self._sxx == 0:
            return None
        residual = self._syy - self._sxy * self._sxy / self._sxx
        return math.sqrt(max(residual, 0.0) / self.n)


class Segment:
    """A stretch of one stream with no clock discontinuity and one ts_domain.

    Holds the drift fit of host_ns against ts_ms over the stretch: host_ns -
    ts_ms against ts_ms, both in seconds and relative to the first sample.
    The slope of that is the slope of host_ns against ts_ms minus 1, so the
    fit works on the small part that matters.
    """

    def __init__(self, index, sample, line, ts_us):
        self.index = index
        self.domain = sample.ts_domain
        self.first_line = self.last_line = line
        self.first_frame = self.last_frame = sample.frame_number
        self.first_ts = ts_us
        self.last_ts = ts_us
        self.samples = 0
        self._origin_host = sample.host_ns
        self.drift = LinearFit()
        self.add(sample, line, ts_us)

    def add(self, sample, line, ts_us):
        self.samples += 1
        self.last_line = line
        self.last_frame = sample.frame_number
        self.last_ts = ts_us
        x = (ts_us - self.first_ts) / 1e6
        host = (sample.host_ns - self._origin_host) / 1e9
        self.drift.add(x, host - x)

    def summary(self, count):
        """Plain values for the sidecar; count is how many segments the stream has."""
        slope = self.drift.slope()  # seconds per second
        residual = self.drift.residual_rms()  # seconds
        return {
            'index': self.index,
            'of': count,
            'ts_domain': self.domain,
            'first_line': self.first_line,
            'last_line': self.last_line,
            'first_frame_number': self.first_frame,
            'last_frame_number': self.last_frame,
            'samples': self.samples,
            'duration_s': (self.last_ts - self.first_ts) / 1e6,
            'drift_ppm': None if slope is None else slope * 1e6,
            'drift_residual_us': None if residual is None else residual * 1e6,
        }


class StreamStats:
    """Running counts, rate, drops, clock discontinuities and drift for one stream.

    Dropped frames are gaps in frame_number, the authoritative count. The
    rate is 1 / the median interval between consecutive ts_ms. An interval
    outside DISCONTINUITY_BAND times the median so far is a clock
    discontinuity, recorded in discontinuities; a ts_domain change is
    recorded in domain_changes. Either one ends a segment, and the drift fit
    is reported for the longest segment.
    """

    def __init__(self, rate):
        self.nominal_period_us = 1e6 / rate
        self.count = 0
        self.frame_gaps = DropCounter()
        self.missing_hw = 0
        self.domain = None
        self.domain_changes = []  # dicts: line, frame_number, from, to
        self.discontinuities = []  # dicts, see _discontinuity
        self.periods = Histogram()  # ts_ms intervals in microseconds, within a domain
        self._window = Histogram()  # the same, since the last window_rate()
        self._reference = self.nominal_period_us
        self._first_host_ns = None
        self._last_ts = None
        self.segments = 0
        self.segment = None
        self._longest = None

    def add(self, sample, line=None):
        """Add one sample; line is its line number in the CSV, for warnings."""
        self.count += 1
        previous_frame = self.frame_gaps.last
        self.frame_gaps.add(sample.frame_number)
        if sample.hw_timestamp_us is None:
            self.missing_hw += 1
        if self._first_host_ns is None:
            self._first_host_ns = sample.host_ns

        ts = round(sample.ts_ms * 1000)  # microseconds, the camera clock's resolution
        new_segment = self.segment is None
        if self.domain is not None and sample.ts_domain != self.domain:
            # Two clocks: the step between them is not an interval.
            self.domain_changes.append({
                'line': line, 'frame_number': sample.frame_number,
                'from': self.domain, 'to': sample.ts_domain})
            new_segment = True
        elif self._last_ts is not None:
            interval = ts - self._last_ts
            low, high = DISCONTINUITY_BAND
            if not low * self._reference <= interval <= high * self._reference:
                self.discontinuities.append(
                    self._discontinuity(sample, line, interval, previous_frame))
                new_segment = True
            self.periods.add(interval)
            self._window.add(interval)
            if self.periods.n >= MIN_INTERVALS and (
                    self.periods.n < 1000 or self.periods.n % 1000 == 0):
                self._reference = self.periods.median()
        self.domain = sample.ts_domain
        self._last_ts = ts

        if new_segment:
            if self.segment is not None and (
                    self._longest is None or self.segment.samples > self._longest.samples):
                self._longest = self.segment
            self.segments += 1
            self.segment = Segment(self.segments, sample, line, ts)
        else:
            self.segment.add(sample, line, ts)

    def _discontinuity(self, sample, line, interval_us, previous_frame):
        seconds_in = (sample.host_ns - self._first_host_ns) / 1e9
        missing = 0
        if previous_frame is not None:
            missing = max(0, sample.frame_number - previous_frame - 1)
        return {
            'line': line,
            'frame_number': sample.frame_number,
            'step_ms': interval_us / 1000,
            'frames_missing': missing,
            'seconds_into_log': seconds_in,
            'startup': seconds_in < STARTUP_SECONDS,
        }

    def longest_segment(self):
        """The segment with the most samples, the earlier one on a tie."""
        if self._longest is None or (
                self.segment is not None and self.segment.samples > self._longest.samples):
            return self.segment
        return self._longest

    def period_us(self):
        """Median interval between consecutive ts_ms, None without one."""
        return self.periods.median()

    def rate(self):
        """Rate in Hz, 1 / the median period; None without two samples."""
        return _rate(self.periods.median())

    def window_rate(self):
        """Rate in Hz from the intervals since the last call."""
        rate = _rate(self._window.median())
        self._window = Histogram()
        return rate


def _rate(period_us):
    if period_us is None or period_us <= 0:
        return None
    return 1e6 / period_us


class Writer:
    """Drains the sample queue into the CSV on its own thread.

    put() is the only thing the camera callback calls. Samples that arrived
    before begin() are the warm-up: they are counted and not written.
    stop() writes everything still queued, then flushes the file.
    rate is the requested rate in Hz, the period clock discontinuities are
    judged against until a stream has a median period of its own.
    """

    def __init__(self, file, rate):
        self._rate = rate
        self._file = file
        self._csv = csv.writer(file)
        self._csv.writerow(CSV_FIELDS)
        self._queue = queue.SimpleQueue()
        self.put = self._queue.put
        self._lock = threading.Lock()
        self.stats = {name: StreamStats(rate) for name in STREAMS}
        self.warmup_samples = 0
        self.written = 0
        self.error = None
        self._begin_ns = None
        self._thread = threading.Thread(target=self._run, name='imu_writer', daemon=True)
        self._thread.start()

    def begin(self, host_ns):
        """Write samples that arrived at or after host_ns (time.time_ns())."""
        self._begin_ns = host_ns

    def queue_depth(self):
        return self._queue.qsize()

    def snapshot(self):
        """Counts, drops, discontinuities and rate since the last snapshot, per stream."""
        with self._lock:
            return {
                name: {
                    'samples': stats.count,
                    'dropped': stats.frame_gaps.dropped,
                    'rate': stats.window_rate(),
                    'discontinuities': list(stats.discontinuities),
                }
                for name, stats in self.stats.items()
            }

    def domain_changes(self):
        """Every ts_domain change so far, in the order they were written."""
        return self._events('domain_changes')

    def discontinuities(self):
        """Every clock discontinuity so far, in the order they were written."""
        return self._events('discontinuities')

    def _events(self, kind):
        with self._lock:
            events = [dict(event, stream=name)
                      for name, stats in self.stats.items()
                      for event in getattr(stats, kind)]
        return sorted(events, key=lambda event: event['line'])

    def stop(self):
        self._queue.put(_STOP)
        self._thread.join()

    def _run(self):
        last_flush = time.monotonic()
        try:
            while True:
                try:
                    item = self._queue.get(timeout=FLUSH_INTERVAL)
                except queue.Empty:
                    item = None
                if item is _STOP:
                    break
                if item is not None:
                    self._write(item)
                if time.monotonic() - last_flush >= FLUSH_INTERVAL:
                    self._file.flush()
                    last_flush = time.monotonic()
            # The sensors are stopped before stop() is called, but a sample
            # could still have been queued behind the stop marker.
            while True:
                try:
                    item = self._queue.get_nowait()
                except queue.Empty:
                    break
                if item is not _STOP:
                    self._write(item)
            self._file.flush()
        except Exception as error:  # for example a full disk
            self.error = error

    def _write(self, sample):
        if self._begin_ns is None or sample.host_ns < self._begin_ns:
            self.warmup_samples += 1
            return
        self._csv.writerow(csv_row(sample))
        self.written += 1
        with self._lock:
            if sample.stream not in self.stats:
                self.stats[sample.stream] = StreamStats(self._rate)
            # Line 1 of the CSV is the header.
            self.stats[sample.stream].add(sample, line=self.written + 1)


def format_elapsed(seconds):
    seconds = int(seconds)
    return f'{seconds // 3600:02d}:{seconds // 60 % 60:02d}:{seconds % 60:02d}'


def _number(value, digits, unit=''):
    return 'n/a' if value is None else f'{value:.{digits}f}{unit}'


def temperature_text(temperatures):
    """The temperatures the camera offers, for example 'asic 41.0 C'."""
    return ', '.join(
        f'{name} {_number(value, 1, " C")}' for name, value in temperatures.items())


def _step_text(step_ms):
    if abs(step_ms) >= 1000:
        return f'{step_ms / 1000:+.3f} s'
    return f'{step_ms:+.3f} ms'


def _plural(count, word):
    return f'{count} {word}' + ('' if count == 1 else 's')


def discontinuities_text(discontinuities):
    """The count, and where the last one was and how big it was."""
    if len(discontinuities) == 1:
        text = '1 clock discontinuity'
    else:
        text = f'{len(discontinuities)} clock discontinuities'
    if discontinuities:
        last = discontinuities[-1]
        text += (f' (last: line {last["line"]}, frame {last["frame_number"]}, '
                 f'{_step_text(last["step_ms"])})')
    return text


def status_line(elapsed, streams, queue_depth, temperatures):
    """One status line: elapsed time, then per stream, queue, temperatures."""
    parts = [format_elapsed(elapsed)]
    for name in STREAMS:
        stream = streams.get(name, {'samples': 0, 'dropped': 0, 'rate': None})
        parts.append(
            f'{name} {stream["samples"]} samples, '
            f'{_number(stream["rate"], 2, " Hz")}, {stream["dropped"]} dropped, '
            f'{discontinuities_text(stream.get("discontinuities", []))}')
    parts.append(f'queue {queue_depth}')
    if temperatures:
        parts.append(temperature_text(temperatures))
    return ' | '.join(parts)


def summarize(stats):
    """The final numbers for each stream, as plain values for the sidecar."""
    streams = {}
    for name, s in stats.items():
        segment = s.longest_segment()
        drift = None if segment is None else segment.summary(s.segments)
        streams[name] = {
            'samples': s.count,
            'ts_domain': s.domain,
            'domain_changes': list(s.domain_changes),
            'rate_hz': s.rate(),
            'measured_period_us': s.period_us(),
            'dropped': s.frame_gaps.dropped,
            'frame_number_backwards': s.frame_gaps.backwards,
            'clock_discontinuities': list(s.discontinuities),
            'missing_hw_timestamp': s.missing_hw,
            'segments': s.segments,
            'drift_segment': drift,
            'drift_ppm': None if drift is None else drift['drift_ppm'],
            'drift_residual_us': None if drift is None else drift['drift_residual_us'],
        }
    return streams


def domain_change_text(change):
    return (f'{change["stream"]}: ts_domain changed from {change["from"]} to '
            f'{change["to"]} at CSV line {change["line"]} '
            f'(frame_number {change["frame_number"]})')


def discontinuity_text(discontinuity):
    d = discontinuity
    text = (f'{d["stream"]}: ts_ms stepped {_step_text(d["step_ms"])} at CSV line {d["line"]} '
            f'(frame_number {d["frame_number"]}), {d["seconds_into_log"]:.3f} s into the log')
    if d['frames_missing']:
        text += f', {_plural(d["frames_missing"], "frame")} missing across it'
    if d['startup']:
        text += '; expected: the startup clock jump (see the README)'
    return text


def segment_text(segment):
    return (f'segment {segment["index"]} of {segment["of"]}: CSV lines '
            f'{segment["first_line"]}-{segment["last_line"]}, frame_number '
            f'{segment["first_frame_number"]}-{segment["last_frame_number"]}, '
            f'{segment["samples"]} samples, {segment["duration_s"]:.1f} s')


def check_streams(streams, rate):
    """Return warnings about anything in the summary that looks wrong.

    A blank hw_timestamp_us is not one: on the Jetson's build the frame
    metadata is unsupported for motion frames, and ts_ms carries the
    camera clock instead. Nor is a clock discontinuity in the first
    STARTUP_SECONDS: that is noted in the report instead.
    """
    warnings = []
    for name in STREAMS:
        s = streams.get(name)
        if s is None or s['samples'] == 0:
            warnings.append(f'{name}: no samples')
            continue
        for change in s['domain_changes']:
            warnings.append(domain_change_text(dict(change, stream=name)))
        if s['domain_changes']:
            warnings.append(
                f'{name}: ts_domain is mixed; the drift fit uses only '
                f'{segment_text(s["drift_segment"])}')
        if s['ts_domain'] != HARDWARE_CLOCK:
            warnings.append(
                f'{name}: ts_domain is {s["ts_domain"]}, not {HARDWARE_CLOCK}; '
                'the drift fit is not against the camera clock')
        for discontinuity in s['clock_discontinuities']:
            if not discontinuity['startup']:
                warnings.append(discontinuity_text(dict(discontinuity, stream=name)))
        measured = s['rate_hz']
        if measured is not None and abs(measured - rate) > RATE_TOLERANCE * rate:
            warnings.append(
                f'{name}: measured {measured:.2f} Hz, more than '
                f'{RATE_TOLERANCE:.0%} from the requested {rate} Hz')
        if s['dropped']:
            warnings.append(f'{name}: {s["dropped"]} dropped frames (gaps in frame_number)')
        if s['frame_number_backwards']:
            warnings.append(
                f'{name}: frame number went backwards '
                f'{s["frame_number_backwards"]} times')
    return warnings


def report_lines(duration, streams, warnings, csv_path, csv_size, error=None):
    lines = ['', 'final report', f'  duration  {format_elapsed(duration)} ({duration:.1f} s)']
    for name in STREAMS:
        s = streams.get(name)
        if s is None:
            continue
        lines.append(
            f'  {name:<5}     {s["samples"]} samples, '
            f'{_number(s["rate_hz"], 2, " Hz")} '
            f'(median period {_number(s["measured_period_us"], 1, " us")}), '
            f'{s["dropped"]} dropped, '
            f'{discontinuities_text(s["clock_discontinuities"])}, ts_domain {s["ts_domain"]}')
    for name in STREAMS:
        s = streams.get(name)
        if s is None:
            continue
        for discontinuity in s['clock_discontinuities']:
            if discontinuity['startup']:
                lines.append(f'  note      {discontinuity_text(dict(discontinuity, stream=name))}')
    for name in STREAMS:
        s = streams.get(name)
        if s is None:
            continue
        lines.append(
            f'  drift {name:<5} host_ns against ts_ms: '
            f'slope {_number(s["drift_ppm"], 2, " ppm")}, '
            f'residual {_number(s["drift_residual_us"], 1, " us")} rms')
        if s['drift_segment'] is not None:
            lines.append(f'            over the longest {segment_text(s["drift_segment"])}')
    lines.append(f'  file      {csv_path} ({csv_size / 1e6:.1f} MB)')
    if error is not None:
        lines.append(f'  ERROR     {error}')
    for warning in warnings:
        lines.append(f'  WARNING   {warning}')
    if not warnings and error is None:
        lines.append(
            f'  checks    ok: rates within {RATE_TOLERANCE:.0%} of --rate, no dropped frames, '
            f'no clock discontinuities after startup, ts_ms in {HARDWARE_CLOCK} throughout')
    return lines


def git_commit():
    """Return (commit, dirty) of the repo this file is in, or (None, None)."""
    here = os.path.dirname(os.path.abspath(__file__))
    try:
        commit = subprocess.run(
            ['git', '-C', here, 'rev-parse', 'HEAD'],
            capture_output=True, text=True, timeout=10, check=True).stdout.strip()
        status = subprocess.run(
            ['git', '-C', here, 'status', '--porcelain'],
            capture_output=True, text=True, timeout=10, check=True).stdout
    except (OSError, subprocess.SubprocessError):
        return None, None
    return commit, bool(status.strip())


def write_json(path, data):
    """Write JSON through a temporary file, so a crash never leaves half a file."""
    temporary = path + '.tmp'
    with open(temporary, 'w') as file:
        json.dump(data, file, indent=2)
        file.write('\n')
    os.replace(temporary, path)
    hand_to_user(path)


def describe(info):
    """Startup lines about the camera, so a log shows what was opened."""
    lines = [
        f'camera    {info.get("name")} serial {info.get("serial")} '
        f'firmware {info.get("firmware")} USB {info.get("usb_type")}',
        f'library   librealsense {info.get("librealsense_version")} '
        f'from {info.get("pyrealsense2_file")}',
    ]
    for name, stream in info.get('motion_streams', {}).items():
        lines.append(f'{name:<9} {stream.get("fps")} Hz {stream.get("format")}')
    if info.get('global_time_enabled'):
        lines.append('global time enabled (0: ts_ms is the camera clock) = ' + ', '.join(
            f'{sensor} {value}' for sensor, value in info['global_time_enabled'].items()))
    if info.get('motion_correction') is not None:
        lines.append(f'motion correction (applies the IMU calibration) = {info["motion_correction"]}')
    if info.get('depth_streams'):
        lines.append('depth     ' + ', '.join(
            f'{s["stream"]} {s["width"]}x{s["height"]} {s["fps"]} fps {s["format"]}'
            for s in info['depth_streams']))
    if 'temperature_options' in info:
        offered = info['temperature_options']
        if offered:
            lines.append('temperature ' + ', '.join(
                f'{option} on {sensor}' for option, sensor in offered.items()))
        else:
            lines.append('temperature no sensor offers a temperature option; '
                         'the status lines leave it out')
    return lines


def run_log(camera, folder, warmup, duration, rate, options, stop,
            clock=datetime.now, status_interval=STATUS_INTERVAL, note=None):
    """Warm up, log until duration or stop, then flush and report.

    camera is a started-on-demand object with start(on_sample), stop(),
    info(), temperatures() and temperature_problems(); open_camera returns
    the real one. stop is a
    threading.Event: setting it (Ctrl+C) ends the run cleanly. note is free
    text for the sidecar, such as how the camera was mounted. Returns the
    path of the CSV.
    """
    os.makedirs(folder, exist_ok=True)
    hand_to_user(folder)
    started = clock()
    base = os.path.join(folder, log_name(started))
    csv_path = base + '.csv'
    json_path = base + '.json'
    status_path = base + '.status.log'

    commit, dirty = git_commit()
    info = camera.info()
    sidecar = {
        'csv': os.path.basename(csv_path),
        'note': note,
        'camera': info,
        'global_time_enabled': info.get('global_time_enabled'),
        'options': options,
        'hostname': socket.gethostname(),
        'git_commit': commit,
        'git_dirty': dirty,
        'python': sys.version.split()[0],
        'started': started.isoformat(),
        'logging_started': None,
        'ended': None,
        'units': {'gyro': 'rad/s', 'accel': 'm/s^2'},
        'csv_fields': list(CSV_FIELDS),
    }
    write_json(json_path, sidecar)

    with open(csv_path, 'w', newline='') as csv_file, \
            open(status_path, 'a') as status_file:
        for path in (csv_path, status_path):
            hand_to_user(path)

        def say(line):
            print(line, flush=True)
            status_file.write(line + '\n')
            status_file.flush()

        for line in describe(sidecar['camera']):
            say(line)
        if note:
            say(f'note      {note}')
        say(f'writing   {csv_path}')

        writer = Writer(csv_file, rate)
        logging_start = None
        changes_said = 0
        discontinuities_said = 0
        problems_said = set()

        def loud(text):
            say('!' * 72)
            say(f'WARNING   {text}')
            say('!' * 72)

        def say_clock_events():
            # Loud, because either one splits the file into separate clocks.
            # The startup clock jump is expected and only noted.
            nonlocal changes_said, discontinuities_said
            changes = writer.domain_changes()
            for change in changes[changes_said:]:
                loud(domain_change_text(change))
            changes_said = len(changes)
            discontinuities = writer.discontinuities()
            for discontinuity in discontinuities[discontinuities_said:]:
                if discontinuity['startup']:
                    say(f'note      {discontinuity_text(discontinuity)}')
                else:
                    loud(discontinuity_text(discontinuity))
            discontinuities_said = len(discontinuities)

        def temperatures():
            # A reading that fails is n/a on every line; why is said once.
            readings = camera.temperatures()
            for name, problem in camera.temperature_problems().items():
                if name not in problems_said:
                    say(f'warning   could not read {problem}')
                    problems_said.add(name)
            return readings

        try:
            camera.start(writer.put)

            # Warm-up: the streams run so the sensor reaches its working
            # temperature, and the writer discards what arrives.
            warmup_end = time.monotonic() + warmup
            if warmup:
                say(f'warm-up   {format_elapsed(warmup)}: streaming, nothing written')
            next_line = time.monotonic() + status_interval
            while not stop.is_set() and time.monotonic() < warmup_end:
                if time.monotonic() >= next_line:
                    left = warmup_end - time.monotonic()
                    say(f'warm-up {format_elapsed(warmup - left)} of {format_elapsed(warmup)} | '
                        f'{writer.warmup_samples} samples discarded | '
                        f'{temperature_text(temperatures())}')
                    next_line += status_interval
                stop.wait(max(0.0, min(1.0, warmup_end - time.monotonic(),
                                       next_line - time.monotonic())))

            if not stop.is_set():
                writer.begin(time.time_ns())
                logging_start = time.monotonic()
                sidecar['logging_started'] = clock().isoformat()
                write_json(json_path, sidecar)
                say(f'logging   started {sidecar["logging_started"]}'
                    + (f', for {duration} s' if duration else ', until Ctrl+C'))

                end = logging_start + duration if duration else math.inf
                next_line = logging_start + status_interval
                while not stop.is_set() and writer.error is None:
                    now = time.monotonic()
                    if now >= end:
                        break
                    say_clock_events()
                    if now >= next_line:
                        say(status_line(now - logging_start, writer.snapshot(),
                                        writer.queue_depth(), temperatures()))
                        next_line += status_interval
                    stop.wait(max(0.0, min(1.0, end - now, next_line - now)))
            if stop.is_set():
                say('stopping: Ctrl+C or a stop signal')
        finally:
            elapsed = 0.0 if logging_start is None else time.monotonic() - logging_start
            # Order matters for the tail of the data: stop the sensors so no
            # more samples come, then let the writer drain the queue.
            try:
                camera.stop()
            except Exception as error:
                say(f'warning: stopping the camera failed: {error}')
            writer.stop()
            say_clock_events()
            end_temperatures = temperatures()
            say(status_line(elapsed, writer.snapshot(), writer.queue_depth(), end_temperatures))
            streams = summarize(writer.stats)
            warnings = check_streams(streams, rate) if logging_start is not None else []
            csv_file.flush()
            for line in report_lines(elapsed, streams, warnings, csv_path,
                                     os.path.getsize(csv_path), writer.error):
                say(line)
            sidecar['ended'] = clock().isoformat()
            sidecar['report'] = {
                'duration_s': elapsed,
                'streams': streams,
                'warnings': warnings,
                'warmup_samples_discarded': writer.warmup_samples,
                'rows_written': writer.written,
                'end_temperatures': end_temperatures,
                'error': None if writer.error is None else repr(writer.error),
            }
            write_json(json_path, sidecar)
    return csv_path


# Everything below talks to the camera. It is the only code that needs
# pyrealsense2.

def open_camera(librealsense, rate, depth, depth_fps):
    """Import pyrealsense2 from the librealsense build folder and open the camera."""
    path = os.path.expanduser(librealsense)
    sys.path.insert(0, path)
    try:
        import pyrealsense2 as rs
    except ImportError as error:
        raise RuntimeError(
            f'could not import pyrealsense2 with {path} on the path '
            f'(set it with --librealsense): {error}') from error
    return RealSenseImu(rs, rate, depth, depth_fps)


def _discard(frame):
    """Depth callback: the frames are not needed, only the heat they make."""


class RealSenseImu:
    """The D435i's motion sensor, opened directly, plus optional depth.

    No pipeline and no framesets: those pair IMU samples with image frames
    and drop the rest.
    """

    def __init__(self, rs, rate, depth, depth_fps):
        self._rs = rs
        # The context is librealsense's handle on USB; it must outlive the
        # sensors, so it is kept for the lifetime of this object.
        self._context = rs.context()
        devices = self._context.query_devices()
        if len(devices) == 0:
            raise RuntimeError('no RealSense camera is connected')
        self._device = devices[0]
        self._motion = self._find_motion_sensor()
        self._motion_profiles = [
            self._motion_profile(rs.stream.accel, rate),
            self._motion_profile(rs.stream.gyro, rate),
        ]
        try:
            self._depth_sensor = self._device.first_depth_sensor()
        except Exception:
            self._depth_sensor = None
        self._depth_profiles = []
        if depth:
            if self._depth_sensor is None:
                raise RuntimeError('--depth: the camera has no depth sensor')
            self._depth_profiles = self._depth_stream_profiles(depth_fps)
        self._global_time = self._disable_global_time()
        self._temperature_options = self._find_temperature_options()
        self._temperature_problems = {}
        self._opened = []
        self._started = []

    def _disable_global_time(self):
        """Set global_time_enabled to 0, so ts_ms is the camera clock.

        Done before the sensors are opened. The motion sensor must support
        it; the depth sensor is set too when it does. Returns the values
        read back, by sensor name, None where the option is unsupported.
        """
        option = self._rs.option.global_time_enabled
        values = {}
        for sensor, required in ((self._motion, True), (self._depth_sensor, False)):
            if sensor is None:
                continue
            name = sensor.get_info(self._rs.camera_info.name)
            if not sensor.supports(option):
                if required:
                    raise RuntimeError(
                        f'{name} does not support global_time_enabled, so ts_ms '
                        'cannot be kept in the hardware_clock domain')
                values[name] = None
                continue
            sensor.set_option(option, 0)
            value = sensor.get_option(option)
            if value != 0:
                raise RuntimeError(
                    f'{name}: global_time_enabled was set to 0 but reads back {value}')
            values[name] = value
        return values

    def _find_temperature_options(self):
        """Every *_temperature option offered by any of the camera's sensors.

        Returns {short name: (sensor, option, sensor name)}, for example
        'asic' for asic_temperature. Each sensor is asked with
        get_supported_options rather than assuming the depth sensor has
        them, and the startup lines say which sensor offered which.
        """
        rs = self._rs
        found = {}
        for sensor in self._device.query_sensors():
            sensor_name = sensor.get_info(rs.camera_info.name)
            for option in sensor.get_supported_options():
                text = enum_text(option)
                if text.endswith(TEMPERATURE_SUFFIX):
                    found.setdefault(text[:-len(TEMPERATURE_SUFFIX)], (sensor, option, sensor_name))
        return found

    def _find_motion_sensor(self):
        rs = self._rs
        for sensor in self._device.query_sensors():
            types = [p.stream_type() for p in sensor.get_stream_profiles()]
            if rs.stream.accel in types and rs.stream.gyro in types:
                return sensor
        raise RuntimeError('the camera has no sensor with both accel and gyro streams')

    def _motion_profile(self, stream, rate):
        # By stream type and rate only: asking for the motion_xyz32f format
        # explicitly failed on the Jetson's build (see perch_perception).
        profiles = [p for p in self._motion.get_stream_profiles() if p.stream_type() == stream]
        for profile in profiles:
            if profile.fps() == rate:
                return profile
        offered = sorted({p.fps() for p in profiles})
        raise RuntimeError(f'{enum_text(stream)} does not offer {rate} Hz; it offers {offered} Hz')

    def _depth_stream_profiles(self, fps):
        rs = self._rs
        wanted = [(rs.stream.depth, rs.format.z16), (rs.stream.infrared, rs.format.y8)]
        chosen = []
        for stream, fmt in wanted:
            for profile in self._depth_sensor.get_stream_profiles():
                if (profile.stream_type() != stream or profile.format() != fmt
                        or profile.fps() != fps or not profile.is_video_stream_profile()):
                    continue
                if stream == rs.stream.infrared and profile.stream_index() != LEFT_IR:
                    continue
                video = profile.as_video_stream_profile()
                if video.width() == DEPTH_WIDTH and video.height() == DEPTH_HEIGHT:
                    chosen.append(profile)
                    break
            else:
                raise RuntimeError(
                    f'--depth: no {enum_text(stream)} {enum_text(fmt)} profile at '
                    f'{DEPTH_WIDTH}x{DEPTH_HEIGHT} {fps} fps')
        return chosen

    def start(self, on_sample):
        """Start the streams. on_sample(Sample) is called for every motion frame."""
        accel = self._rs.stream.accel
        frame_timestamp = self._rs.frame_metadata_value.frame_timestamp

        def on_motion_frame(frame):
            host_ns = time.time_ns()
            on_sample(motion_sample(frame, host_ns, accel, frame_timestamp))

        if self._depth_profiles:
            self._depth_sensor.open(self._depth_profiles)
            self._opened.append(self._depth_sensor)
            self._depth_sensor.start(_discard)
            self._started.append(self._depth_sensor)
        self._motion.open(self._motion_profiles)
        self._opened.append(self._motion)
        self._motion.start(on_motion_frame)
        self._started.append(self._motion)

    def stop(self):
        """Stop and close every sensor that was started. Safe to call twice."""
        for sensor in reversed(self._started):
            try:
                sensor.stop()
            except Exception as error:
                print(f'warning: stopping a sensor failed: {error}', flush=True)
        self._started = []
        for sensor in reversed(self._opened):
            try:
                sensor.close()
            except Exception as error:
                print(f'warning: closing a sensor failed: {error}', flush=True)
        self._opened = []

    def temperatures(self):
        """Each offered temperature in deg C, None where the read failed.

        librealsense refuses some of these unless the sensor is streaming,
        so a failure is kept in temperature_problems() rather than hidden.
        """
        readings = {}
        for name, (sensor, option, sensor_name) in self._temperature_options.items():
            try:
                readings[name] = sensor.get_option(option)
            except Exception as error:
                readings[name] = None
                self._temperature_problems[name] = (
                    f'{enum_text(option)} from {sensor_name}: {error}')
        return readings

    def temperature_problems(self):
        """Why a temperature read failed, by short name, for each that ever did."""
        return dict(self._temperature_problems)

    def info(self):
        """Camera, library, chosen streams, IMU calibration and sensor options."""
        rs = self._rs
        device = self._device

        def camera_info(field):
            return device.get_info(field) if device.supports(field) else None

        motion_streams = {}
        for profile in self._motion_profiles:
            entry = {'fps': profile.fps(), 'format': enum_text(profile.format())}
            try:
                intrinsics = profile.as_motion_stream_profile().get_motion_intrinsics()
                entry['intrinsics'] = {
                    'data': [[float(v) for v in row] for row in intrinsics.data],
                    'noise_variances': [float(v) for v in intrinsics.noise_variances],
                    'bias_variances': [float(v) for v in intrinsics.bias_variances],
                }
            except Exception as error:
                entry['intrinsics'] = None
                entry['intrinsics_error'] = str(error)
            motion_streams[enum_text(profile.stream_type())] = entry

        depth_streams = []
        for profile in self._depth_profiles:
            video = profile.as_video_stream_profile()
            depth_streams.append({
                'stream': enum_text(profile.stream_type()),
                'index': profile.stream_index(),
                'width': video.width(),
                'height': video.height(),
                'fps': profile.fps(),
                'format': enum_text(profile.format()),
            })

        sensors = {}
        for sensor in (self._motion, self._depth_sensor):
            if sensor is not None:
                sensors[sensor.get_info(rs.camera_info.name)] = _sensor_options(sensor)

        motion_correction = None
        if self._motion.supports(rs.option.enable_motion_correction):
            motion_correction = self._motion.get_option(rs.option.enable_motion_correction)

        return {
            'name': camera_info(rs.camera_info.name),
            'serial': camera_info(rs.camera_info.serial_number),
            'firmware': camera_info(rs.camera_info.firmware_version),
            'usb_type': camera_info(rs.camera_info.usb_type_descriptor),
            'librealsense_version': getattr(rs, '__version__', None),
            'pyrealsense2_file': getattr(rs, '__file__', None),
            'motion_streams': motion_streams,
            'motion_correction': motion_correction,
            'global_time_enabled': self._global_time,
            'depth_streams': depth_streams,
            'sensor_options': sensors,
            'temperature_options': {
                enum_text(option): sensor_name
                for sensor, option, sensor_name in self._temperature_options.values()},
        }


def _sensor_options(sensor):
    """Every option the sensor supports, with its current value."""
    values = {}
    for option in sensor.get_supported_options():
        try:
            values[enum_text(option)] = sensor.get_option(option)
        except Exception as error:
            values[enum_text(option)] = f'unreadable: {error}'
    return values


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        '--librealsense', default=default_librealsense(),
        help='folder holding the pyrealsense2 module (default: %(default)s)')
    parser.add_argument(
        '--rate', type=int, default=200,
        help='accel and gyro rate in Hz (default: %(default)s)')
    parser.add_argument(
        '--duration', type=float, default=0,
        help='seconds to log after the warm-up; 0 logs until Ctrl+C (default)')
    parser.add_argument(
        '--warmup', type=float, default=1200,
        help='seconds of streaming before logging starts, so the sensor reaches '
             'its working temperature (default: %(default)s)')
    parser.add_argument(
        '--depth', action='store_true',
        help=f'also stream depth and left IR at {DEPTH_WIDTH}x{DEPTH_HEIGHT}, '
             'so the camera runs as warm as in flight. Use it for the characterization run.')
    parser.add_argument(
        '--depth-fps', type=int, default=30,
        help='depth and IR frame rate with --depth (default: %(default)s)')
    parser.add_argument(
        '--out', default=default_folder(),
        help='folder for the logs (default: vespr_recordings/imu in your home folder)')
    parser.add_argument(
        '--dry-run', action='store_true',
        help='check the setup: same as --warmup 0 --duration 60')
    parser.add_argument(
        '--note', default=None,
        help='free text written into the .json sidecar, for example how the camera '
             'was mounted and which way it faced')
    args = parser.parse_args(argv)
    if args.dry_run:
        args.warmup = 0
        args.duration = 60
    if args.duration < 0 or args.warmup < 0:
        parser.error('--duration and --warmup cannot be negative')
    return args


def main(argv=None):
    args = parse_args(argv)
    stop = threading.Event()

    def on_signal(signum, frame):
        stop.set()
        # A second Ctrl+C kills the program, in case stopping hangs.
        signal.signal(signal.SIGINT, signal.SIG_DFL)

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    camera = open_camera(args.librealsense, args.rate, args.depth, args.depth_fps)
    run_log(camera, args.out, args.warmup, args.duration, args.rate, vars(args), stop,
            note=args.note)
    return 0


if __name__ == '__main__':
    sys.exit(main())
