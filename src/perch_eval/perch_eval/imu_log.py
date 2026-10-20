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

# The hardware timestamp may be a 32-bit microsecond counter, which wraps
# every 71.6 minutes. Rates and the drift fit unwrap it; the CSV keeps the
# raw value. A wider counter never goes backwards by half this span, so
# unwrapping leaves it alone.
HW_TIMESTAMP_BITS = 32

DEPTH_WIDTH = 848
DEPTH_HEIGHT = 480
LEFT_IR = 1  # RealSense numbers the infrared imagers 1 (left) and 2 (right)

RATE_TOLERANCE = 0.01  # a measured rate further than 1 % from --rate is flagged

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


class Unwrapper:
    """Turns a wrapping counter into one that keeps increasing."""

    def __init__(self, bits):
        self.span = 1 << bits
        self.offset = 0
        self.last = None

    def __call__(self, value):
        if self.last is not None and value < self.last - self.span // 2:
            self.offset += self.span
        self.last = value
        return value + self.offset


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


class StreamStats:
    """Running counts, rates, drops and clock drift for one stream."""

    def __init__(self):
        self.count = 0
        self.drops = DropCounter()
        self.missing_hw = 0
        self._unwrap = Unwrapper(HW_TIMESTAMP_BITS)
        self.hw_count = 0  # samples that carried hw_timestamp_us
        self.first_hw = None  # unwrapped, microseconds
        self.last_hw = None
        self.first_ts = None  # ts_ms
        self.last_ts = None
        # host_ns - hw_timestamp_us against hw time, both relative to the
        # first sample: x in seconds, y in nanoseconds.
        self.drift = LinearFit()
        self._origin = None
        self._mark = None  # (hw_count, last_hw) at the last status line

    def add(self, sample):
        self.count += 1
        self.drops.add(sample.frame_number)
        if self.first_ts is None:
            self.first_ts = sample.ts_ms
        self.last_ts = sample.ts_ms
        if sample.hw_timestamp_us is None:
            self.missing_hw += 1
            return
        hw = self._unwrap(sample.hw_timestamp_us)
        offset_ns = sample.host_ns - hw * 1000
        if self._origin is None:
            self._origin = (hw, offset_ns)
            self.first_hw = hw
        self.hw_count += 1
        self.last_hw = hw
        self.drift.add(
            (hw - self._origin[0]) / 1e6, float(offset_ns - self._origin[1]))

    def mean_rate(self):
        """Mean rate in Hz from hw_timestamp_us, None without two of them."""
        return _rate(self.hw_count - 1, self.first_hw, self.last_hw, 1e6)

    def mean_rate_ts(self):
        """Mean rate in Hz from ts_ms, a cross-check on the hardware clock."""
        return _rate(self.count - 1, self.first_ts, self.last_ts, 1e3)

    def window_rate(self):
        """Rate in Hz from hw_timestamp_us since the last call."""
        if self._mark is None:
            rate = self.mean_rate()
        else:
            count, hw = self._mark
            rate = _rate(self.hw_count - count, hw, self.last_hw, 1e6)
        if self.last_hw is not None:
            self._mark = (self.hw_count, self.last_hw)
        return rate

    def drift_ppm(self):
        slope = self.drift.slope()  # nanoseconds per second
        return None if slope is None else slope / 1e3

    def drift_residual_ms(self):
        residual = self.drift.residual_rms()  # nanoseconds
        return None if residual is None else residual / 1e6


def _rate(intervals, start, end, per_second):
    if start is None or end is None or intervals < 1 or end <= start:
        return None
    return intervals * per_second / (end - start)


class Writer:
    """Drains the sample queue into the CSV on its own thread.

    put() is the only thing the camera callback calls. Samples that arrived
    before begin() are the warm-up: they are counted and not written.
    stop() writes everything still queued, then flushes the file.
    """

    def __init__(self, file):
        self._file = file
        self._csv = csv.writer(file)
        self._csv.writerow(CSV_FIELDS)
        self._queue = queue.SimpleQueue()
        self.put = self._queue.put
        self._lock = threading.Lock()
        self.stats = {name: StreamStats() for name in STREAMS}
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
        """Counts, drops and rate since the last snapshot, per stream."""
        with self._lock:
            return {
                name: {
                    'samples': stats.count,
                    'dropped': stats.drops.dropped,
                    'rate': stats.window_rate(),
                }
                for name, stats in self.stats.items()
            }

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
                self.stats[sample.stream] = StreamStats()
            self.stats[sample.stream].add(sample)


def format_elapsed(seconds):
    seconds = int(seconds)
    return f'{seconds // 3600:02d}:{seconds // 60 % 60:02d}:{seconds % 60:02d}'


def _number(value, digits, unit=''):
    return 'n/a' if value is None else f'{value:.{digits}f}{unit}'


def temperature_text(temperatures):
    return ', '.join(
        f'{name} {_number(temperatures.get(name), 1, " C")}'
        for name in ('asic', 'projector'))


def status_line(elapsed, streams, queue_depth, temperatures):
    """One status line: elapsed time, then per stream, queue, temperatures."""
    parts = [format_elapsed(elapsed)]
    for name in STREAMS:
        stream = streams.get(name, {'samples': 0, 'dropped': 0, 'rate': None})
        parts.append(
            f'{name} {stream["samples"]} samples, '
            f'{_number(stream["rate"], 2, " Hz")}, {stream["dropped"]} dropped')
    parts.append(f'queue {queue_depth}')
    parts.append(temperature_text(temperatures))
    return ' | '.join(parts)


def summarize(stats):
    """The final numbers for each stream, as plain values for the sidecar."""
    return {
        name: {
            'samples': s.count,
            'mean_rate_hz': s.mean_rate(),
            'mean_rate_ts_ms_hz': s.mean_rate_ts(),
            'dropped': s.drops.dropped,
            'frame_number_backwards': s.drops.backwards,
            'missing_hw_timestamp': s.missing_hw,
            'drift_ppm': s.drift_ppm(),
            'drift_residual_ms': s.drift_residual_ms(),
        }
        for name, s in stats.items()
    }


def check_streams(streams, rate):
    """Return warnings about anything in the summary that looks wrong."""
    warnings = []
    for name in STREAMS:
        s = streams.get(name)
        if s is None or s['samples'] == 0:
            warnings.append(f'{name}: no samples')
            continue
        if s['missing_hw_timestamp']:
            warnings.append(
                f'{name}: {s["missing_hw_timestamp"]} samples without '
                'hw_timestamp_us (frame metadata unsupported?)')
        measured = s['mean_rate_hz'] or s['mean_rate_ts_ms_hz']
        if measured is not None and abs(measured - rate) > RATE_TOLERANCE * rate:
            warnings.append(f'{name}: measured {measured:.2f} Hz, requested {rate} Hz')
        if s['dropped']:
            warnings.append(f'{name}: {s["dropped"]} dropped frames')
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
            f'mean {_number(s["mean_rate_hz"], 3, " Hz")} '
            f'(ts_ms: {_number(s["mean_rate_ts_ms_hz"], 3, " Hz")}), '
            f'{s["dropped"]} dropped')
    for name in STREAMS:
        s = streams.get(name)
        if s is None:
            continue
        lines.append(
            f'  drift {name:<5} host_ns - hw_timestamp_us: '
            f'slope {_number(s["drift_ppm"], 2, " ppm")}, '
            f'residual {_number(s["drift_residual_ms"], 3, " ms")} rms')
    lines.append(f'  file      {csv_path} ({csv_size / 1e6:.1f} MB)')
    if error is not None:
        lines.append(f'  ERROR     {error}')
    for warning in warnings:
        lines.append(f'  WARNING   {warning}')
    if not warnings and error is None:
        lines.append('  checks    ok: rates within 1 %, no drops, hw timestamps present')
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
    if info.get('motion_correction') is not None:
        lines.append(f'motion correction (applies the IMU calibration) = {info["motion_correction"]}')
    if info.get('depth_streams'):
        lines.append('depth     ' + ', '.join(
            f'{s["stream"]} {s["width"]}x{s["height"]} {s["fps"]} fps {s["format"]}'
            for s in info['depth_streams']))
    return lines


def run_log(camera, folder, warmup, duration, rate, options, stop,
            clock=datetime.now, status_interval=STATUS_INTERVAL):
    """Warm up, log until duration or stop, then flush and report.

    camera is a started-on-demand object with start(on_sample), stop(),
    info() and temperatures(); open_camera returns the real one. stop is a
    threading.Event: setting it (Ctrl+C) ends the run cleanly. Returns the
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
    sidecar = {
        'csv': os.path.basename(csv_path),
        'camera': camera.info(),
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
        say(f'writing   {csv_path}')

        writer = Writer(csv_file)
        logging_start = None
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
                        f'{temperature_text(camera.temperatures())}')
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
                    if now >= next_line:
                        say(status_line(now - logging_start, writer.snapshot(),
                                        writer.queue_depth(), camera.temperatures()))
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
            temperatures = camera.temperatures()
            say(status_line(elapsed, writer.snapshot(), writer.queue_depth(), temperatures))
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
                'end_temperatures': temperatures,
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
        self._opened = []
        self._started = []

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
        """asic and projector temperatures in deg C, None where unavailable."""
        rs = self._rs
        readings = {}
        for name, option in (('asic', rs.option.asic_temperature),
                             ('projector', rs.option.projector_temperature)):
            readings[name] = None
            if self._depth_sensor is None:
                continue
            try:
                if self._depth_sensor.supports(option):
                    readings[name] = self._depth_sensor.get_option(option)
            except Exception:
                pass
        return readings

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
            'depth_streams': depth_streams,
            'sensor_options': sensors,
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
    run_log(camera, args.out, args.warmup, args.duration, args.rate, vars(args), stop)
    return 0


if __name__ == '__main__':
    sys.exit(main())
