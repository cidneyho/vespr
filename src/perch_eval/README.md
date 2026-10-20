# perch_eval

Ground-truth and evaluation tooling: the measurements that tell us how good the drone's sensors
and estimates are. The first tool characterizes the D435i's IMU. `imu_log.py` records hours of
raw, stationary accelerometer and gyroscope data for an Allan variance analysis. The analysis
script, `imu_allan.py`, is being written separately and is not in this package yet.

**Package name.** [NAMING.md](../../NAMING.md) calls this package `vespr_eval`. It is named
`perch_eval` to match the existing `perch_*` packages. Renaming the whole repo to `vespr_*`
is a team decision and has not been made yet; when it is, this package is renamed with the rest.

## Layout

```
perch_eval/
  imu_log.py            standalone IMU logger for the D435i (no ROS needed)
calibration/            raw data from the 2026-10-20 IMU calibration (see below)
resource/
  perch_eval            empty marker file that lets ROS find the package (colcon needs it)
test/                   pytest tests; no camera needed
```

`calibration/` holds the raw samples that `rs-imu-calibration` recorded while
calibrating this camera (D435i, serial 348522076012) on 2026-10-20. The result of that
calibration was written to the camera. Each row is `timestamp_ms, x, y, z`. The accel file has
six blocks of 1000 samples at about 100 Hz, in m/s², one block per orientation. The gyro file
has about 10 s at 200 Hz, in rad/s.

## Why the logger opens the camera itself

perch_perception's `LiveSource` reads framesets from a pipeline. Each frameset carries one IMU
sample per camera frame, and the samples in between are thrown away. For Allan variance every
sample is needed. So `imu_log.py` does not use `pipeline` or framesets. It finds the motion
sensor, opens its accel and gyro profiles, and starts it with a callback. librealsense then
hands over every sample, one frame at a time.

The callback does nothing except take `time.time_ns()` and put the frame's fields on a queue.
A writer thread drains the queue to disk. If the callback is slow, librealsense drops frames.

The motion streams are requested by stream type and rate only. perch_perception found that
asking for the `motion_xyz32f` format explicitly failed on the Jetson's build.

### Which clock `ts_ms` is in

`frame.get_timestamp()` reports in the domain librealsense chooses. By default that is
`global_time`, librealsense's mapping of the camera clock onto the host's epoch milliseconds.
On the Jetson the first dry run showed the first 13 or so samples in `hardware_clock` (the
camera's own clock, milliseconds with microsecond resolution) and the rest in `global_time`.
The jump broke the rate and drift numbers.

So before the sensors start, the logger sets `global_time_enabled` to 0 on the motion sensor
and, if it supports the option, on the depth sensor, then reads it back. If the motion sensor
does not support it or it does not read back 0, the logger stops with an error. The values are
printed at startup and recorded in the sidecar as `global_time_enabled`. Every `ts_ms` is
then in `hardware_clock`.

The frame metadata `frame_timestamp` is unsupported for motion frames on the Jetson's build, so
`hw_timestamp_us` is blank there. That is expected and not warned about. The rates, clock
discontinuities and drift fit come from `ts_ms`; dropped frames come from `frame_number`.

If the domain still changes in the middle of a file, the logger prints a warning between lines
of `!` as soon as it sees it, naming the stream and the CSV line, and repeats it in the final
report. The step between the two clocks is not counted as an interval, and the drift fit uses
one stretch in one domain (see below).

### Drops, clock discontinuities and the rate

These are three separate things, per stream:

- **Dropped frames** are gaps in `frame_number`, and nothing else. That is the authoritative
  count.
- **Clock discontinuities** are steps in `ts_ms` outside 0.5 to 1.5 times the median period:
  a jump forward, a step back, or a repeat. The band is set from the period at `--rate` until a
  stream has 10 intervals, then from the median so far. Each one is recorded with its CSV line,
  `frame_number`, size, how many seconds into the log it came (by `host_ns`) and how many frames
  were missing across it. One in the first second of logging is the startup clock jump (below)
  and is only noted. Any later one is printed between lines of `!` as soon as it is seen and is
  a warning in the final report. A dropped frame also shows up as a discontinuity, with its
  missing frames counted.
- **The rate** is 1 / the median interval between consecutive `ts_ms`. It is warned about when
  it is more than 2 % from `--rate`.

A clock discontinuity or a domain change starts a new segment. The drift fit of `host_ns`
against `ts_ms` is over the longest segment only, and the report says which one: its CSV lines,
`frame_number` range, samples and length. `host_ns` is never the time base: it is when the
callback ran, which jitters by up to 22 ms on the Jetson.

`ts_ms` is not unwrapped. There is no evidence that the camera clock wraps, and unwrapping could
only add errors. If it ever does wrap, that shows up as a clock discontinuity.

### Known behaviour of this unit (D435i, serial 348522076012)

Measured on 2026-10-20 in the second 60 s dry run on the Jetson, with global time off:

- accel runs at **203.2 Hz** (period 4.920 to 4.924 ms) and gyro at **200.5 Hz** (4.987 to
  4.989 ms), with 200 Hz requested. The rates are off by a fixed amount, and the intervals are
  very regular. Both are within the 2 % tolerance, so they are not warned about. The analysis
  should take the period from the data, not from `--rate`.
- `ts_ms` jumps forward once per stream by about 41.2 s within the first 30 ms after the sensor
  starts: at accel sample 5 and gyro sample 6. `frame_number` has no gap there, so no samples
  are missing; the camera clock steps. The earlier version of the logger read this one jump as
  8245 dropped samples, a rate of 120 Hz and a meaningless drift fit.

The warm-up discards the jump: it happens in the first milliseconds of streaming, long before
logging starts. A dry run (`--warmup 0`) logs it, and the report notes it as the expected
startup jump.

### Temperatures

The D435i's IMU has no temperature reading of its own. The nearest proxies are the depth
module's `asic_temperature` and `projector_temperature`. At startup the logger asks every
sensor for its supported options and prints which sensor offers each `*_temperature` option,
for example `temperature asic_temperature on Stereo Module, ...`. The list is also in the
sidecar as `temperature_options`. Only the offered ones are shown in the status lines; if no
sensor offers any, the startup line says so and the status lines leave temperatures out.

If a read fails, the reading shows as `n/a` and the reason librealsense gave is printed once,
starting `warning   could not read`. librealsense may refuse these options unless the
depth sensor is streaming, so without `--depth` they can read `n/a`.

The second dry run printed `n/a` for both even with `--depth`. Earlier versions read them only
from the depth sensor and dropped the error, so why is not known yet. The next run will show
which sensor offers them and why the read fails.

pyrealsense2 is only imported when the camera is opened, from the folder given by
`--librealsense`. On the Jetson, librealsense 2.58.4 is built from source and not installed,
so the module is not on `PYTHONPATH`. The default folder is `~/librealsense/build/Release`.

## Running it on the Jetson

No ROS is needed. Run it from this folder:

```bash
cd ~/vespr/src/perch_eval     # wherever the repo is cloned on the Jetson
python3 -m perch_eval.imu_log --help
```

### First, the dry run

```bash
python3 -m perch_eval.imu_log --dry-run --depth
```

`--dry-run` means `--warmup 0 --duration 60`: one minute of logging, then the final report.
Before trusting an overnight run, check that the report shows:

- `global time enabled ... = Motion Module 0.0, Stereo Module 0.0` at startup
- both streams within 2 % of 200 Hz (this unit: 203.2 and 200.5 Hz) and `ts_domain hardware_clock`
- 0 dropped frames, and no warning that `ts_domain` changed
- at most one clock discontinuity per stream, the startup jump, noted as expected; any
  warning about a later one needs looking into
- that `motion correction` reads 1 at startup, and that the `.json` sidecar has the motion
  intrinsics: together they record that the calibration from 2026-10-20 was applied
- the `temperature` line at startup, and whether the status lines show numbers (see
  [Temperatures](#temperatures))
- a drift slope of a few tens of ppm or less, over a segment covering almost the whole minute

Record how the camera is mounted with `--note`, for example
`--note "vise on the bench, lens facing the wall, USB to the left"`.

### The overnight characterization run

The setup:

1. Clamp the camera to something heavy, such as a steel block or a vise on a solid bench. Any
   movement shows up in the data as noise.
2. Strain-relieve the USB cable: tape or tie it down near the camera, so a tug on the cable
   cannot move the camera.
3. Use a room nobody will enter. No foot traffic, doors, fans blowing on the camera, or
   machines running on the same bench.
4. Use `--depth`. This also streams depth and left IR at 848x480 and 30 fps, which heats the
   camera the way flight does. IMU noise depends on temperature, so this is the setting to use
   for the characterization run. Its frames are discarded.
5. Keep the default 20 min warm-up (`--warmup 1200`). The streams run but nothing is written,
   so the sensor reaches thermal equilibrium before logging starts.
6. Log for 3 hours or more. Allan deviation is only trustworthy up to averaging times of about
   a tenth of the recording, and bias instability needs averaging times of hundreds of seconds.

Run it in tmux, so it survives the SSH session closing:

```bash
tmux new -s imu
cd ~/vespr/src/perch_eval
python3 -u -m perch_eval.imu_log --depth --duration 10800
# detach with Ctrl+B then D; come back with: tmux attach -t imu
# stop early with Ctrl+C in the tmux window
```

Or with nohup, if tmux is not installed:

```bash
cd ~/vespr/src/perch_eval
mkdir -p ~/vespr_recordings/imu
nohup python3 -u -m perch_eval.imu_log --depth --duration 10800 \
    > ~/vespr_recordings/imu/nohup_$(date +%Y-%m-%dT%H-%M-%S).out 2>&1 &
# stop early with: pkill -INT -f perch_eval.imu_log   (never kill -9: that loses the tail)
```

Follow the run from another terminal with `tail -f ~/vespr_recordings/imu/*.status.log`.

The total time is the warm-up plus `--duration`: 3 h 20 min for the command above. Ctrl+C,
`kill -INT`, `kill` (SIGTERM) and the end of `--duration` all stop it the same way: the sensors
stop, the queue is written out, the files are closed and the final report is printed. Nothing
already received is lost. A second Ctrl+C kills the program at once, if stopping ever hangs.

If the camera only opens with `sudo` (no udev rules installed), `sudo python3 -m
perch_eval.imu_log ...` works: the default `--librealsense` and `--out` folders are still in
your home folder, and the files are handed back to you.

At 200 Hz the CSV grows by about 0.2 GB per hour.

### Options

| Option | Default | Meaning |
|---|---|---|
| `--librealsense PATH` | `~/librealsense/build/Release` | folder holding the pyrealsense2 module |
| `--rate HZ` | 200 | accel and gyro rate. The D435i offers accel at 100, 200, 400 and gyro at 200, 400 |
| `--duration S` | 0 | seconds to log after the warm-up; 0 logs until Ctrl+C |
| `--warmup S` | 1200 | seconds of streaming before logging starts |
| `--depth` | off | also stream depth and left IR at 848x480 for heat |
| `--depth-fps FPS` | 30 | depth and IR frame rate with `--depth` |
| `--out DIR` | `~/vespr_recordings/imu` | folder for the files |
| `--dry-run` | | `--warmup 0 --duration 60` |
| `--note TEXT` | none | free text written into the `.json` sidecar: how the camera was mounted and which way it faced |

The output folder is in the home folder for the same reason as perch_perception's clips: macOS
can sync Desktop and Documents to iCloud Drive, and these files are large.

## What the files contain

Each run writes three files to `--out`, named by the time the program started, for example
`imu_2026-10-20T22-15-00`:

| File | What it is |
|---|---|
| `.csv` | One row per motion frame, accel and gyro interleaved in arrival order. This is the data. |
| `.json` | Sidecar: camera serial, firmware, USB type, librealsense version and the file it was loaded from, the chosen streams and the motion intrinsics read from the camera, every sensor option's value at startup, the `global_time_enabled` values read back, the
`--note`, every command-line option, hostname, git commit (and whether the tree had uncommitted changes), the program start, logging start and end times, and the final report. It is written at startup and rewritten when logging starts and when the run ends. |
| `.status.log` | The startup lines, a warm-up line and a status line every minute, and the final report. |

CSV columns:

| Column | Meaning |
|---|---|
| `stream` | `accel` or `gyro` |
| `frame_number` | librealsense's frame counter for that stream. Gaps in it are the dropped frames. |
| `hw_timestamp_us` | the frame metadata `frame_timestamp`, in microseconds. Blank when the metadata is unsupported, which it is for motion frames on the Jetson's build. |
| `ts_ms` | `frame.get_timestamp()`, in milliseconds, in the domain given by the next column. With global time off this is the camera clock. Raw, including the startup clock jump. |
| `ts_domain` | `hardware_clock` expected; `global_time` or `system_time` means the clock was not the camera's |
| `host_ns` | `time.time_ns()` when the callback ran. This is the wall clock, so an NTP step during the run shows up here. |
| `x`, `y`, `z` | the reading in librealsense's units and axes: rad/s for gyro, m/s² for accel |

Accel and gyro are not merged, paired or interpolated. That is a decision for the analysis.

Each status line shows the time since logging started; for each stream, the sample count, the
rate over the last minute (1 / the median `ts_ms` interval), the dropped frames so far, and the
clock discontinuities so far with the CSV line, frame number and size of the last one; then the
queue depth and the temperatures (see [Temperatures](#temperatures)). A queue depth that keeps
growing means the disk cannot keep up.

The final report gives the duration and, for each stream, the samples, the rate and median
period, the dropped frames, the clock discontinuities and the `ts_domain`. A startup clock
jump gets a `note` line. Then a linear fit of `host_ns` against `ts_ms` (both in seconds) over
the longest segment, and which segment that was: the slope, minus 1, is the drift between the
host clock and the camera clock in ppm, and the residual is the rms jitter around that line in
microseconds. Then the file path and size, and warnings for anything that looks wrong.

The sidecar's `report.streams` has the same per stream: `rate_hz`, `measured_period_us`,
`dropped`, `clock_discontinuities` (each with `line`, `frame_number`, `step_ms`,
`frames_missing`, `seconds_into_log` and `startup`), `segments` and `drift_segment`.

## Test

Without ROS, from this folder. It needs `pytest`; one test also uses `numpy` and is skipped
without it. No camera or pyrealsense2 is needed.

```bash
python3 -m pytest
```

With ROS:

```bash
colcon build --symlink-install --packages-select perch_eval
colcon test --packages-select perch_eval
colcon test-result --verbose
```

`test/test_no_realsense_leak.py` checks, in a fresh interpreter, that importing `imu_log`
does not import pyrealsense2.

## What has been run on real hardware

One 60 s dry run on the Jetson with `--depth`: 12180 accel and 12011 gyro samples, 0 dropped,
`hw_timestamp_us` blank throughout, and `ts_domain` switching from `hardware_clock` to
`global_time` after about 13 samples. That run led to turning off global time.

A second 60 s dry run on 2026-10-20, with global time off: no gaps in `frame_number` on either
stream, and `ts_ms` in `hardware_clock` with the rates and startup jump described under
[Known behaviour of this unit](#known-behaviour-of-this-unit-d435i-serial-348522076012). The
asic and projector temperatures read `n/a`. That run led to the drop, discontinuity, rate and
temperature changes above, which have not yet been run on the camera.
