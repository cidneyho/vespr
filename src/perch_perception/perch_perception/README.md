# What the perception code does today

This page describes the code in this folder as it is now, module by module. It covers what each
part does, what it guarantees, and what it does not do yet. For how to build, run and test the
package, see the [package README](../README.md).

In short: the package can stream frames from the D435i, replay recorded `.bag` files, and
record clips from a browser page. Nothing finds a branch yet. There is no detection, no pose
estimate and no diameter estimate, and the ROS node publishes nothing.

## Contents

- [How the pieces fit together](#how-the-pieces-fit-together)
- [The frame every source yields](#the-frame-every-source-yields-frame_sourcepy)
- [Camera and bag sources](#camera-and-bag-sources-realsense_sourcepy)
- [Running a source in a child process](#running-a-source-in-a-child-process-process_sourcepy)
- [The clip recorder](#the-clip-recorder-data_collectionrecordpy)
- [Keeping clips on Google Drive](#keeping-clips-on-google-drive-data_collectionclip_storagepy)
- [Keeping part of a clip](#keeping-part-of-a-clip-data_collectiontrimpy)
- [Reading a source from the command line](#reading-a-source-from-the-command-line-runpy)
- [The ROS node](#the-ros-node-perception_nodepy)
- [The branch distance script](#the-branch-distance-script-branch_distancepy)
- [Tests](#tests)
- [What has been run on real hardware](#what-has-been-run-on-real-hardware)
- [Not implemented yet](#not-implemented-yet)

## How the pieces fit together

```
                     D435i over USB                       recorded .bag file
                           |                                      |
                       LiveSource                             BagSource
                           \______ realsense_source.py ______/
                                (the only pyrealsense2 import)
                                           |
                               Frame (NumPy arrays and dataclasses)
                                           |
              +----------------------------+-----------------------------+
              |                            |                             |
           run.py                    ProcessSource                (future detection
     prints a depth summary     runs a source in a child          and pose modules)
                                process, frames over a pipe
                                           |
                               data_collection/record.py
                         browser page, .bag + .mp4 per clip
```

The design rule is that camera SDK types stop at `realsense_source.py`. Everything after it gets
plain NumPy arrays and the dataclasses from `frame_source.py`. So the same code runs on a live
camera, on a recording, or on in-memory test frames, and it runs on a machine without the SDK.
`test/test_no_realsense_leak.py` enforces this.

Conventions used everywhere:

- Units are meters and seconds.
- The camera frame is OpenCV's: X right, Y down, Z forward. The origin is at the optical center
  of the left infrared imager.
- A depth of 0 means "unknown", never 0 m.

## The frame every source yields (`frame_source.py`)

Plain Python and NumPy. This module defines the contract between sources and everything that
consumes them.

### `Frame`

A `NamedTuple`, so it unpacks as `image, depth, intrinsics, timestamp, imu = frame` and also
reads by name (`frame.depth`).

| Field | Type | Meaning |
|---|---|---|
| `image` | `H x W` `uint8` array | Left infrared image (grayscale). The IR emitter is on, so its dot pattern is visible. |
| `depth` | `H x W` `float32` array | Depth in meters, pixel for pixel aligned with `image`. 0 means no reading. |
| `intrinsics` | `Intrinsics` | Camera parameters of the image stream. Depth shares them. |
| `timestamp` | `float` | Seconds. Taken from the depth frame's librealsense timestamp. |
| `imu` | `ImuSample` or `None` | `None` when the source has no IMU. Never an error. |

Image and depth line up with no alignment step because the D435i computes depth in the left
imager's own frame. The pixel at `(row, col)` in `image` is the same scene point as the pixel at
`(row, col)` in `depth`.

### `Intrinsics`

A frozen dataclass of pinhole camera parameters in pixels: `width`, `height`, `fx`, `fy`
(focal lengths), `cx`, `cy` (principal point, where the optical axis meets the image),
`distortion_model` (a name such as `'brown_conrady'`) and `distortion_coeffs` (a tuple).

### `ImuSample`

A frozen dataclass: `accel` (x, y, z in m/s²), `gyro` (x, y, z in rad/s) and `timestamp`
(seconds).

### `depth_to_meters(raw_depth, depth_scale)`

Converts the camera's raw 16-bit depth to `float32` meters by multiplying by `depth_scale`
(0.001 on a D435i). Raw 0 stays 0, so callers still filter zeros before any statistic.

### `FrameSource`

The abstract base class for anything that produces frames. A subclass implements `frames()`, a
generator of `Frame`s, and may override `close()`. Every source can be:

- iterated directly: `for frame in source:`
- used in a `with` block, which calls `close()` however the block ends.

`close()` is safe to call more than once.

## Camera and bag sources (`realsense_source.py`)

The only module in the package that imports `pyrealsense2`. Both sources share one frame loop
and convert every RealSense frame to a `Frame` before yielding it.

### `LiveSource(width=848, height=480, fps=None, imu=False, record_to=None)`

Streams from the first connected D435i.

- **Streams.** It turns on depth (`z16`) and the left infrared imager (`y8`) at the requested
  size, plus accelerometer and gyroscope when `imu=True`.
- **Frame rate.** With `fps=None` it reads the camera's USB type descriptor (for example `'3.2'`
  or `'2.1'`) and picks 30 fps on USB 3 and 10 fps otherwise. 10 fps is the most the camera
  delivers at 848x480 over USB 2. An unknown USB type counts as USB 2.
- **IMU.** `imu=True` is refused on macOS with a `RuntimeError`, because librealsense crashes
  the whole process there when the motion streams start, and Python cannot catch that. On
  other platforms the IMU is only turned on if the camera offers motion streams. Otherwise it
  carries on without IMU. The IMU streams are requested without a format, because asking for
  `motion_xyz32f` failed on the Jetson's build.
- **Recording.** With `record_to` set to a path, librealsense's own recorder also writes every
  enabled stream to that `.bag` file, raw and with its timestamps. The file is only finished
  when the source is closed.
- **Errors.** It raises `RuntimeError` if no camera is connected, or if no frame arrives for
  5 seconds while streaming.
- **Attributes** after opening: `usb_type` (the descriptor string, `''` if unknown), `fps` (the
  rate in use) and `imu_enabled`.

### `BagSource(path)`

Replays a `.bag` recording. It needs no camera.

- It raises `FileNotFoundError` naming the file if the path does not exist.
- Playback is not real-time. Every recorded frame is delivered as fast as the caller reads,
  instead of frames being dropped when the caller falls behind.
- Playback does not loop. The source ends when librealsense reports that playback has stopped
  and no frame has arrived for 1 second.
- Intrinsics come from the recording, so a bag replays with the calibration it was recorded
  with.
- Each depth image comes paired with the left IR image of the same exposure. On real clips this
  was checked with librealsense 2.54.2 (Mac) and 2.58.4 (Jetson): every pair has identical
  timestamps. A clip recorded on the Jetson replays on the Mac.

### The shared frame loop

For every set of frames librealsense delivers, the loop:

1. Skips sets that do not carry both a depth frame and a left IR frame, for example a set that
   only holds an IMU reading.
2. Skips a depth frame it has already yielded. librealsense hands the same depth frame out again
   when a new IMU reading arrives between two camera frames.
3. Copies the IR image and converts depth to meters. Both produce new arrays, so librealsense's
   own buffers go straight back to its small pool. Without the copy, a caller that keeps frames
   (a list, for example) makes librealsense silently drop new ones. This was seen as 8 of 40
   frames lost.
4. Converts the timestamp from milliseconds to seconds.
5. Attaches an IMU sample if the set carries both an accel and a gyro reading. It is the latest
   reading librealsense attached, not interpolated to the image time. Its timestamp is the gyro
   reading's. On the Jetson this reading was about 85 ms newer than the image, so it arrives
   once per frame, at 10 Hz, not at the IMU's native 100 Hz and 200 Hz.

### Closing

`close()` stops the pipeline and then drops the references to the pipeline and its profile.
Dropping them is what makes librealsense finish a `.bag` it was recording. If the pipeline is
only stopped, the file is left "unindexed" and cannot be replayed.

### One librealsense context per process

`_shared_context()` creates one `rs.context()`, librealsense's handle on the USB system, on first
use and never destroys it. `LiveSource` uses it to find the camera. On macOS the sequence that
tested reliable kept one context alive for the life of the process.

## Running a source in a child process (`process_source.py`)

On macOS, librealsense can freeze while it destroys camera objects. A freeze inside the library
stops every thread of its process, so a recorder serving a web page would stop answering too.
`ProcessSource` keeps the camera in a separate, short-lived process so that a freeze there
cannot freeze the caller.

`ProcessSource(open_source, args=(), open_timeout=40, frame_timeout=10, close_timeout=15)` is
itself a `FrameSource`.

- **Starting.** It starts a child process with the `'spawn'` method, a fresh Python interpreter.
  `'fork'` would copy the parent, threads included, which is unsafe for a program running a web
  server. The child calls `open_source(*args)`. `open_source` must be a module-level function,
  because the child finds it by importing its module.
- **Opening.** The constructor returns once the child reports that the source opened. It raises
  `RuntimeError` if the child reports an error, if the child dies, or if nothing is heard within
  `open_timeout` seconds. In every one of those cases, and on Ctrl+C while it waits, it ends the
  child first, so a child stuck opening the camera cannot keep hold of it, or create a `.bag`
  after the recorder has decided there is none.
- **Frames.** The child sends every frame back over a one-way pipe. Each `Frame` is pickled, so
  the parent gets its own copy. `frames()` raises `RuntimeError` if no frame arrives within
  `frame_timeout` seconds, or if the child dies (`'the camera process ended unexpectedly'`). It
  ends normally when the inner source runs out of frames.
- **`fps`.** The inner source's `fps` attribute is passed back and exposed as `source.fps`.
- **Ctrl+C.** A terminal sends Ctrl+C to the child as well as the parent. The child ignores it
  and waits to be told to stop, so a recording in the child is always closed properly.
- **Closing.** `close()` sets a stop flag and keeps reading the pipe, because the child may be
  blocked part-way through sending a frame. The child closes its source, sends `'closed'` and
  exits with `os._exit(0)`. That skips Python's normal shutdown cleanup, which is where
  librealsense froze. If the child is still alive after `close_timeout` seconds, it is killed and
  `close()` raises `RuntimeError` saying the camera process froze and the clip may be
  unfinished.

Messages on the pipe are `(kind, payload)` pairs: `('opened', fps)`, `('frame', Frame)`,
`('error', message)` and `('closed', None)`.

## The clip recorder (`data_collection/record.py`)

```bash
python -m perch_perception.data_collection.record --domain indoor|outdoor [--out FOLDER] [--port 8080] [--width 848] [--height 480] [--fps N]
```

A camera loop plus a small web server. Open `http://localhost:8080` to get a live view of the
left IR image, a Record / Stop button, fields for the clip's site, branch, light and notes, and a
button to mark the last clip bad. Run it on the Jetson and reach the page through an SSH
tunnel (see "Recording clips" in the package README). Do not run it on John's Mac, which
kernel-panicked twice while it streamed. At startup it prints the computer's clock, which goes
into the clip names.

### Sessions

librealsense can only record from the moment a pipeline starts. So the recorder runs the camera
in **sessions**, one mode per session, and restarts the camera whenever the mode changes:

- A **preview session** streams without recording and only feeds the live view.
- A **recording session** records one clip.

Every session opens the camera through `ProcessSource`, so each session is its own process.
Expect a pause of about five seconds at each start and stop while the camera restarts. After a
clip ends, for whatever reason, the recorder goes back to preview.

### What one clip produces

Files are named by the clip's number and start time, for example `0042_2026-10-23T14-02-11`.
The number is padded to 4 digits so clips sort by it, up to clip 9999, and colons are left out of
the time so the name is valid on every file system. If the folder already has a clip by that name,
`free_clip_name` adds `-2`, `-3` and so on, so an earlier `.bag` is never written over.

**Clip numbers** come from one counter file, `~/vespr_recordings/next_clip_id`, next to the two
domain folders, so indoor and outdoor clips share one count. The file holds the next number;
`take_clip_number` reads it and writes the one after it, flushed to the disk before it is swapped
in, so a pulled battery never leaves it half written. The number is taken when Record is
pressed, so a recording that fails to start leaves a gap, and a number is never handed out
twice, even after `clip_storage cleanup` deletes its clip. That is why the count is kept in a
file and not worked out from the folder. Without the file the count starts at 1. If the Jetson
is reinstalled, write a number higher than every clip on the Drive into the file before
recording. If you forget, names can repeat ones already on the Drive (an offline Jetson's clock
restarts at the same date), and `clip_storage upload` reports those clips as conflicts and copies
none of their files. A file holding
anything but a whole number from 1 stops each recording with an error on the page, and changes
nothing.

| File | Contents |
|---|---|
| `<name>.bag` | Raw 16-bit depth and the left IR image at the session's frame rate, written by librealsense, with per-frame timestamps and the camera's calibration. This is the data. About 12 MB per second at 848x480 and 10 fps. |
| `<name>.mp4` | Every recorded IR frame as grayscale video, for watching only. It is H.264 (`avc1`), or MPEG-4 (`mp4v`) where OpenCV cannot encode H.264. It is lossy: never use it for training or measurement. |
| `<name>.json` | The clip's details, below. Written before the camera starts, marked bad, and written again when the clip ends. |

The details file:

| Key | Value |
|---|---|
| `clip`, `number` | The clip's name and number |
| `domain` | `indoor` or `outdoor`, from `--domain` |
| `started` | The start time from the computer's clock, for example `2026-10-23T14:02:11`. Wrong if the Jetson's clock was not set. |
| `seconds` | How long the clip is, from its first frame to its last. `null` until the clip ends, and so it stays `null` only when the recorder never got to the end, as after a pulled battery or a killed recorder. |
| `site`, `branch`, `light`, `notes` | What was in the page's fields when Record was pressed. Spaces at the ends are trimmed. |
| `bad` | `true` if the clip was marked bad |
| `error` | Why the clip was marked bad by itself: the camera's error, or `the recording did not finish`. Kept if the clip is unmarked later. |
| `keep` | Only present once `trim` has set it: `[start, end]`, the seconds of the clip to extract (see Keeping part of a clip) |

The fields carry over from clip to clip, so they are typed once per branch. `extract_frames`
uses `site` and `branch` to keep every clip of a branch in one split (see the training README),
so type a new branch name for each branch.

**Marked bad until it ends.** The `.json` is first written with `bad` set to `true` and `error`
set to `the recording did not finish`, before the camera starts. Only when the clip ends normally
is it written again, as not bad. So a clip cut short by a pulled battery or a killed recorder,
which may not replay to the end, is never taken for a good one. If the camera fails during the
clip, it stays bad with the camera's error. If that last write fails (a full disk), the page says
so and shows the clip as bad, which is what the file says; unmarking it then saves the clip's
full details. If the camera fails to start and leaves no `.bag`, the `.json` is deleted, because a
`.json` alone is no clip; if it leaves a `.bag`, the `.json` stays bad, with the camera's error.
Any other way out of a clip, such as a bug in the recorder, leaves it bad too.

Ctrl+C during a clip ends it as Stop would, saves it as good once the camera has finished the
`.bag`, and then quits. If the camera then fails to close, the clip stays bad instead: an error
can mark a good clip bad, never a bad clip good. A second Ctrl+C while the camera is closing (which can take seconds)
leaves the clip bad, because the `.bag` may be unfinished. A power cut during the few seconds the
camera takes to start leaves a `.json` with no `.bag`. `clip_storage upload` uploads it like any
clip, and `extract_frames` never sees it, since it only reads `.bag` files: it is clutter, and
can be deleted.

**Marking a clip bad.** Between clips, the page shows "Mark #0042 bad" for the last clip. It sets
`bad` to `true` in the clip's `.json` and deletes nothing, so a clip marked by mistake is not
lost: press the button again ("Unmark #0042 bad"), or edit the `.json` later. `extract_frames`
skips clips marked bad. Only the last clip of the running recorder can be marked from the page,
and only between clips; mark older clips by editing their `.json`. A clip whose `.bag` has gone
from the Jetson, deleted by `clip_storage cleanup`, cannot be marked there: edit its `.json` on
the Mac after downloading it, before running `extract_frames`. Mark a clip before it is
uploaded: marking changes the `.json`'s size, so for a clip already on the Drive, `clip_storage
upload` reports a conflict and leaves the Drive's old copy; replace that one by hand.

The recorder does **not** record the IMU, save camera details (emitter state, exposure) in the
`.json`, or refuse to record over USB 2.

### Where clips go

`--domain` (required) picks `~/vespr_recordings/indoor` or `~/vespr_recordings/outdoor`, the
folders `clip_storage` uploads from; `--out` overrides it. They sit directly in the home folder,
because macOS can sync Desktop and Documents to iCloud Drive and clips are hundreds of megabytes.
Under `sudo` the folder is still the home folder of the person who ran `sudo` (from `SUDO_USER`).
The folder is created if it is missing. Under `sudo` every folder the recorder creates, the
counter and every clip file are handed back to that person (from `SUDO_UID` and `SUDO_GID`), so
they don't end up owned by root.

### Recorder states

What the page shows comes from `Recorder.status()`:

| State | Meaning | Button |
|---|---|---|
| `preview` | Camera streaming, not recording | Record |
| `starting` | Record pressed, camera restarting to record | disabled |
| `recording` | Clip being recorded. The page shows its number and running time, and the image gets a red border. | Stop |
| `stopping` | Stop pressed, clip being finished | disabled |

If the camera fails to open, or fails part-way through, the error is shown on the page, any
recording request is cancelled, and the loop tries again every second. A repeating error is
printed to the terminal only once.

### Web server

It listens on `127.0.0.1` only, so the page is not reachable from the network except through a
tunnel.

| Request | Response |
|---|---|
| `GET /` | The page |
| `GET /status` | JSON: `state`, `clip`, `seconds`, `last_clip`, `last_bad`, `details` (the fields kept for the next clip), `error`, `folder` |
| `GET /stream` | The live view as MJPEG: one HTTP response that never ends, carrying one JPEG after another |
| `POST /start` | Ask for a recording; replies with the status. A JSON body of fields, such as `{"site": "park", "branch": "7"}`, replaces the kept fields (a missing field becomes empty); no body keeps them. An unknown field, or one that is not text, gets `400` and starts nothing. |
| `POST /stop` | Ask for the recording to end; replies with the status. Any body is ignored, so stopping always works. |
| `POST /bad` | `{"bad": true}` or `{"bad": false}`: mark the last clip bad or not; replies with the status. `400` for any other body; `409` before any clip, while a clip is recording, or when the last clip's `.bag` is gone; `500` if the `.json` cannot be written. |

Refusals reply with `{"error": "..."}`, and the page shows the message under the buttons, or
"The recorder did not answer" when no reply comes.

The page asks for `/status` twice a second and says "The recorder is not responding" if it gets
no answer.

### Quitting

Ctrl+C in the terminal ends the current session. A recording in progress is finished and closed
properly. Then the web server shuts down.

## Keeping clips on Google Drive (`data_collection/clip_storage.py`)

```bash
python -m perch_perception.data_collection.clip_storage upload | download NAME... | download --missing | cleanup [--yes]
    --domain indoor|outdoor [--folder FOLDER] [--remote REMOTE]
```

Copies clips between a local folder and a folder in John's Google Drive, using rclone. The
required `--domain` picks both: `indoor` means `~/vespr_recordings/indoor` and
`vespr-drive:VESPR Recordings/Indoors`; `outdoor` means `.../outdoor` and `.../Outdoors`. Setup is
in the [package README](../README.md#google-drive-storage).

- **Clips.** A clip is every file directly in the folder that shares a name and ends in `.bag`,
  `.mp4` or `.json`. Hidden files and other extensions are ignored.
- **Still being written.** A clip with any file changed in the last 60 seconds is skipped by
  `upload` and `cleanup`.
- **`upload`** lists the Drive folder once (names, sizes, MD5 checksums), then copies each local
  file that is missing there with `rclone copyto --ignore-existing`. rclone checks each copy's
  checksum. A file already there with the same size is skipped. A clip that has a file there
  with the same name but a different size is a conflict, and none of it is uploaded.
- **`download`** works the same way in the other direction. A file name such as `X.bag` is taken
  to mean clip `X`. `--missing` fetches every clip file that is not local.
- **`cleanup`** computes each local file's MD5 in one-megabyte blocks and compares it with the
  Drive's. Only clips whose every file matches are listed as safe, and deleted with `--yes`.
- **Output.** `upload` and `download` print `uploading <clip> (...)...` before each clip and
  `uploaded <clip> (...)` after it, one line at a time even when run over ssh. Any command refuses
  to run when `--remote` is the same local folder as `--folder`.
- **Exit codes.** 0 when everything that should have moved moved; 1 when a clip failed,
  conflicted or was not found, when a deletion failed, or when rclone is not set up.
- **Structure.** Plain-Python functions decide what to do. The `Rclone` class is the only code
  that runs rclone, and the commands take it as an argument, so tests use an in-memory stand-in.

## Keeping part of a clip (`data_collection/trim.py`)

```bash
python -m perch_perception.data_collection.trim 42 --keep 0 12.5 --domain outdoor
python -m perch_perception.data_collection.trim 42 --all --domain outdoor
```

For a clip that is good apart from its start or end, such as one stopped too late. Run it on the
Mac after downloading the clip, and before `extract_frames`. Find the times by playing the clip's
`.mp4`.

- **The clip** is named by its number (`42` or `0042`) or by its whole name, which old clips
  named only by their start time need. Two clips with the same number, as after the counter file
  was lost, are refused with both names, so give the whole name.
- **`--keep START END`** writes `"keep": [START, END]` into the clip's `.json` and sets `bad` to
  `false`, because a trimmed clip is one worth using. It refuses a range that does not start at
  0 or later and end after it starts, and one that starts after the clip's `seconds`. An end past
  the end of the clip keeps the rest of it.
- **`--all`** removes the range, so the whole clip is kept again. It leaves `bad` as it is.
- **`--domain`** picks `~/vespr_recordings/<domain>`, as `clip_storage` does; `--folder` picks
  another folder. A clip with no `.json`, recorded before the recorder wrote one, gets one.

Nothing is cut out of the `.bag`, which stays exactly as it was recorded. `extract_frames` takes
only the frames in the range, ends included, timed from the clip's first frame, and stops reading
after its end.

**Trimming cannot save a truncated `.bag`.** librealsense refuses to open a `.bag` cut short,
whatever range is kept (tried on synthetic bags cut to 50 to 99% of their size, with the Mac's
librealsense 2.54.2). A clip whose `.json` still says `the recording did not finish`, as after a
pulled battery or a killed recorder, may be one, so `trim` warns: if `extract_frames` then fails
on it, mark it bad again. Trimming is for a good recording with a bad start or end, such as one
stopped too late.

The times are close, not exact: the `.mp4` holds only the frames the recorder received, played at
a fixed rate, while the range is matched to the camera's timestamps. If frames were dropped
while recording, `.mp4` times run behind, more so the later in the clip. Leave half a second of
margin. `trim` warns when the `.mp4` plays more than half a second shorter than the clip's
`seconds`: then leave a wider margin late in the clip.

Set the range before the clip is first extracted: the `.json` is only read then, and changing the
range later only brings a warning from `extract_frames`. The range lives only in the Mac's copy
of the `.json`; the Drive's copy and the Jetson's keep the untrimmed one, which is why the clip
should also be marked bad in the field: a fresh download then skips it, not extracts it whole.
Do not `upload` from the Mac afterwards: it reports a conflict for a trimmed clip's `.json`.
Marking the last clip bad or not on the record page keeps a range already saved in its `.json`.

## Reading a source from the command line (`run.py`)

```bash
python -m perch_perception.run               # live D435i
python -m perch_perception.run --bag FILE    # replay a recording
```

Only `--bag` changes between the two. Other flags: `--width`, `--height`, `--fps` and `--imu`
(live only; `--imu` is refused on macOS), `--max-frames N` (stop after N frames) and
`--print-every N` (default 10).

What it prints:

- For a live camera, a first line with the USB type, resolution, frame rate and whether IMU is
  on. That is followed by a warning when the link is not USB 3, and a note when `--imu` was asked
  for but the camera has no IMU streams.
- For every Nth frame, a line such as
  `t=12.345 s  848x480  median depth 1.523 m  valid 87%  imu no`. The median is taken over valid
  pixels only, and "valid" is the share of pixels that have depth.
- At the end, `read N frames in T s`.

It returns exit code 1 with a message if `pyrealsense2` is missing or the source cannot be
opened. Ctrl+C stops it cleanly, and the camera or bag is always closed on the way out.
`pyrealsense2` is imported only when a source is opened, so the summary functions work and are
tested without it.

Unlike the recorder, `run.py` opens the live camera in its own process, not through
`ProcessSource`.

## The ROS node (`perception_node.py`)

ROS wiring only, and the only module that imports `rclpy`. It starts, logs
`perception_node started`, and spins until Ctrl+C or shutdown. It has no subscribers, publishers
or parameters yet. Those wait on the topics, message types and frames marked TBD in
[`docs/system-map/`](../../../docs/system-map/README.md).

It is installed as the `perception_node` executable (`ros2 run perch_perception
perception_node`) and started by `launch/perception.launch.py`.

## The branch distance script (`branch_distance.py`)

A standalone tool, not part of the pipeline, and not installed as an executable. It opens the
camera directly at 848x480 and 30 fps. That needs USB 3, so it does not start over USB 2. It
shows the IR image in an OpenCV window. Click two points along a branch and it samples depth in a
5-pixel-wide strip between them, drops zeros, and shows the median distance and the share of
valid pixels. `c` clears the clicks and `q` quits. Keep the camera still: the clicked line is
fixed in the image.

Unlike the rest of the package, it imports `pyrealsense2` directly and has no tests.

## Tests

213 tests in the files below, run with `python -m pytest` from the package folder. The training
pipeline's tests, `test_dataset.py`, `test_extract_frames.py` and the others named after its
steps, are not listed here. All pass on the Mac and on the
Jetson. None of them needs a camera
or a network.

| File | Tests | What it checks |
|---|---|---|
| `test_frame_source.py` | 6 | Depth scaling, zero kept as unknown, `float32`, the `Frame` unpacks in the agreed order, iteration and `with` blocks |
| `test_realsense_source.py` | 12 | `BagSource` end to end on a synthetic bag: every frame once (the buffer copy), depth in meters, zeros kept, the image is left IR, timestamps in seconds, recorded intrinsics, no IMU, missing file. Also the USB frame rate choice and the macOS IMU refusal. |
| `test_process_source.py` | 11 | Frames and images cross the process boundary, `fps` is passed back, open errors, open timeouts and a Ctrl+C while opening end the child, the source is closed before the child exits, Ctrl+C does not kill the child early, a frozen child is killed and reported, a dying child is reported |
| `test_record.py` | 72 | Clip numbers and names, the shared counter (never reused, a counter file that is not a number from 1 refused), a taken clip name getting `-2`, `--domain` and the folders, giving files, folders and the counter back under `sudo`, the details file and its fields, the details marked bad until the clip ends (and when the last write fails, after an unexpected error, or after a Ctrl+C while the camera closes), Ctrl+C mid-clip saving a good clip, a failed start that leaves a `.bag`, `seconds` ending at the last frame (after a camera error too), marking bad (by hand, after a camera error, refused while recording or after cleanup), preview and recording sessions, the preview video's frame count, status states, error handling, the camera loop following the button, and every web endpoint |
| `test_trim.py` | 26 | Finding a clip by its number or name (with or without `.bag`), a number two clips share, keeping a range and clearing `bad`, refused ranges (NaN and infinity too), `-0` saved as `0`, an end past the clip's end, `--all`, a clip with no `.json`, a `.json` that is not JSON, the warnings for a clip that did not finish and an `.mp4` shorter than the clip, the command line, and that `extract_frames` reads what it writes |
| `test_run.py` | 9 | The depth summary ignores zeros, the summary line, frame counting and `--max-frames`, and replaying a synthetic bag through `main()` |
| `test_clip_storage.py` | 60 | Clip grouping, still-being-written, copy and delete decisions, the rclone listing format, `upload`, `download`, `cleanup`, the command line, an unreadable clip, Ctrl+C, partial downloads and a login that expires mid-run, all with an in-memory rclone |
| `test_clip_storage_rclone.py` | 4 | `upload`, `download` and `cleanup` with the real rclone between two local folders (skipped where rclone is missing) |
| `test_no_realsense_leak.py` | 13 | `data_collection.clip_storage`, `data_collection.record`, `data_collection.trim`, `frame_source`, `process_source`, `run` and the seven `training` modules import without pulling in `pyrealsense2` (checked in a fresh interpreter) |

Helpers:

- `test/fake_source.py` holds in-memory sources, including ones that are slow to open, freeze on
  close or die part-way, for the process tests.
- `test/fake_rclone.py` is an in-memory stand-in for rclone and the Drive, for the
  `clip_storage` tests.
- `test/synthetic_bag.py` writes a small `.bag` with known contents using librealsense's software
  device, so bag replay is tested on the real playback path without a camera. It writes each IR
  frame before its depth frame: librealsense 2.58 pairs a depth frame with the newest IR frame
  already written, so the other order pairs depth `i` with IR `i - 1`. Bags from a real camera
  pair correctly either way.

Tests that need `pyrealsense2` or `cv2` are skipped where those are missing. In the dev
container, where `pyrealsense2` is not installed, the bag tests do not run.

## What has been run on real hardware

| Part | Real camera or real data | Result |
|---|---|---|
| `LiveSource` through `run.py` | Jetson Orin Nano, D435i on USB 2, 2026-10-10 | 30 frames in 5.0 s at 10 fps, chosen from the USB link. With `--imu`, every frame carried an IMU sample. |
| `BagSource` | Synthetic bags in the tests. Real clips from the Mac (2026-10-11) and the Jetson (2026-10-19). | Replays on both. Depth and IR of each frame have identical timestamps. |
| Recorder, first version (camera opened in-process) | John's Mac, 2026-10-11 | Recorded one good clip, then froze while closing the camera |
| Recorder, current version (through `ProcessSource`) | Jetson, D435i on USB 2, controlled from the Mac over the USB-C cable, 2026-10-19 | Two indoor clips at 848x480 and 10 fps (106 and 49 frames). No freezes. |
| `clip_storage` | Jetson to Google Drive and Drive to the Mac, 2026-10-19 | Uploaded 198 MB in 49 s with matching checksums; a second `upload` sent nothing; the downloaded clip replayed on the Mac. `cleanup` not yet run on real clips. |

The macOS findings that shaped `ProcessSource` and `close()` came from `pyrealsense2-macosx`
2.54.2 on John's Mac. On the Jetson (2.58.4) the recorder opened and closed the camera through
`ProcessSource` for every preview and recording session without freezing, but whether an
in-process close would also freeze there has not been tested.

## Not implemented yet

- Branch detection (classical line-segment baseline, then a learned detector)
- Mask to 3D branch axis, diameter, confidence
- Publishing anything from the ROS node. NAMING.md proposes `branch_pose` as a
  `geometry_msgs/msg/PoseStamped`, not a custom message type
- A video-file-plus-intrinsics-JSON source
- Camera details in the recorder's `.json` (emitter state, exposure, git commit), its USB 2
  refusal, and IMU recording
- Depth preset selection (the camera runs its default preset)
- A comparison clip at 1280x720 on USB 3, against today's 848x480
- IMU to image timestamp alignment
