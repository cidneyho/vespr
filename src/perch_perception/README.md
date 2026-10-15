# perch_perception

Finds the perch branch in camera images and reports its location to flight control.

This is a skeleton. The node starts and spins but does not subscribe or publish yet, and branch
detection and localization are not written yet. What the code does today, module by module, is
written up in [`perch_perception/README.md`](perch_perception/README.md).

## Layout

```
perch_perception/
  perception_node.py    ROS 2 node: wiring only (subscribers, publishers, parameters)
  frame_source.py       FrameSource base class and the Frame it yields (no camera SDK imports)
  realsense_source.py   live D435i and .bag replay sources (the only pyrealsense2 import)
  run.py                reads a source end to end and prints a depth summary per frame
  process_source.py     runs a frame source in a child process, so a camera freeze cannot freeze the caller
  record.py             browser page with a live view and a Record / Stop button
launch/
  perception.launch.py  starts perception_node
resource/
  perch_perception      empty marker file that lets ROS find the package (colcon needs it)
test/                   pytest tests; synthetic_bag.py writes a small .bag without a camera
```

## Frame sources

`run.py` reads the live camera by default and a recording with `--bag`. Nothing else changes:

```bash
python -m perch_perception.run               # live D435i
python -m perch_perception.run --bag FILE    # replay a .bag
```

Every source yields the same tuple: image (left IR), depth in meters (0 means unknown),
intrinsics, timestamp in seconds, and an IMU sample or `None`. On USB 2 the live source runs
at 10 fps. `--imu` adds accel and gyro on Linux; it is refused on macOS, where librealsense
crashes when the motion streams start.

## Interfaces

Topics, message types and frames are defined in [`docs/interfaces.md`](../../docs/interfaces.md).
The rows that involve perception are still TBD there:

| Direction | Interface | Other side |
|---|---|---|
| in | Camera images | camera driver / simulation |
| in | Drone pose | PX4 onboard estimate |
| out | Branch location | flight control |

## Recording clips

Record on the Jetson, not on a Mac. On 2026-10-11 a Mac kernel-panicked twice while the
recorder was streaming from the D435i. Start the recorder on the Jetson and view its page on
your laptop through an SSH tunnel:

```bash
# on the Jetson
python -m perch_perception.record

# on the laptop, in a second terminal
ssh -N -L 8080:localhost:8080 perch@<jetson address>
```

Then open http://localhost:8080. The page shows the live left IR image and a Record / Stop
button. Each clip is saved to `~/vespr_recordings/` (change it with `--out`) as two files
named by start time, for example `2026-10-10T14-32-05`:

| File | What it is |
|---|---|
| `.bag` | Raw depth and left IR. This is the data, and `run.py --bag` replays it. |
| `.mp4` | A small grayscale video for watching the clip. Lossy: never use it for training or measurement. |

The default folder is in the home folder on purpose: macOS can sync Desktop and Documents to
iCloud Drive, and clips are large.

The camera restarts when recording starts and stops, and each camera session runs in its own
process, so expect a pause of about five seconds each time. A `.bag` is uncompressed: about 12 MB per second at 848x480 and 10 fps.

## Build and run

From the workspace root (`~/vespr` in the dev container):

```bash
colcon build --symlink-install --packages-select perch_perception
source install/setup.bash

ros2 run perch_perception perception_node
# or
ros2 launch perch_perception perception.launch.py
```

## Test

In the dev container or on the Jetson:

```bash
colcon test --packages-select perch_perception
colcon test-result --verbose
```

Without ROS, for example on a Mac, run pytest from this folder. It needs Python 3.11 with
`numpy`, `opencv-python`, `pytest` and `pyrealsense2-macosx==2.54.2`. Tests that need
`pyrealsense2` or `cv2` are skipped where they are missing.

```bash
python -m pytest
```
