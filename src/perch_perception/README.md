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
  clip_storage.py       upload, download and clean up clips on Google Drive (rclone)
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

## Google Drive storage

Clips are kept in the `VESPR Recordings` folder of John's Google Drive, split into `Indoors` (the
indoor rig) and `Outdoors` (outdoor handheld), because the two are evaluated separately. rclone
does the copying, and `clip_storage` decides what to copy. Every command needs `--domain indoor`
or `--domain outdoor`, which picks both folders:

| `--domain` | Local folder | Drive folder |
|---|---|---|
| `indoor` | `~/vespr_recordings/indoor` | `VESPR Recordings/Indoors` |
| `outdoor` | `~/vespr_recordings/outdoor` | `VESPR Recordings/Outdoors` |

```bash
python -m perch_perception.clip_storage upload --domain indoor              # send finished clips not on the Drive yet
python -m perch_perception.clip_storage download --domain indoor NAME ...   # fetch named clips
python -m perch_perception.clip_storage download --domain indoor --missing  # fetch every clip not here yet
python -m perch_perception.clip_storage cleanup --domain indoor             # list local clips safe to delete
python -m perch_perception.clip_storage cleanup --domain indoor --yes       # delete them
```

Record straight into the matching folder with the recorder's `--out`, for example
`python -m perch_perception.record --out ~/vespr_recordings/indoor`. Clips in the recorder's
default `~/vespr_recordings` are in neither domain, so move them into one before uploading.

Run `upload` on the Jetson after a recording session, when it is online. Re-running it is
always safe: it only sends what is still missing. A session's clips can take a long time over
Wi-Fi, so when you start it over ssh, run it inside `tmux` (`sudo apt install tmux` if it is
missing): start `tmux`, then the command, and after a dropped connection ssh back in and run
`tmux attach`. Otherwise the upload stops when the ssh connection drops. A clip changed in the last minute is skipped
as still being written. Nothing on the Drive is ever overwritten or deleted. `cleanup` deletes a
local clip only when every one of its files is on the Drive with a matching checksum. `--folder`
and `--remote` change the local folder and the Drive location.

Teammates don't need any of this: the folder is shared with them, and they download clips in the
browser.

### One-time setup

rclone logs in to Google through a Google "OAuth client". rclone's shared one is being retired,
so the project uses its own, made once in Google Cloud by following
[rclone's instructions](https://rclone.org/drive/#making-your-own-client-id). Make it a
**Desktop app** with audience **External**. Its client ID and secret are kept out of the repo. Ask
John for them.

The app is in Google's testing mode, so every login expires after 7 days. When a command says
the login may have expired, run `rclone config reconnect vespr-drive:`. On the Jetson, which
has no browser, answer `n` to the web browser question and paste a token from
`rclone authorize`, as in step 2 below. Publishing the app would
end that, but Google requires a home page and a privacy policy link for it first.

On each machine that uploads or downloads:

1. Install rclone. On a Mac, `brew install rclone`. On the Jetson,
   `sudo -v ; curl https://rclone.org/install.sh | sudo bash`, which installs a current ARM64
   build (Ubuntu's `apt` package is older).
2. Run `rclone config` and create a remote named `vespr-drive`: storage type `drive`, the client
   ID and secret, scope `1` (full access), and `n` to "Configure this as a Shared Drive". Log in
   with the Google account that owns the `VESPR Recordings` folder, and continue past the
   "unverified app" warning.
   - The Jetson has no browser. Answer `n` to auto config, run
     `rclone authorize "drive" "<client id>" "<client secret>"` on a laptop, and paste the token
     it prints into the Jetson.
3. Check it with `rclone lsd vespr-drive:`, which lists the account's Drive folders.

The login token stays in rclone's config file on that machine (`~/.config/rclone/rclone.conf`),
never in the repo.

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
