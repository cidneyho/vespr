# perch_perception

Finds the perch branch in camera images and reports its location to flight control.

This is a skeleton. The node starts and spins but does not subscribe or publish yet, and branch
detection and localization are not written yet. Data collection works end to end: on 2026-10-19
clips were recorded on the Jetson from a Mac, uploaded to Google Drive, downloaded to the Mac and
replayed there. What the code does today, module by module, is
written up in [`perch_perception/README.md`](perch_perception/README.md).

## Layout

```
perch_perception/
  perception_node.py    ROS 2 node: wiring only (subscribers, publishers, parameters)
  frame_source.py       FrameSource base class and the Frame it yields (no camera SDK imports)
  realsense_source.py   live D435i and .bag replay sources (the only pyrealsense2 import)
  run.py                reads a source end to end and prints a depth summary per frame
  process_source.py     runs a frame source in a child process, so a camera freeze cannot freeze the caller
  data_collection/      tools for collecting training data, kept apart from the perception code
    record.py           browser page with a live view and a Record / Stop button
    clip_storage.py     upload, download and clean up clips on Google Drive (rclone)
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
intrinsics, timestamp in seconds, and an IMU sample or `None`. The live source runs at 848x480,
at 30 fps on USB 3 and 10 fps on USB 2. 848x480 stays the resolution for now; once the camera is
on USB 3, a clip at 1280x720 (`--width 1280 --height 720`) is worth comparing for spotting the
branch from farther away. `--imu` adds accel and gyro on Linux; it is refused on macOS, where
librealsense crashes when the motion streams start.

## Interfaces

Topics and message types are defined in [`topics-and-interfaces.md`](../../docs/system-map/topics-and-interfaces.md), frames in [`naming.md`](../../docs/system-map/naming.md).
The rows that involve perception are still TBD there:

| Direction | Interface | Other side |
|---|---|---|
| in | Camera images | camera driver / simulation |
| in | Drone pose | PX4 onboard estimate |
| out | Branch location | flight control |

## Setting up the Jetson

What the Jetson needs besides this repo (cloned to `~/vespr` on ours):

- **librealsense built from source with `BUILD_ROSBAG2=OFF`.** Since 2.58, librealsense records
  in the ROS 2 format (`.db3`) by default and refuses to write `.bag`, so the recorder fails. The
  Mac's `pyrealsense2-macosx` 2.54.2 cannot read `.db3`, and raw depth is kept in `.bag` files
  only. Ours is v2.58.4, built in `~/librealsense/build` with:

  ```bash
  cmake .. -DCMAKE_BUILD_TYPE=Release -DFORCE_RSUSB_BACKEND=ON -DBUILD_PYTHON_BINDINGS=ON \
           -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF -DBUILD_ROSBAG2=OFF
  make -j5
  ```

- **A Python environment** with `numpy`, `opencv-python` and `pytest`, which finds
  `pyrealsense2` in `~/librealsense/build/Release` through a `.pth` file. Ours is
  `~/d435i-check/.venv`; run the commands below with its `python`.
- **tmux and rclone** (see Google Drive storage), and the cable fix below if a Mac connects over
  USB-C.
- **The camera on a USB 3 port** for 30 fps. Check the Jetson sees it with `lsusb`, which lists
  "Intel(R) RealSense(TM) Depth Camera 435i".

## Recording clips

Record on the Jetson, not on a Mac. On 2026-10-11 a Mac kernel-panicked twice while the
recorder was streaming from the D435i. The recorder runs on the Jetson, and you control it from a
Mac: its page, with the live left IR image and a Record / Stop button, opens in the Mac's browser
through an SSH tunnel.

### Connecting the Mac to the Jetson

| Where | Connection | Jetson address | Internet needed |
|---|---|---|---|
| Indoors | Both on the same Wi-Fi | its Wi-Fi address | No |
| Outdoors | A USB-C cable from the Mac to the Jetson's USB-C port | `192.168.55.1` | No |

Over the cable the Mac gets `192.168.55.100` by itself. If macOS asks whether to allow the
accessory, allow it. The cable was tested on 2026-10-18 (JetPack R39.2.1).

**Once per Jetson: fix the cable link for Macs.** The Jetson's USB-C port carries two network
links: RNDIS for Windows (`usb0`) and NCM for macOS (`usb1`). NVIDIA's startup script,
`/opt/nvidia/l4t-usb-device-mode/nv-l4t-usb-device-mode-start.sh`, sets up `usb0` twice and
never `usb1`, so a Mac's link stays down. A systemd drop-in attaches `usb1` after that script
runs, and a JetPack update that replaces the script leaves it in place. Run this from the Mac
once, over the cable while the link is up or over Wi-Fi:

```bash
ssh -t perch@<jetson address> 'sudo mkdir -p /etc/systemd/system/nv-l4t-usb-device-mode.service.d && printf "%s\n" "[Service]" "# NVIDIA'"'"'s script sets up usb0 twice and never usb1, the NCM link macOS uses." "ExecStartPost=/bin/sh -c '"'"'if [ -e /sys/class/net/usb1 ]; then /sbin/ip link set dev usb1 master l4tbr0; /sbin/ip link set dev usb1 up; fi'"'"'" | sudo tee /etc/systemd/system/nv-l4t-usb-device-mode.service.d/usb1.conf && sudo systemctl daemon-reload'
```

Until the Jetson reboots, bring the link up by hand on the Jetson:
`sudo ip link set usb1 master l4tbr0` and then `sudo ip link set usb1 up`.

### Starting a session

```bash
# outdoors, on the Mac: copy the Mac's clock to the Jetson, which has no internet to set it
ssh -t perch@192.168.55.1 "sudo date -s @$(date +%s)"

# on the Mac: log in to the Jetson and forward the page to the Mac
ssh -L 8080:localhost:8080 perch@<jetson address>

# then, in that ssh session on the Jetson
tmux new -A -s record          # sudo apt install tmux if it is missing
python -m perch_perception.data_collection.record --domain outdoor     # or indoor
```

Then open http://localhost:8080 on the Mac. Nothing is uploaded while you record: once the
Jetson is back on Wi-Fi, send the clips with `clip_storage upload` (see Google Drive storage).

The recorder runs inside `tmux`, so a dropped ssh connection does not stop it, even in the
middle of a clip. Log in again with the same `ssh -L` command and run `tmux attach -t record` to
get its terminal back. Press Ctrl+C there to quit the recorder.

### What a clip is

`--domain` picks the folder, `~/vespr_recordings/indoor` or `~/vespr_recordings/outdoor`, the
same folders `clip_storage` uploads from. `--out` picks another folder. Each clip is two files
named by its start time, for example `2026-10-10T14-32-05`:

| File | What it is |
|---|---|
| `.bag` | Raw depth and left IR. This is the data, and `run.py --bag` replays it. |
| `.mp4` | A small grayscale video for watching the clip. Lossy: never use it for training or measurement. |

The names come from the Jetson's clock, and without internet the Jetson cannot set it. Until it
reaches the internet, its clock starts at the date its system image was built (June 5, 2026 on
ours), so outdoor clips would be named June 5. So outdoors, copy the Mac's clock to it first
(see Starting a session). The
recorder also prints the clock's time when it starts, so check it then. A clip never replaces an
earlier one: if the name is taken, the new clip gets `-2`, `-3` and so on.

The folders are in the home folder on purpose: macOS can sync Desktop and Documents to iCloud
Drive, and clips are large.

The camera restarts when recording starts and stops, and each camera session runs in its own
process, so expect a pause of about five seconds each time. A `.bag` is uncompressed: at 848x480
it grows by about 12 MB per second at 10 fps and about 37 MB per second at 30 fps.

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
python -m perch_perception.data_collection.clip_storage upload --domain indoor              # send finished clips not on the Drive yet
python -m perch_perception.data_collection.clip_storage download --domain indoor NAME ...   # fetch named clips
python -m perch_perception.data_collection.clip_storage download --domain indoor --missing  # fetch every clip not here yet
python -m perch_perception.data_collection.clip_storage cleanup --domain indoor             # list local clips safe to delete
python -m perch_perception.data_collection.clip_storage cleanup --domain indoor --yes       # delete them
```

The recorder's `--domain` saves clips straight into these folders, so they are ready to upload.

Run `upload` on the Jetson after a recording session, when it is online. Re-running it is always
safe: it only sends what is still missing. A session's clips can take a long time over Wi-Fi, so
when you start it over ssh, run it inside `tmux` (`sudo apt install tmux` if it is missing): start
`tmux`, then the command, and after a dropped connection ssh back in and run `tmux attach`.
Otherwise the upload stops when the ssh connection drops. Over the Jetson's Wi-Fi it sent about 4
MB per second on 2026-10-19, so a 20-second clip at 30 fps (about 0.75 GB) takes about three
minutes. A clip changed in the last minute is
skipped as still being written. Nothing on the Drive is ever overwritten or deleted. `cleanup`
deletes a local clip only when every one of its files is on the Drive with a matching checksum.
`--folder` and `--remote` change the local folder and the Drive location.

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

Without ROS, run pytest from this folder. On a Mac it needs Python 3.11 with `numpy`,
`opencv-python`, `pytest` and `pyrealsense2-macosx==2.54.2`. On the Jetson, use the Python
environment from Setting up the Jetson. Tests that need `pyrealsense2` or `cv2` are skipped where
they are missing. All 132 pass on both (librealsense 2.54.2 on the Mac, 2.58.4 on the Jetson).

```bash
python -m pytest
```
