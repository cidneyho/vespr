# Mocap: from a reflective ball to a number

Board box 07. See the [index](README.md).

OptiTrack's own software (Motive) is Windows-only and its Linux SDK stops at 2D blobs. Everything above that is ours, which is why we can explain every number the system produces.

## Pipeline layers

| # | Layer | What happens |
|---|---|---|
| 0 | Light | 850 nm IR ring + 850 nm bandpass filter: the camera sees mostly its own light bouncing off retroreflective 15.9 mm markers. Sunlight carries 850 nm too (the likely noise source). Filter stays on during calibration. |
| 1 | Frames | 10 cameras, PoE, hardware-synced at 120 Hz with a shared frame number. A marker seen by cam 3 and cam 7 in frame 4,812 was in the same place at the same instant. |
| 2 | Blobs (object mode) | Each camera thresholds on-board and sends only centroids: (x, y) and size per marker. Not the image. This is the mode mocap actually runs in; grayscale is for setup. **The open bug lives here.** |
| 3 | Intrinsics | Focal length, image centre, distortion curve per lens (ChArUco + coverage grid). Pixel → ray. Acceptance = the coverage grid, NOT RMS: a mid-frame-only calibration can be 10× wrong at the edges. |
| 4 | Extrinsics | Where each camera is relative to the others. Wave the 3-marker wand (0 / 160 / 400 mm, asymmetric so markers are identifiable), solve across all cameras. |
| 5 | Ground plane | The L-frame with unequal arms defines origin, up and forward. After this, (1.2, 0.4, 1.8) means something in the room. |
| 6 | Triangulation | One camera gives a ray; two or more give intersecting rays, and the intersection is a 3D point. More cameras = better; the drone body hides markers (occlusion check). |
| 7 | Labeling + rigid bodies | Markers in a pattern with unique inter-marker distances → one 6-DoF pose per body per frame, timestamped. Output: `/mocap/vespr/pose` and `/mocap/branch/pose`. |
| 8 | Output | A small ROS 2 publisher on Wi-Fi, stamped from the shared clock. The only piece of the mocap repo that knows ROS exists. |

## What the evaluator computes

- Compare **RELATIVE pose (drone-to-branch), not absolute.** Mocap reports in the room frame; VIO reports in its own frame that starts wherever the drone booted, and aligning the two adds error. Drone-to-branch from mocap is one subtraction; from vision it is what the detector sees directly. Same frame, no alignment.
- Then **TIME**: an 8 ms stamp offset at 0.5 m/s is 4 mm of fake error. Every pose carries the shared-clock stamp, and the evaluator interpolates mocap to the vision timestamp before differencing.

## The two-repo boundary

| Repo | Runs on | Contents |
|---|---|---|
| Mocap repo | ground PC | Ends at layer 8. Camera capture, calibration, marker tracking, rigid bodies, one ROS node at the edge: `mocap_node`. Knows nothing about drones. |
| VESPR repo (this one) | Jetson | `perch_bringup` (launch), `perch_interfaces` (our types), `perch_perception`, `perch_flight`, `perch_mission`, `perch_legs`, `perch_eval`, `perch_sim`. Flies without mocap running. |

The contract between them: **2 topics · 1 type · 1 frame · 1 clock.**

## Phasing out mocap

Mocap is taken out in stages.

| Stage | Name | What happens |
|---|---|---|
| 1 | Mocap stands in | Vision not ready. The mission node's "drone pose" is remapped to `/mocap/vespr/pose` so flight and legs get tested early. A known, temporary violation of [the rule](overview.md#the-one-rule-mocap-measures-it-never-steers): agree on it as a stage with an end date. |
| 2 | Side by side | Vision publishes its own estimate. Mission still flies on mocap; the evaluator logs vision vs mocap on every flight. This is where you learn whether vision meets the error budget. |
| 3 | Vision flies | The remap flips to `/vespr/perception/odom`. Mocap still runs, still logged, still feeds the evaluator, but nothing it says reaches the flight loop. **Every result in the report comes from here.** |
| 4 | Mocap off | The mocap node is not launched. Nothing on the drone changes, because nothing depended on it. |

Switching stages is one line in a launch file.

**Proof you are in stage 3:** `ros2 topic info /mocap/vespr/pose` lists the evaluator and the bag recorder as subscribers and nothing else. Run it and screenshot it before every data-collection flight.
