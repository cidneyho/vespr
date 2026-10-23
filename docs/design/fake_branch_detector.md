# Design: fake_branch_detector

Status: draft for review. Nothing is implemented yet.

## Goal

Stand in for the branch detector in Gazebo, so the mission can be built before real perception exists. Publish where the branch is relative to the drone, only while the drone's camera could see it, using Gazebo's ground truth. Perfect by default, with configurable noise.

## Output

| Topic | Type | Frame | Meaning |
|---|---|---|---|
| `/vespr/perception/branch_pose` | `geometry_msgs/msg/PoseStamped` | `base_link` | The branch's `perch_point`, from the centre of the drone's body. x along the branch, z up |

- **Published only when the branch is in view** (below). No message means "not seen".
- **Stamp:** the Gazebo sim time of the sample, as in `mocap_sim`.
- **Branch direction:** the branch has no front, so x could point either way along it. The detector always points x to the drone's **left** (positive y in `base_link`), flipping the pose 180° about its z axis when needed. A real detector can't tell the ends apart either, and this keeps the pose from jumping when the drone turns. If the branch runs exactly fore and aft, x points forward.

## The camera

The fake detector uses the drone model's real camera mount, read from Gazebo, so the view check follows the model if the mount changes.

- **Camera:** an Intel RealSense D435i, as on the real drone. PX4 has no RealSense model, so `x500_hook` gets a Gazebo RGB-D camera with the D435i's colour view: 69° × 42°, 30 Hz, depth aligned to colour. It renders at 1280 × 720, a D435i mode with the same view as 1920 × 1080 that is much cheaper for Gazebo to render.
- **Mount:** the real RealSense goes where PX4's `x500_depth` mounts its OakD-Lite: `camera_link` fixed to `base_link` at (0.12, 0.03, 0.242) m, facing straight ahead.
- **Where it's set:** the `drone.camera` block in `perch_flight/config/hook.yaml`. `perch_sim/scripts/make_hook_model.py` builds the camera into the model from it, alongside the hook. Change the mount there and rerun the script.
- **Images:** Gazebo publishes `/camera/image`, `/camera/depth_image`, `/camera/points` and `/camera/camera_info`, which the camera mock (a later step) will bridge for perception.

## When the branch counts as "in view"

The `perch_point` is converted into the camera's frame (x forward, y left, z up). It's in view when all three hold:

| Check | Default |
|---|---|
| Distance ahead (x) within range | 0.3 to 3.0 m |
| Left/right angle, atan2(y, x), within half the width | ±34.5° |
| Up/down angle, atan2(z, x), within half the height | ±21° |

The range is the D435i's good-depth range; beyond it, depth gets too noisy for a detector to trust.

Limits:
- Only the perch point is tested, not the whole branch. A branch partly in view counts as seen if the point is in view.
- Nothing blocks the view. The posts and the trunk are ignored. The hook ring is behind the camera, so it can't block anything.

**Expected blind spot:** the camera looks straight ahead and sees only 21° up, so the branch leaves the view during the final rise under it (0.4 m ahead and 0.3 m up is already 37° up). At touchdown the branch is inside the hook, behind the camera. The real drone behaves the same way, so the mission must finish the approach without `branch_pose`.

## Source: Gazebo ground truth

Same as `mocap_sim`: read Gazebo directly over gz-transport, finding entities by name.

| Gazebo topic | Used for |
|---|---|
| `/world/<world>/dynamic_pose/info` | Drone model, `base_link`, `camera_link` |
| `/world/<world>/pose/info` | Branch `perch_point`, read once |

## Rate and imperfections

- At most `rate` samples a second (default 30, the camera's frame rate), using `mocap_sim`'s rate limit: never above `rate`, never a sample twice. Gazebo's ~50 Hz pose rate is above 30, so 30 Hz is reachable.
- Noise, latency and dropped frames work as in `mocap_sim`, all off by default. Noise is applied in `base_link`, after the view check, so noise never decides whether the branch is seen.

## Parameters

| Name | Default | Meaning |
|---|---|---|
| `world` | `perch_rod` | Gazebo world name. The branch model is named after it |
| `drone_model`, `drone_link`, `camera_link` | `x500_hook_0`, `base_link`, `camera_link` | Which entities are the drone and its camera |
| `branch_model` | (the world name) | Which model is the branch |
| `rate` | 30.0 | Maximum publish rate, Hz |
| `fov_horizontal`, `fov_vertical` | 69.0, 42.0 | Camera view, degrees (D435i colour) |
| `min_range`, `max_range` | 0.3, 3.0 | Detection range ahead of the camera, m |
| `position_noise`, `orientation_noise` | 0.0, 0.0 | Gaussian noise, m and rad |
| `latency` | 0.0 | Delay before publishing, s |
| `drop_rate` | 0.0 | Chance (0 to 1) that a sample is dropped |

## Package: `vespr_sim`

| File | Contents |
|---|---|
| `vespr_sim/gazebo.py` | Moved out of `mocap_sim`: reading gz poses by name, and the rate limit |
| `vespr_sim/sim_sensor.py` | Base class for both nodes: parameters, reading the branch, dropped frames, noise, latency |
| `vespr_sim/fake_branch_detector.py` | The node, named `branch_detector` so the graph matches the real detector's |
| `vespr_sim/poses.py` | Gains `inverse`, for the branch relative to the drone |
| `launch/fake_branch_detector.launch.py` | The node, every parameter as a launch argument |

## Testing

- Unit: the view check (inside, too near, too far, outside each edge), the left-pointing rule, and the branch pose relative to the drone.
- Node test with fake Gazebo poses: publishes when in view, silent when not, the rate limit.
- Manual, in the sim: hover facing the rod and `ros2 topic echo /vespr/perception/branch_pose`. Turn away and the messages stop.

## Out of scope

- Real images and a real detector.
- Occlusion and false detections.

## Decided

- The hook generator and `hook.yaml` come over from the WIP branch (commit `7d8c06f`) unchanged; they reproduce the existing model exactly. The camera is added through them, not by hand.
- The blind spot during the final approach is expected.
