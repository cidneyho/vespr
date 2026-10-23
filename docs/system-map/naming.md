# Naming: frames, rules, tree

Board box 09. See the [index](README.md).

**[docs/NAMING.md](../NAMING.md) is the full naming reference for the repo** and already includes this box, with decided / proposed status. Follow it when naming anything. This file is a short summary of the board so the system map is complete.

## Frames

REP 103 / REP 105.

| Frame | Meaning |
|---|---|
| `mocap` | the room (placeholder name) |
| `base_link` | the drone's body |
| `camera_link` → `camera_color_optical_frame` | set by the mount; published by the camera driver |
| `branch` | the perch target. Decide where on the branch its origin sits, in both the mocap body and the vision output |

- Every pose names its frame: "the branch is 0.4 m ahead" means nothing until the header says ahead of what.
- PX4 and ROS disagree on axes (ROS x forward, y left, z up; PX4 x forward, y right, z down). Convert in one node only.
- The marker cluster is not `base_link`: measure the fixed offset once.

## Six rules (summary)

1. snake_case everywhere (only type names are CamelCase).
2. System, then subsystem: `/vespr/legs/state`, `/mocap/...`.
3. Nouns for data, verbs for services and actions.
4. Keep third-party names (`/fmu/out/vehicle_status`).
5. Short names in code; the launch file adds the namespace.
6. No units in names (SI always; frame goes in the header).

## Name tree (proposed)

| Namespace | Names |
|---|---|
| `/vespr/camera/` | `color/image_raw` · `color/camera_info` · `aligned_depth_to_color/…` · `imu` |
| `/vespr/perception/` | `branch_pose` · `odom` |
| `/vespr/legs/` | `state` · `set_enabled` (service) · `grasp` (action) · `release` (action) |
| `/vespr/eval/` | `pose_error` · `branch_pose_truth` |
| `/vespr/` | `mission/state` · `perch` (action) |
| `/mocap/` | `vespr/pose` · `branch/pose` |
| `/fmu/` | `in/…` (to PX4) · `out/…` (from PX4) |
| frames | `mocap` · `base_link` · `camera_link` · `branch` |

Decided by the team: root is `/vespr` (not a per-vehicle name), the target is called `branch` (also for poles and wires), the RealSense lives under `/vespr/camera`.
