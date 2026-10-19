# Interfaces

Topics, services, actions, message types and coordinate frames shared between the perception, flight, legs, evaluation, simulation and motion capture subsystems.

Source: the team's system map board ([docs/system-map/](system-map/README.md), mainly [topics-and-interfaces.md](system-map/topics-and-interfaces.md) and [naming.md](system-map/naming.md)). Names follow [NAMING.md](../NAMING.md). `/vespr` and `/mocap` names are proposals the team agreed to build against; `/fmu` and RealSense names belong to PX4 and Intel and are never renamed.

Conventions for every interface:
- SI units (metres, radians, seconds). The frame goes in the message header, never in the name.
- Every pose is stamped from the shared clock (chrony / PTP) so the Jetson and mocap logs agree on time.

## Coordinate frames

REP 103 / REP 105. ROS frames are ENU-style (x forward, y left, z up). PX4 is NED (x forward, y right, z down). Convert between them in one node only.

| Frame | Definition | Published by | Used by |
|---|---|---|---|
| `mocap` | The room. Origin, up and forward set by the mocap L-frame ground plane (placeholder name) | mocap node | mocap poses, evaluator |
| `base_link` | The drone's body. The mocap marker cluster is **not** `base_link`: measure the fixed offset once | TBD (static transform) | perception, mission, evaluator |
| `camera_link` → `camera_color_optical_frame` | RealSense D435i, set by the mount | RealSense driver | perception |
| `branch` | The perch target. Where its origin sits on the branch is TBD, and must match between the mocap body and the vision output | mocap node, perception | mission, evaluator |
| PX4 local frame (NED) | PX4's own frame for all `/fmu/*` data | PX4 | mission (converted in one node) |

## Topics

| Interface | Topic | Message type | Publisher → Subscriber | Frame / units / rate |
|---|---|---|---|---|
| Color image | `/vespr/camera/color/image_raw` | `sensor_msgs/msg/Image` | RealSense driver (or `ros_gz_bridge` in sim) → perception | `camera_color_optical_frame` |
| Depth, aligned to color | `/vespr/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/msg/Image` | RealSense driver (or sim) → perception | `camera_color_optical_frame` |
| Camera calibration | `/vespr/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | RealSense driver (or sim) → perception | `camera_color_optical_frame` |
| Camera IMU | `/vespr/camera/imu` | `sensor_msgs/msg/Imu` | RealSense driver → perception | camera frame |
| Branch location (vision) | `/vespr/perception/branch_pose` | `geometry_msgs/msg/PoseStamped` | branch_detector → mission, evaluator | drone frame; position + branch axis |
| Drone pose (vision) | `/vespr/perception/odom` | `nav_msgs/msg/Odometry` | vio → mission, evaluator | position + velocity; VIO frame starts where the drone booted |
| Drone pose (mocap, ground truth) | `/mocap/vespr/pose` | `geometry_msgs/msg/PoseStamped` | mocap node (mocap repo) → evaluator, bag_recorder | `mocap`; rate TBD (cameras run at 120 Hz) |
| Branch pose (mocap, ground truth) | `/mocap/branch/pose` | `geometry_msgs/msg/PoseStamped` | mocap node (mocap repo) → evaluator, bag_recorder | `mocap`; rate TBD |
| Pose error | `/vespr/eval/pose_error` | `vespr_interfaces/msg/PoseError` | evaluator → bag_recorder | drone-to-branch relative pose, vision vs mocap |
| Branch pose truth | `/vespr/eval/branch_pose_truth` | TBD | evaluator → bag_recorder | TBD |
| Mission state | `/vespr/mission/state` | TBD | mission → bag_recorder | current [perch-cycle](system-map/perch-cycle.md) state |
| Leg state | `/vespr/legs/state` | `sensor_msgs/msg/JointState` (or a small custom msg) | legs_bridge → mission, bag_recorder, RViz | latch position, gripped, motor current |

**Mocap stays out of flight.** Only the evaluator and bag_recorder subscribe to `/mocap/*`. The one exception is the temporary stage 1 stand-in, where the mission node's drone-pose input is remapped to `/mocap/vespr/pose` in the launch file (see [mocap phase-out](system-map/mocap.md#phasing-out-mocap)). In stage 3, `ros2 topic info /mocap/vespr/pose` must list only the evaluator and bag_recorder.

## Services

| Interface | Service | Type | Server → Client | Notes |
|---|---|---|---|---|
| Enable leg motors | `/vespr/legs/set_enabled` | `std_srvs/srv/SetBool` | legs_bridge ← mission | Turns the motor drivers on or off. Also the software stop |

## Actions

| Interface | Action | Type | Server → Client | Notes |
|---|---|---|---|---|
| Grasp branch | `/vespr/legs/grasp` | `vespr_interfaces/action/Grasp` | legs_bridge ← mission | Close until gripped; progress feedback; cancellable |
| Release branch | `/vespr/legs/release` | `vespr_interfaces/action/Grasp` | legs_bridge ← mission | Open the cuff |
| Perch | `/vespr/perch` | `vespr_interfaces/action/Perch` | mission ← TBD | Whole perch cycle; reports its phase; cancellable |

## Custom types (`vespr_interfaces`)

Only three types are ours. All live in `vespr_interfaces`:

| Type | Kind | Used by |
|---|---|---|
| `PoseError` | msg | `/vespr/eval/pose_error` |
| `Grasp` | action | `/vespr/legs/grasp`, `/vespr/legs/release` |
| `Perch` | action | `/vespr/perch` |

Field definitions: TBD.

## PX4 interface

Via the Micro XRCE-DDS Agent (`micro_xrce_agent`), over TELEM2 serial at 921600 baud on the drone and UDP in SITL. Types from `px4_msgs` (`release/1.16`, must match the PX4 firmware). All data in PX4's NED frame.

| Direction | Topic | Meaning |
|---|---|---|
| mission → PX4 | `/fmu/in/offboard_control_mode` | Heartbeat: we are in control, and of which quantity. **At least 2 Hz**, or PX4 leaves offboard (failsafe) |
| mission → PX4 | `/fmu/in/trajectory_setpoint` | Where to go: position, velocity, yaw |
| mission → PX4 | `/fmu/in/vehicle_command` | One-off commands: arm, switch mode, land |
| vio → PX4 *(stage 3)* | `/fmu/in/vehicle_visual_odometry` | Vision pose for the EKF to fuse |
| PX4 → mission | `/fmu/out/vehicle_odometry` | PX4's best estimate of pose and velocity |
| PX4 → mission | `/fmu/out/vehicle_local_position` | Where PX4 thinks it is |
| PX4 → mission | `/fmu/out/vehicle_status` | Armed or not, flight mode, failsafe state |
| PX4 → mission | `/fmu/out/vehicle_command_ack` | Whether PX4 accepted the last command |
| PX4 → mission | `/fmu/out/vehicle_land_detected` | On the ground or in the air |
| PX4 → mission | `/fmu/out/distance_sensor` | Rangefinder (wired to the Pixhawk) |
| PX4 → mission | `/fmu/out/battery_status` | Voltage, current |
| PX4 → TBD | `/fmu/out/sensor_combined` | Flight controller IMU (`px4_msgs/msg/SensorCombined`) |

- **QoS:** PX4 publishes best effort. Every subscriber to `/fmu/out/*` must use best effort, or it silently receives nothing.
- Some topic names carry a version suffix on recent firmware. Confirm with `ros2 topic list` on the real vehicle.
- PX4 only mirrors the topics listed in its `dds_topics.yaml`.

## Non-ROS links

| Link | Between | Protocol |
|---|---|---|
| Legs serial | legs_bridge ↔ ESP32 (USB) | Plain text serial (`grasp` / `release` / `status`) or micro-ROS, not decided yet |
| Pilot override | TX16S → XR1 receiver → PX4 RC IN | ELRS, then SBUS. PX4 obeys this over the Jetson |
| Ground station | QGroundControl ↔ PX4 | MAVLink |
| Clock | Mocap PC ↔ Jetson | chrony or PTP |

Full physical wiring: [system-map/wiring.md](system-map/wiring.md).
