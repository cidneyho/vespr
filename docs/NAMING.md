# VESPR Naming Conventions

Single reference for every name in the VESPR repo: ROS 2 topics, services, actions, nodes, packages, interface types, frames, and files. Claude and humans both follow this when creating or naming anything.

**Sources:** `VESPR_System_Map.pdf` (poster, boxes 03 to 11) and `ROS 2 for VESPR` (slide deck). Where they disagree, the poster wins because it is newer and carries the team's decided stickies. Conflicts are listed in section 11.

**Status labels used below**
- **Decided**: agreed by the team (poster box 11 stickies or "Already settled").
- **Proposed**: in the docs as a proposal, not yet formally agreed. Use it, but do not treat it as frozen.
- **Third-party**: owned by PX4 or Intel. Never rename.
- **Gap-fill**: not in either source. Standard ROS 2 practice added so Claude has a default. Confirm with the team before relying on it.

---

## 1. The six naming rules (Proposed)

1. **snake_case everywhere.** Topics, nodes, packages, parameters, frames. Only type names (messages, services, actions) are CamelCase.
   `branch_pose`, not `BranchPose`.
2. **System, then subsystem.** Ours start with `/vespr`, mocap's start with `/mocap`, then one level per subsystem.
   `/vespr/legs/state`
3. **Nouns for data, verbs for requests.** A topic is a thing. Services and actions are verbs, because they ask for something.
   `state` (topic) · `set_enabled` (service) · `grasp` (action)
4. **Keep third-party names.** PX4's `/fmu` names and the RealSense driver's sub-names stay exactly as shipped, so their docs and examples still apply.
   `/fmu/out/vehicle_status`
5. **Short names in code.** Nodes use the short name (`branch_pose`). The launch file adds the namespace and does any remapping, so a rename never touches a node's source.
   `namespace="vespr/perception"`
6. **No units in names.** Always metres, radians, seconds (SI, REP 103). The frame goes in the message header, not the name.
   `range`, not `range_mm`

Conventions referenced: REP 103 (units, axes), REP 105 (frames), REP 144 (package names), ROS 2 topic name rules.

---

## 2. Root decisions (Decided)

| Question | Decision |
|---|---|
| Root namespace | `/vespr` (finalized) |
| Camera namespace | `/vespr/camera` for the RealSense (moved from driver default `/camera/camera`) |
| Word for the perch target | `branch` (finalized, even if we later use poles or wires) |
| Rangefinder | Wired to the Pixhawk, so it appears as `/fmu/out/distance_sensor`. There is no `/vespr/tof/...` topic. |
| Legs hardware path | ESP32 with DRV8833 drivers, reached from ROS only through `legs_bridge` |
| ROS distro / OS | ROS 2 Jazzy on Ubuntu 24.04 |

---

## 3. Name tree (Proposed unless marked)

```
/vespr/
  camera/                         # Decided location; sub-names are Intel's
    color/image_raw
    color/camera_info
    aligned_depth_to_color/image_raw
    imu
  perception/
    branch_pose
    odom
  legs/
    state
    set_enabled                   # service
    grasp                         # action
    release                       # action
  eval/
    pose_error
    branch_pose_truth
  mission/
    state
  perch                           # action
/mocap/                           # published by the mocap repo
  vespr/pose
  branch/pose
/fmu/                             # Third-party (PX4)
  in/...                          # we publish, PX4 subscribes
  out/...                         # PX4 publishes, we subscribe
```

---

## 4. Full interface table

### Our topics, services, actions

| Kind | Full name | Type | From → To | Status |
|---|---|---|---|---|
| topic | `/vespr/camera/color/image_raw` | `sensor_msgs/msg/Image` | realsense_driver → perception | Decided |
| topic | `/vespr/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/msg/Image` | realsense_driver → perception | Decided |
| topic | `/vespr/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | realsense_driver → perception | Decided |
| topic | `/vespr/camera/imu` | `sensor_msgs/msg/Imu` | realsense_driver → perception | Decided |
| topic | `/vespr/perception/branch_pose` | `geometry_msgs/msg/PoseStamped` | branch_detector → mission, evaluator | Proposed |
| topic | `/vespr/perception/odom` | `nav_msgs/msg/Odometry` | vio → mission, evaluator | Proposed |
| topic | `/vespr/eval/pose_error` | `vespr_interfaces/msg/PoseError` | evaluator → bag, plots | Proposed |
| topic | `/vespr/eval/branch_pose_truth` | not specified | evaluator | Proposed |
| topic | `/vespr/mission/state` | not specified | mission → bag, operator | Proposed |
| topic | `/vespr/legs/state` | `sensor_msgs/msg/JointState` (or a small custom msg) | legs_bridge → mission, bag, RViz | Proposed |
| service | `/vespr/legs/set_enabled` | `std_srvs/srv/SetBool` | mission, operator → legs_bridge | Proposed |
| action | `/vespr/legs/grasp` | `vespr_interfaces/action/Grasp` | mission → legs_bridge | Proposed |
| action | `/vespr/legs/release` | mirrors Grasp (type not yet defined) | mission → legs_bridge | Proposed |
| action | `/vespr/perch` | `vespr_interfaces/action/Perch` | operator → mission | Proposed |
| topic | `/mocap/vespr/pose` | `geometry_msgs/msg/PoseStamped` | mocap_node → evaluator (and mission in stage 1) | Proposed |
| topic | `/mocap/branch/pose` | `geometry_msgs/msg/PoseStamped` | mocap_node → evaluator | Proposed |

### PX4 topics (Third-party, via micro_xrce_agent)

| Direction | Topic | Meaning |
|---|---|---|
| we publish | `/fmu/in/offboard_control_mode` | heartbeat; must arrive at least 2 Hz or PX4 leaves offboard |
| we publish | `/fmu/in/trajectory_setpoint` | where to go: position, velocity, yaw |
| we publish | `/fmu/in/vehicle_command` | arm, switch mode, land |
| we publish | `/fmu/in/vehicle_visual_odometry` | outside pose for PX4 to fuse (vision, stage 3) |
| we subscribe | `/fmu/out/vehicle_odometry` | PX4's pose and velocity estimate |
| we subscribe | `/fmu/out/vehicle_local_position` | listed in poster box 03 as what mission reads |
| we subscribe | `/fmu/out/vehicle_status` | armed, flight mode, failsafe |
| we subscribe | `/fmu/out/vehicle_command_ack` | whether PX4 accepted the last command |
| we subscribe | `/fmu/out/vehicle_land_detected` | on ground or in air |
| we subscribe | `/fmu/out/distance_sensor` | the rangefinder |
| we subscribe | `/fmu/out/battery_status` | voltage, current |
| we subscribe | `/fmu/out/sensor_combined` | flight controller IMU (`px4_msgs/msg/SensorCombined`) |

Some PX4 names carry a version suffix on recent firmware (for example `_v1`). Confirm with `ros2 topic list` on the real vehicle and match `px4_msgs` to the firmware. Every subscriber to `/fmu/out/*` must use **best effort** QoS.

---

## 5. Nodes

| Node name | Package | Job | Status |
|---|---|---|---|
| `branch_detector` | `vespr_perception` | images → branch pose | Proposed |
| `vio` | `vespr_perception` | visual odometry → drone pose | Proposed |
| `mission` | `vespr_flight` | state machine; only node that commands actuators | Proposed |
| `legs_bridge` | `vespr_legs` | serial ↔ ROS translator for the ESP32 | Proposed |
| `evaluator` | `vespr_eval` | vision vs mocap, relative pose first | Proposed |
| `bag_recorder` | not specified | every topic to a rosbag on the NVMe | Proposed |
| `realsense_driver` | third-party (`realsense2_camera`) | launched from `vespr_bringup` | Third-party |
| `micro_xrce_agent` | third-party | PX4 bridge; we run it, we do not write it | Third-party |
| `mocap_node` | mocap repo | the only ROS node in the mocap repo | Proposed |

---

## 6. Packages and repos

**VESPR repo (on the Jetson).** All our packages use the `vespr_` prefix (Proposed):

| Package | Contents |
|---|---|
| `vespr_bringup` | launch files only; sets namespaces and remaps |
| `vespr_interfaces` | our custom types, and only those |
| `vespr_perception` | branch detector, VIO |
| `vespr_flight` | mission logic |
| `vespr_legs` | legs bridge |
| `vespr_eval` | vision vs mocap evaluation |

**Mocap repo (on the ground PC).** Separate repo, knows nothing about drones. Its only ROS-facing piece is `mocap_node`. Raw session data lives under `~/panoptes-data/sessions`.

**The two-repo contract:** 2 topics (`/mocap/vespr/pose`, `/mocap/branch/pose`), 1 type (`PoseStamped`), 1 frame name, 1 shared clock. Neither repo imports the other's code.

---

## 7. Interface types (CamelCase)

Reuse standard types first. Only three custom types exist, all in `vespr_interfaces` (Proposed):

| Type | File |
|---|---|
| `vespr_interfaces/msg/PoseError` | `vespr_interfaces/msg/PoseError.msg` |
| `vespr_interfaces/action/Grasp` | `vespr_interfaces/action/Grasp.action` |
| `vespr_interfaces/action/Perch` | `vespr_interfaces/action/Perch.action` |

Adding a fourth custom type is a team decision, not a coding decision.

---

## 8. Frames

| Frame | Meaning | Status |
|---|---|---|
| `mocap` | the room (placeholder name) | Proposed |
| `base_link` | the drone's body | Proposed (REP 105) |
| `camera_link` → `camera_color_optical_frame` | set by the mount; published by the camera driver | Third-party |
| `branch` | the perch target; where its origin sits must be the same in mocap and vision | Proposed |
| `map`, `odom` | REP 105 world and odometry frames | ROS standard |

Rules:
- Every pose carries `header.stamp` and `header.frame_id`. "The branch is 0.4 m ahead" means nothing without the frame.
- ROS is x forward, y left, z up (ENU / FLU). PX4 is x forward, y right, z down (NED / FRD). Convert in **one node only**.
- The mocap marker cluster is not `base_link`. Measure the fixed offset once.
- Mocap and vision outputs for the same quantity must share **type and frame**. That is what makes the remap swap and the comparison possible.

---

## 9. Launch-file naming and remaps

- Code subscribes to short names (`branch_pose`, `odom`). The launch file in `vespr_bringup` sets the namespace and remaps.
- Mocap phase-out is one remap of the mission node's "drone pose" input:
  - Stages 1 and 2: remap to `/mocap/vespr/pose`
  - Stages 3 and 4: remap to `/vespr/perception/odom`
- RealSense: out of the box the driver publishes under `/camera/camera/...`. Two launch settings move it under `/vespr/camera/...` (Gap-fill: in realsense-ros 4.x these are `camera_namespace:=vespr` and `camera_name:=camera`; aligned depth also needs `align_depth.enable:=true`. Verify on the Jetson.)

---

## 10. Files (Gap-fill unless marked)

Neither source specifies file names beyond packages and types. These defaults follow standard ROS 2 practice so Claude has something consistent to apply.

| Thing | Convention | Example |
|---|---|---|
| Package directory | same as package name | `src/vespr_perception/` |
| Python node file | snake_case, matches node name | `vespr_perception/vespr_perception/branch_detector.py` |
| C++ node file | snake_case, matches node name | `vespr_legs/src/legs_bridge.cpp` |
| Launch file | snake_case, `.launch.py` | `vespr_bringup/launch/vespr.launch.py` |
| Parameter file | snake_case, `.yaml`, in `config/` | `vespr_bringup/config/realsense.yaml` |
| Interface file | CamelCase, in `msg/`, `srv/`, `action/` | `vespr_interfaces/action/Grasp.action` |
| Test file | `test_` + module name | `vespr_eval/test/test_evaluator.py` |
| Parameters | snake_case, SI units, no unit suffix | `confidence_threshold`, `standoff_distance` |
| Rosbag | one bag per flight, named by date and test ID (**source**); suggested format `YYYY-MM-DD_<test_id>` (gap-fill) | `2026-10-20_approach_03` |

---

## 11. Conflicts between the two sources

The poster is treated as current. If you see the old name anywhere, it means the new one.

| Old (ROS 2 deck) | Current (poster) | Note |
|---|---|---|
| `perch_manager` | `mission` | mission node |
| `leg_controller` | `legs_bridge` | legs node, now talks to an ESP32 |
| `pose_eval` | `evaluator` | evaluation node |
| `realsense2_camera` | `realsense_driver` | node label; package is still `realsense2_camera` |
| `/vespr/perception/odometry` | `/vespr/perception/odom` | `odom` also matches REP 105 |
| `/vespr/tof/range`, `tof_driver` | `/fmu/out/distance_sensor` | rangefinder is on the Pixhawk |
| (not in deck) | `/vespr/mission/state`, `vio`, `bag_recorder`, `micro_xrce_agent` | added in poster |

Also unresolved inside the poster: mission reads `/fmu/out/vehicle_local_position` in box 03 but the PX4 table in box 05 lists `/fmu/out/vehicle_odometry`.

---

## 12. Still open (do not invent answers)

- What perception publishes: branch pose only, or also drone odometry for PX4 (stage 3)?
- What mocap promises: rate, frame name, and which point on drone and branch each pose means. The `mocap` frame name is a placeholder until this is answered.
- Mocap stand-in: the sticky says "first we fly using mocap", but "Already settled" says mocap is ground truth only. Resolve before writing the stage 1 remap.
- ESP32 link: plain serial + `legs_bridge` (starting point) or micro-ROS on the ESP32. Names do not change either way.
- `/vespr/legs/state` type: `JointState` or a small custom msg. Depends on what the legs can sense.
- Types for `/vespr/eval/branch_pose_truth`, `/vespr/mission/state`, and the `release` action.
- Which package owns `bag_recorder`.

---

## Rules for Claude when creating or naming anything

1. Look up the name in this file first. If it exists, use it exactly.
2. If it does not exist, derive it from the six rules in section 1 and the file defaults in section 10, then tell the user it is a new name that is not yet in this file.
3. Never rename a Third-party name.
4. Never add a custom type to `vespr_interfaces` without the user confirming.
5. Never put a namespace in node source code. Namespaces live in `vespr_bringup` launch files.
6. If a name touches anything in section 12, stop and ask instead of picking one.
