# Topics and interfaces: all the data on VESPR

Board box 05. See the [index](README.md).

Sensors send data into ROS and actuators receive commands from it, each through exactly one node. Every `/vespr` and `/mocap` name is a **proposal**; `/fmu` and camera driver names belong to PX4 and Intel. Naming rules: [/NAMING.md](../../NAMING.md).

## Hardware → ROS entry points

| Hardware | Data | Flow | Enters ROS through |
|---|---|---|---|
| Mocap cameras | pose of the drone and of the branch | sends | mocap node, in the mocap repo |
| RealSense D435i camera | color and depth images | sends | RealSense driver node |
| RealSense D435i IMU | acceleration and rotation rate | sends | the same driver node |
| Rangefinder | one distance reading | sends | PX4 (wired to the Pixhawk), appears as `/fmu/out/distance_sensor` |
| Pixhawk flight controller | its IMU, pose estimate, battery, mode | both ways | PX4 bridge on the Jetson |
| Propeller motors (4) | thrust commands | receives | PX4 only. We send it setpoints |
| Leg motors (2 to 4 N20) | open and close commands, leg state | both ways | legs_bridge + ESP32 |
| RC transmitter | pilot sticks, mode and kill switches | sends | PX4, then the bridge |

## PX4 topics (through the micro XRCE agent)

| Direction | Topic | Meaning |
|---|---|---|
| we publish | `/fmu/in/offboard_control_mode` | heartbeat: we are in control, and of which quantity |
| we publish | `/fmu/in/trajectory_setpoint` | where to go: position, velocity, yaw |
| we publish | `/fmu/in/vehicle_command` | one-off commands: arm, switch mode, land |
| we publish | `/fmu/in/vehicle_visual_odometry` | an outside pose for PX4 to fuse: vision (stage 3) |
| we subscribe | `/fmu/out/vehicle_odometry` | PX4's best estimate of pose and velocity |
| we subscribe | `/fmu/out/vehicle_status` | armed or not, flight mode, failsafe state |
| we subscribe | `/fmu/out/vehicle_command_ack` | whether PX4 accepted the last command |
| we subscribe | `/fmu/out/vehicle_land_detected` | on the ground or in the air |
| we subscribe | `/fmu/out/distance_sensor` | the rangefinder |
| we subscribe | `/fmu/out/battery_status` | voltage, current: the energy numbers |

- PX4 is not a ROS node. It has its own message bus; the bridge mirrors a fixed list (`dds_topics.yaml`) as topics.
- We never drive propellers: we publish where the drone should go and PX4 turns that into four motor outputs.
- The heartbeat is a safety feature: PX4 stays in offboard only while it keeps arriving, **at least twice a second**.
- Some names carry a version suffix on recent firmware. Confirm with `ros2 topic list` on the real vehicle, and match `px4_msgs` to the firmware.
- Subscribe to `/fmu/out/*` with best-effort QoS (see [ros2-primer.md](ros2-primer.md#qos-the-silent-trap)).

## Camera, perception, mocap and eval topics

| Data | Topic | Message type |
|---|---|---|
| Color image | `/vespr/camera/color/image_raw` | `sensor_msgs/msg/Image` |
| Depth, aligned to color | `/vespr/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/msg/Image` |
| Camera calibration | `/vespr/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` |
| Camera IMU | `/vespr/camera/imu` | `sensor_msgs/msg/Imu` |
| Flight controller IMU | `/fmu/out/sensor_combined` | `px4_msgs/msg/SensorCombined` |
| Branch (vision) | `/vespr/perception/branch_pose` | `geometry_msgs/msg/PoseStamped` |
| Drone (vision) | `/vespr/perception/odom` | `nav_msgs/msg/Odometry` |
| Drone (mocap) | `/mocap/vespr/pose` | `geometry_msgs/msg/PoseStamped` |
| Branch (mocap) | `/mocap/branch/pose` | `geometry_msgs/msg/PoseStamped` |
| Error | `/vespr/eval/pose_error` | `vespr_interfaces/msg/PoseError` |

## Legs: one topic, one service, one action

The worked example for classifying any subsystem.

| Kind | Name | Type | What it does | Ask yourself |
|---|---|---|---|---|
| TOPIC | `/vespr/legs/state` | `sensor_msgs/msg/JointState` (or a small custom msg) | Where each leg is, published all the time. Mission, recorder and RViz can all listen. | What does it report continuously? |
| SERVICE | `/vespr/legs/set_enabled` | `std_srvs/srv/SetBool` | Switch the motor drivers on or off. Instant yes/no answer. Also the software stop. | What can I switch or ask instantly? |
| ACTION | `/vespr/legs/grasp` (and `release`) | `vespr_interfaces/action/Grasp` | Close until the branch is gripped. Takes seconds, reports progress, can be cancelled. | What do I ask it to do that takes time? |

The N20 motors do not speak ROS. The ESP32 does the electrical work, and `legs_bridge` is the only program that talks to it.

## Custom types

Only three types need a definition of our own, and all three live in one package, `vespr_interfaces`:

- `PoseError` (msg): evaluator output.
- `Grasp` (action): legs grasp / release.
- `Perch` (action): the whole perch. "Go perch on that branch" takes time, reports its phase, and must be cancellable.
