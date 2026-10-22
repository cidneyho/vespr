# Design: mocap_sim

Status: draft for review. Nothing is implemented yet.

## Goal

Stand in for the mocap system in Gazebo. Publish the drone and branch poses on the same topics, with the same type and frame, as the real mocap node, using Gazebo's ground truth. Perfect by default, with configurable noise.

## Output

| Topic | Type | Frame | Pose of |
|---|---|---|---|
| `/mocap/vespr/pose` | `geometry_msgs/msg/PoseStamped` | `mocap` | The drone's `base_link` (body centre) |
| `/mocap/branch/pose` | `geometry_msgs/msg/PoseStamped` | `mocap` | The branch model's `perch_point` (below) |

- **`mocap` frame** = Gazebo's world frame: x forward, y left, z up, origin at the world origin.
- **Stamps** are the Gazebo sim time of the sample, so the evaluator lines poses up correctly.

The real mocap rigid bodies should be defined at the same points (drone body centre, the perch point on the branch's centreline), so sim and real mean the same thing.

## The perch point

Each branch model gets an invisible link named `perch_point` (no collision, no visual), marking where to perch. Gazebo publishes every link's pose, so `mocap_sim` works in any world without knowing the branch's shape.

`perch_point` sits on the branch's **centreline**. Its x axis runs along the branch, y is horizontal and z is perpendicular to both, pointing up.

| Model | `perch_point` in the model | In the world | Change |
|---|---|---|---|
| `perch_rod` | (0, 0, 1.5), yaw 90°: rod centre, x along the rod | (3, 0, 1.5) | Edit `model.sdf` |
| `perch_branch` | (1.384, 0, 1.802), pitched up 4.2°: middle of the perch zone, x along the limb away from the trunk | (3.62, 0, 1.80), x pointing along −x | Add to `scripts/make_branch_mesh.py`, then regenerate |

`perch_branch`'s point comes from the script's own limb geometry: the middle of `PERCH_ZONE`. The script also gets a comment noting that the point is on the centreline; the top surface is 2.2 cm above it there.

## Source: Gazebo ground truth

| Gazebo topic | Used for | Notes |
|---|---|---|
| `/world/<world>/dynamic_pose/info` | Drone | Model pose in the world, `base_link` relative to the model. The node combines them |
| `/world/<world>/pose/info` | Branch | Branch models are static, so they only appear here. Combine the model pose and `perch_point`. Read once at startup, then the node unsubscribes |

The node reads both **directly through gz-transport** (Gazebo's Python bindings), not through `ros_gz_bridge`: the bridge's `TFMessage` conversion drops the entity names. Gazebo doesn't say which model a link belongs to, so links are found by name, and the node refuses a name that appears more than once.

## Rate

- Gazebo publishes poses at about 50 Hz, below the 120 Hz we want.
- The node publishes **only when a new Gazebo sample arrives**, at most `rate` per second. It never repeats an old sample with a new stamp, since that would fake data the evaluator trusts.
- So until the source rate is raised, the output runs at about 50 Hz whatever `rate` is set to. Raising Gazebo's pose rate in our perch worlds is a follow-up (open question 1).

## Parameters

| Name | Default | Meaning |
|---|---|---|
| `world` | `perch_rod` | Gazebo world name, for the source topics (launch argument) |
| `drone_model`, `drone_link` | `x500_hook_0`, `base_link` | Which entity is the drone |
| `branch_model` | `perch_rod` | Which model is the branch. Its `perch_point` link is used. The launch file sets it from the world (`perch_rod` or `perch_branch`) |
| `rate` | 120.0 | Maximum publish rate, Hz |
| `position_noise` | 0.0 | Gaussian noise on position, standard deviation in m |
| `orientation_noise` | 0.0 | Gaussian noise on each rotation axis, standard deviation in rad |
| `latency` | 0.0 | Delay before publishing, s. The stamp stays the sample time |
| `drop_rate` | 0.0 | Chance (0 to 1) that a sample is dropped |

The drone and branch get independent noise. A dropped sample is dropped for both bodies, as when a real mocap frame is lost.

## Package: `vespr_sim`

| File | Contents |
|---|---|
| `vespr_sim/mocap_sim.py` | The node, named `mocap_sim` |
| `vespr_sim/poses.py` | Pure helpers: combine poses, add noise |
| `launch/mocap_sim.launch.py` | The node, with every parameter above as a launch argument |
| `test/` | Unit tests for `poses.py`, and a node test with fake Gazebo poses |

No bridge and no `/clock`: stamps come from the Gazebo messages, and `latency` is wall time.

## Testing

- Unit: pose combination, noise off gives exact poses, the rate limit, and drop_rate 1 publishes nothing.
- Node test against fake Gazebo pose messages, as with the legs.
- Manual, in both worlds: `ros2 topic echo /mocap/branch/pose` shows the perch point from the table above. `ros2 topic echo /mocap/vespr/pose` while flying with `flight_cli` tracks the drone.

## Out of scope

- The real mocap node (mocap repo).

## Open questions

1. **Real 120 Hz:** raise Gazebo's pose publish rate in our perch worlds now, or accept about 50 Hz until it matters?
