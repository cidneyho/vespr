# Design: fake_vio

Status: implemented (2026-10-23). In the sim, with noise and drift off, it matches mocap seen from the start pose exactly, at 30 Hz.

## Goal

Stand in for the VIO (visual-inertial odometry) node in Gazebo, so the mission can be built and tested on a "vision" drone pose before real VIO exists. Publish where the drone is and how fast it's moving, measured from where it started, using Gazebo's ground truth. Perfect by default, with configurable noise and drift.

## Output

| Topic | Type | Frame | Meaning |
|---|---|---|---|
| `/vespr/perception/odom` | `nav_msgs/msg/Odometry` | `odom`, child `base_link` | The drone body's pose and velocity, from where it started |

- **Stamp:** the Gazebo sim time of the sample, as in `mocap_sim`.
- **No TF.** Only the topic, like `mocap_sim`. Add an `odom` → `base_link` transform later if something needs it.

## The `odom` frame: zero where the drone started

Real VIO only knows how far the drone has moved since it switched on. So does the fake:

- **Origin:** where `base_link` is in the first Gazebo sample the node receives.
- **x:** the way the drone faced at that moment, kept level. **z:** straight up. Real VIO feels gravity through its IMU, so it knows which way is up but not which way is north.
- **Sim reset** (sim time jumps back): the origin is taken again from the next sample, as if VIO restarted.

Example: the drone spawns 2 m from the mocap origin, facing east, and flies 1 m east. Mocap says (3, 0, z) in the room; the fake VIO says (1, 0, z).

## Velocity

Real VIO also measures speed, so the fake does too:

- **Worked out** from the previous published sample and this one: moved 5 cm in 0.1 s is 0.5 m/s. Gazebo's poses are exact, so this is smooth enough at 30 Hz.
- **Given in the body frame** (`base_link`), as ROS's `Odometry` defines: x is forward speed, y is sideways speed, z is up. Turning rate is given the same way.
- **First sample** after start or a reset: velocity 0.

## Noise and drift

All off by default.

| Effect | What it does | Settings |
|---|---|---|
| Noise | Random jitter on each sample, different every time | `position_noise`, `orientation_noise`, `linear_velocity_noise`, `angular_velocity_noise` |
| Position drift | Error that builds up the further the drone flies, like real VIO. A random direction is picked at start; the error grows along it by `position_drift` metres per metre flown | `position_drift` (for example 0.01 = 1 % of distance) |
| Yaw drift | The heading slowly turns away from the truth at a steady rate, in a random direction picked at start. Because the whole frame turns, position errors also grow with distance from the start, as on the real drone | `yaw_drift` (degrees per minute) |
| Latency, dropped samples | As in `mocap_sim` | `latency`, `drop_rate` |

Drift is "the answer is getting more wrong over time"; noise is "each answer is a bit off". Real VIO has both.

**How much to trust it (covariance).** `Odometry` has a field where the sender says how uncertain each number is. The fake fills it from the noise settings (noise² on the diagonal), with a tiny floor so perfect output isn't read as "unknown". Drift isn't included, since real VIO doesn't know its own drift either.

## Rate

At most `rate` samples a second (default 30, the camera's frame rate), with the shared rate limit: never above `rate`, never a sample twice. Gazebo's ~50 Hz pose rate is above 30, so 30 Hz is reachable.

## Parameters

| Name | Default | Meaning |
|---|---|---|
| `world` | `perch_rod` | Gazebo world name |
| `drone_model`, `drone_link` | `x500_hook_0`, `base_link` | Which entity is the drone body |
| `rate` | 30.0 | Maximum publish rate, Hz |
| `position_noise`, `orientation_noise` | 0.0, 0.0 | Gaussian noise, m and rad |
| `linear_velocity_noise`, `angular_velocity_noise` | 0.0, 0.0 | Gaussian noise, m/s and rad/s |
| `position_drift` | 0.0 | Drift, m per m flown |
| `yaw_drift` | 0.0 | Drift, degrees per minute |
| `seed` | 0 | Random seed; 0 means a new one each run. Set it to repeat a run's noise and drift exactly |
| `latency` | 0.0 | Delay before publishing, s |
| `drop_rate` | 0.0 | Chance (0 to 1) that a sample is dropped |

## Package: `perch_sim`

| File | Contents |
|---|---|
| `perch_sim/fake_vio.py` | The node, named `vio` so the graph matches the real VIO's |
| `perch_sim/sim_sensor.py` | Gains the `seed` parameter. The branch reading stays; the fake VIO just doesn't use it |
| `perch_sim/poses.py` | Gains whatever velocity and yaw helpers the node needs |
| `launch/fake_vio.launch.py` | The node, every parameter as a launch argument |

## Testing

- Unit: the start pose becomes the origin; level and facing the start heading; velocity from two poses, in the body frame; drift grows as set; a sim reset re-zeroes.
- Node test with fake Gazebo poses: the published message, the rate limit.
- Manual, in the sim: fly a short path and compare `/vespr/perception/odom` with mocap, after subtracting the start pose. They match with drift and noise off.

## For the team

**Mocap and VIO don't match yet.** The plan says moving from stage 1 (fly on mocap) to stage 3 (fly on vision) is one line in a launch file. That needs the two to send the same kind of message. Today they don't:

| | Mocap | VIO |
|---|---|---|
| Type | `PoseStamped`: pose only | `Odometry`: pose and velocity |
| Zero | the room (L-frame) | where the drone started |

- **Type:** easy to fix by translating. Either the mission accepts both, or a small node turns one into the other. The swap stays one line.
- **Zero:** the numbers themselves differ. The swap stays easy only if the mission never uses a fixed room position, for example "the branch is at (3, 0, 1.5)". It should take the branch from the detector, relative to the drone, which is the same either way. Worth agreeing on before the mission loop is written.

## Out of scope

- Sending VIO to PX4 (`/fmu/in/vehicle_visual_odometry`), so PX4's EKF fuses it in stage 3. Needs a team discussion first, and is a separate converter node.
- VIO losing tracking (fast motion, a blank wall). Can be added later as an on/off switch.
- Real images and a real VIO.
