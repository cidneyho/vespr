# Simulation and logging

Board box 10 (power rails are in [wiring.md](wiring.md#power-rails)). See the [index](README.md). Hands-on sim setup: [src/perch_sim/README.md](../../src/perch_sim/README.md).

The mission node cannot tell the difference between the real drone and the simulated one. That is the whole point of the `/fmu` interface.

## Simulation (Dom, with Cidney)

- **PX4 SITL + `gz_x500_depth`**: the whole flight stack runs as a program on a laptop, publishing the same `/fmu/out` and listening on the same `/fmu/in`. The stock depth camera is an OAK-D: edit the sensor block (FOV, resolution, range, noise) to approximate the D435i.
- **Gazebo Harmonic + `ros_gz_bridge`**: physics and rendering; the branch is a cylinder in the world. The bridge carries Gazebo images into ROS on the same `camera/...` names, so John's detector cannot tell they are synthetic. Skip anything that uses gazebo-classic.
- **Legs in sim**: links and joints from the CadQuery files with real mass and inertia, so PX4 tuning reflects the loaded aircraft. A stub `legs_bridge` answers grasp / release with "success" after a delay.
- **What sim is for**: the state machine (every transition and abort, hundreds of times for free), approach trajectories, loop timing, failure injection (wrong branch pose, dropped frames, late heartbeat).
- **What sim is NOT for**: the grasp itself. Contact physics is unreliable; the bench rig covers touchdown onward.

## Logging and analysis (Dom, Rushav secondary)

| Log | Where | Contents |
|---|---|---|
| rosbag (`bag_recorder`) | Jetson NVMe | Every ROS 2 topic: frames, PX4 state, setpoints, leg state, mission state, mocap poses, evaluator output. One bag per flight, named by date and test ID. |
| Mocap session | mocap PC, `~/panoptes-data/sessions` | Raw record in the mocap repo's own format, even when poses were streamed live. |
| PX4 `.ulg` | Pixhawk SD card | The flight controller's internal view at high rate: IMU, EKF state, motor outputs. The log to open when the question is "why did it wobble". QGroundControl pulls it after a flight. |

## The five numbers

Everything else exists to produce these:

1. **Position accuracy**: vision vs mocap, relative, by distance to branch.
2. **Success rate** per condition.
3. **Failure taxonomy**: fixed short list.
4. **Energy**: current in state 9 vs state 2 (see [perch-cycle.md](perch-cycle.md)).
5. **Endurance**: how long state 9 lasted.
