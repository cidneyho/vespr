# VESPR

A perching drone.

Mocap lives in its own repo.

## Project structure

```
src/
  perch_perception/
  perch_flight/
  perch_sim/
  perch_bringup/
docs/interfaces.md      topics shared between packages
px4_config/             exported PX4 parameter files
.devcontainer/          Docker dev environment
```

## Versions

| Component | Version |
|---|---|
| Orin | JetPack 7.2 (Ubuntu 24.04) |
| ROS 2 | Jazzy |
| PX4 (SITL and Pixhawk) | v1.16.0 |
| px4_msgs | `release/1.16` (must match PX4) |
| Micro XRCE-DDS Agent | v2.4.3 |
| Gazebo | Harmonic |

## Dev environment

Requirements: Docker, the NVIDIA Container Toolkit, and VS Code with the Dev Containers extension.

1. Allow the container to open windows on your screen (once per login): `xhost +local:`
2. Open this folder in VS Code → "Reopen in Container". The first build takes a while (it compiles PX4 SITL and px4_msgs). After that it runs `rosdep install` and `colcon build`.

Inside the container:

| What | Where |
|---|---|
| this repo | `~/vespr` |
| PX4 | `~/PX4-Autopilot` |
| px4_msgs (`release/1.16`) | `~/px4_ws`, already built and sourced |

ROS_DOMAIN_ID set to 16 to avoid interference with other teams. To talk to the drone or mocap: export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

Check PX4 SITL and the ROS 2 bridge work

```bash
# terminal 1: Gazebo window should open
cd ~/PX4-Autopilot && make px4_sitl gz_x500
```

```bash
# terminal 2
MicroXRCEAgent udp4 -p 8888
```

```bash
# terminal 3: should show /fmu/out/*
ros2 topic list
```

Rebuild after pulling changes: `colcon build --symlink-install && source install/setup.bash`.
