# perch_sim

Gazebo simulation of the X500, and a test flight that flies it over ROS 2.

## Quick start

1. Open this repo in VS Code and run **Dev Containers: Reopen in Container**.
2. In a VS Code terminal (it is already inside the container), start the sim:
   ```bash
   cd ~/PX4-Autopilot && make px4_sitl gz_x500
   ```
   A Gazebo window opens with the drone, and the terminal shows a `pxh>` prompt.
3. First time only, at the `pxh>` prompt, let PX4 fly without a ground station app:
   ```
   param set NAV_DLL_ACT 0
   ```
4. Wait for `Ready for takeoff!`, then type `commander takeoff`. The drone climbs and hovers. Type `commander land` to bring it down.

To stop the sim, press Ctrl+C in the first terminal.

## Test flight over ROS 2

`fly_square` is a ROS 2 node that flies a 5 m square at 5 m altitude and lands. It talks to PX4 the same way our real flight code will, so if it works, the whole chain works: sim → Agent → ROS 2 → our code.

1. Terminal 1, start the sim as above and wait for `Ready for takeoff!`.
2. Terminal 2, start the bridge between PX4 and ROS 2:
   ```bash
   MicroXRCEAgent udp4 -p 8888
   ```
   It prints `session established` once PX4 connects.
3. Terminal 3, build and run the node:
   ```bash
   cd ~/vespr && colcon build --symlink-install && source install/setup.bash
   ros2 run perch_sim fly_square
   ```

Edit `WAYPOINTS` in [perch_sim/fly_square.py](perch_sim/fly_square.py) to change the path. With `--symlink-install` the edit takes effect on the next `ros2 run`, no rebuild needed.

How the node flies the drone:
- Every 0.1 s it publishes the target position to `/fmu/in/trajectory_setpoint`, plus a heartbeat on `/fmu/in/offboard_control_mode`. If these stop for about a second, PX4 assumes the program has crashed and triggers a failsafe (set by `COM_OF_LOSS_T` and `COM_OBL_RC_ACT`).
- It switches PX4 to offboard mode and arms by publishing to `/fmu/in/vehicle_command`.
- It reads the drone's position from `/fmu/out/vehicle_local_position` and its armed state and mode from `/fmu/out/vehicle_status_v1`, and moves to the next waypoint after hovering 2 s at each one.

To see what PX4 is publishing: `ros2 topic list`, then for example `ros2 topic echo /fmu/out/vehicle_local_position`.

## Terms

| Term | Meaning |
|---|---|
| PX4 | The autopilot firmware. Same one that runs on our Pixhawk 6C |
| SITL | Software in the loop: PX4 runs as a normal program on your machine, and Gazebo stands in for the real sensors and motors |
| Gazebo | The physics simulator (Gazebo Harmonic, the newer `gz sim`, not Gazebo Classic) |
| `gz_x500` | The simulated Holybro X500, matching our airframe |
| `pxh>` | The PX4 shell that appears in the terminal running SITL |
| QGroundControl (QGC) | Ground station app: map, telemetry, parameters |
| ROS 2 | Toolkit our robot code is built on. Programs talk by publishing messages to named channels |
| Node | One ROS 2 program, such as `fly_square` |
| Topic | A named channel nodes publish to and listen on, such as `/fmu/out/vehicle_local_position` |
| Micro XRCE-DDS Agent | Bridge that turns PX4's internal data into ROS 2 topics (`/fmu/out/*`) and ROS 2 topics into PX4 commands (`/fmu/in/*`) |
| Offboard mode | PX4 mode where an external program sends position or velocity setpoints |
| NED | North-East-Down, PX4's local frame. Up is negative `down`, so 5 m altitude is `z = -5` |

## Other vehicles and worlds

```bash
PX4_GZ_WORLD=baylands make px4_sitl gz_x500_depth
```

Vehicles (in `~/PX4-Autopilot/Tools/simulation/gz/models`): `gz_x500_depth` (forward depth camera), `gz_x500_lidar_down` (downward rangefinder), `gz_x500_mono_cam_down` (downward camera), and more.
Worlds (in `~/PX4-Autopilot/Tools/simulation/gz/worlds`): `default`, `baylands`, `walls`, `windy`, `forest`, `lawn`, and more.

Run headless (no Gazebo window): `HEADLESS=1 make px4_sitl gz_x500`.

## QGroundControl (host)

QGC runs on the host, not in the container. The container uses host networking, so QGC finds the simulated drone automatically. Download the x86_64 AppImage from the [QGC install page](https://docs.qgroundcontrol.com/Stable_V5.1/en/qgc-user-guide/getting_started/download_and_install.html). The current AppImage targets Ubuntu 24.04 and later. On 22.04 you need an older release. Prerequisites:

```bash
sudo usermod -aG dialout "$(id -un)"
sudo systemctl mask --now ModemManager.service
sudo apt install -y libfuse2 libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor0
```

## Next steps

1. **Custom world for perch practice:** a ground grid, a few posts at known coordinates, and a horizontal cylinder standing in for a branch. Keep it in this package. PX4 loads `PX4_GZ_WORLD` from `~/PX4-Autopilot/Tools/simulation/gz/worlds`, so the world must be linked there or added to `GZ_SIM_RESOURCE_PATH`. Not tried yet.
2. **Perch-approach node:** fly beside the bar, then descend slowly. Same setpoint pattern as `fly_square`, later velocity setpoints.
3. **Model the perching mechanism** once the design exists: copy the X500 model, add links and joints.
4. **Move flight code to `perch_flight`** once it is more than a test. The topics are listed in [docs/interfaces.md](../../docs/interfaces.md).

Known limit: Gazebo is good for flight dynamics, control logic, and sensor data, and weak at contact detail such as a claw gripping bark. Validate the grip on hardware.

## References

- [PX4 Gazebo vehicles](https://docs.px4.io/main/en/sim_gazebo_gz/vehicles)
- [PX4 Gazebo worlds and environment variables](https://docs.px4.io/main/en/sim_gazebo_gz/)
- [PX4 ROS 2 offboard control example](https://docs.px4.io/main/en/ros2/offboard_control)
