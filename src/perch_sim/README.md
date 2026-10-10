# perch_sim

Gazebo simulation of the X500.

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

## Terms

| Term | Meaning |
|---|---|
| PX4 | The autopilot firmware. Same one that runs on our Pixhawk 6C |
| SITL | Software in the loop: PX4 runs as a normal program on your machine, and Gazebo stands in for the real sensors and motors |
| Gazebo | The physics simulator (Gazebo Harmonic, the newer `gz sim`, not Gazebo Classic) |
| `gz_x500` | The simulated Holybro X500, matching our airframe |
| `pxh>` | The PX4 shell that appears in the terminal running SITL |
| QGroundControl (QGC) | Ground station app: map, telemetry, parameters |

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
2. **Fly from code:** command the sim drone from a script (MAVSDK offboard), then a perch-approach script: fly beside the bar, then descend slowly.
3. **Model the perching mechanism** once the design exists: copy the X500 model, add links and joints.
4. **Move control to ROS 2** (`px4_msgs` over the Micro XRCE-DDS Agent, see [docs/interfaces.md](../../docs/interfaces.md)) when flight code becomes a `perch_flight` node.

Known limit: Gazebo is good for flight dynamics, control logic, and sensor data, and weak at contact detail such as a claw gripping bark. Validate the grip on hardware.

## References

- [PX4 Gazebo vehicles](https://docs.px4.io/main/en/sim_gazebo_gz/vehicles)
- [PX4 Gazebo worlds and environment variables](https://docs.px4.io/main/en/sim_gazebo_gz/)
