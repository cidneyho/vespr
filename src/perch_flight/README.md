# perch_flight

Flight control nodes. They command PX4 over ROS 2 through the `/fmu/in/*` and `/fmu/out/*` topics (see [docs/interfaces.md](../../docs/interfaces.md)). The terms used here are explained in [perch_sim](../perch_sim/README.md#terms).

## Test flight: fly_square

`fly_square` is a ROS 2 node that flies a 5 m square at 5 m altitude and lands. It talks to PX4 the same way our real flight code will, so if it works, the whole chain works: sim → Agent → ROS 2 → our code.

1. Terminal 1, start the sim (see [perch_sim](../perch_sim/README.md)) and wait for `Ready for takeoff!`.
2. Terminal 2, start the bridge between PX4 and ROS 2:
   ```bash
   MicroXRCEAgent udp4 -p 8888
   ```
   It prints `session established` once PX4 connects.
3. Terminal 3, build and run the node:
   ```bash
   cd ~/vespr && colcon build --symlink-install && source install/setup.bash
   ros2 run perch_flight fly_square
   ```

Edit `WAYPOINTS` in [perch_flight/fly_square.py](perch_flight/fly_square.py) to change the path. With `--symlink-install` the edit takes effect on the next `ros2 run`, no rebuild needed.

How the node flies the drone:
- Every 0.1 s it publishes the target position to `/fmu/in/trajectory_setpoint`, plus a heartbeat on `/fmu/in/offboard_control_mode`. If these stop for about a second, PX4 assumes the program has crashed and triggers a failsafe (set by `COM_OF_LOSS_T` and `COM_OBL_RC_ACT`).
- It switches PX4 to offboard mode and arms by publishing to `/fmu/in/vehicle_command`.
- It reads the drone's position from `/fmu/out/vehicle_local_position` and its armed state and mode from `/fmu/out/vehicle_status_v1`, and moves to the next waypoint after hovering 2 s at each one.

To see what PX4 is publishing: `ros2 topic list`, then for example `ros2 topic echo /fmu/out/vehicle_local_position`.

## References

- [PX4 ROS 2 offboard control example](https://docs.px4.io/main/en/ros2/offboard_control)
