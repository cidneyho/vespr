# Design: optical flow and rangefinder on the sim drone

Status: implemented and tested in sim (2026-10-23).

## Goal

Make the sim hook drone fly the way the real one will indoors: on optical flow and a downward rangefinder, with GPS off. Keep a one-flag switch to turn GPS back on, for outdoor tests in sim later.

## What changed

| Piece | Change |
|---|---|
| `x500_hook` model | Gains a downward optical flow sensor and a downward rangefinder, mounted as on PX4's `x500_flow`. Built by `make_hook_model.py` from `perch_sim/config/x500_hook.yaml` |
| Airframe | Our own PX4 airframe, `airframes/4920_gz_x500_hook`: PX4's X500 (4001), GPS off unless `GPS=1`, no ground station needed |
| `run_sim.sh` | Links our airframes into PX4 and starts `x500_hook` with 4920. New `GPS` option, off by default |
| Perch worlds | The plain floor is now `perch_floor`, a textured floor that flow can track. Built by `make_floor.py` |
| `Commander` | Waits only for a position estimate before takeoff, and judges PX4's arming checks once in offboard mode (separate commit, `998250b`) |
| Our nodes | None. PX4 SITL reads the sensors itself |

## The sensors

Both copy PX4's `x500_flow`, so PX4's simulation bridge finds them by the names it expects.

| Sensor | Gazebo | Link (name PX4 expects) | Mount from `base_link` | Rate |
|---|---|---|---|---|
| Optical flow | PX4's `optical_flow` model: a 100 × 100 px downward camera, 42° view, plus PX4's flow sensor | `flow_link` | 3 cm forward, 10 cm down, looking down | 50 Hz |
| Rangefinder | A single-beam lidar, 0.1 to 100 m, like PX4's LW20 setup | `lidar_sensor_link`, sensor `lidar` | 5 cm down, looking down | 50 Hz |

- The mounts are placeholders, in the `drone.flow` and `drone.rangefinder` blocks of `perch_sim/config/x500_hook.yaml`, like the camera's. Replace them with the real positions when known.
- Both sit under the body. The hook is on top and the camera in front, so nothing blocks them.

## Where the readings appear in ROS

PX4 keeps the raw sensor readings to itself. We read its estimate in `/fmu/out/vehicle_local_position` (option A):
- `dist_bottom`: height above the ground under the drone, in metres.
- `dist_bottom_valid`: whether PX4 trusts it.

`Commander` already subscribes to this topic. The system map's `/fmu/out/distance_sensor` doesn't exist in stock PX4 v1.16 (it's an input only), so the system map needs updating to match.

## Airframe and GPS option

```
run_sim.sh perch_rod gz_x500_hook           # indoor: flow + rangefinder, GPS off
GPS=1 run_sim.sh perch_rod gz_x500_hook     # outdoor: GPS + flow + rangefinder
```

`4920_gz_x500_hook` runs PX4's 4001, then sets these defaults:

| Parameter | GPS off (default) | `GPS=1` |
|---|---|---|
| `SYS_HAS_GPS` | 0 | 1 (PX4 default) |
| `SIM_GPS_USED` | 0 | 10 (PX4 default) |
| `EKF2_GPS_CTRL` | 0 | 7 (PX4 default) |
| `NAV_DLL_ACT` | 0 | 0 |

`run_sim.sh` picks the airframe by name: a file in `airframes/` ending in `_<vehicle>`. Other vehicles still get 4001.

**Why our own airframe, not 4021 plus environment parameters.** The first plan set these through `PX4_PARAM_*` environment variables. That fails whenever the wanted value equals PX4's default: PX4 doesn't save a value equal to the default, so the airframe's own `param set-default` then wins. `NAV_DLL_ACT 0` was overridden by 4001's default of 2, and `GPS=1` would have failed the same way. An airframe file runs after 4001, so its `set-default`s always win. It also means the README's `param set NAV_DLL_ACT 0` step isn't needed for the hook drone.

Changing airframe makes PX4 reset its saved parameters, once.

## Textured floor

The perch worlds had a plain grey floor. Flow over a featureless floor sees no motion, and still reports high quality. PX4 then believes the drone is still while it drifts:
- Flow only: PX4's position held within 1 cm while the drone truly drifted up to 17 m (one later test, 210 m).
- GPS + flow: GPS and flow disagreed, and the estimate swung 2.7 m.
- The same flow-only test in PX4's `baylands` world (textured ground) drifted only about 0.5 m.

`perch_floor` replaces the floor in both worlds: a 200 m square tiled every 1 m with a seamless grey speckle, detail from 1 to 20 cm. The collision plane is unchanged.

**For the real drone:** the same applies. Flow needs a textured floor under the flight area, or the drone will drift unseen. The team should check the lab floor.

## Commander readiness

With GPS off, `Commander` never became ready: it waited for PX4's `pre_flight_checks_pass`, which PX4 judges for its current mode. After boot that is Hold, which needs GPS. Offboard needs only a local position.

`Commander` now waits for a position estimate, switches to offboard, and only then arms if `pre_flight_checks_pass` is true. If the checks still fail in offboard, it doesn't arm and logs why. Committed separately (`998250b`).

## Test results

In `perch_rod` with the textured floor, taking off to 1.5 m and hovering 25 s. "True" is Gazebo's ground truth from the PX4 log, not PX4's estimate.

| Test | Fusing | True drift in hover | Estimate − truth |
|---|---|---|---|
| GPS off | flow, rangefinder | 1 cm | 9 cm offset, sd 0 |
| `GPS=1` | GPS, flow, rangefinder | 5 cm | 2 cm, sd 1 cm |
| `GPS=1`, flow fusion off | GPS, rangefinder | 9 cm | 1 cm, sd 2 cm |

The offset with GPS off is a constant error from where PX4 started its frame on the ground, not drift.

Also checked:
- `dist_bottom` tracks the height (0.18 m on the ground, about 1.5 m in hover), and `dist_bottom_valid` is true throughout.
- Starting again without `GPS=1` after a `GPS=1` run really has GPS off (`SYS_HAS_GPS`, `SIM_GPS_USED`, `EKF2_GPS_CTRL` all 0).
- With `COM_PARACHUTE=1` forcing a failed check, `Commander` enters offboard, doesn't arm, and logs the reason.
- `mocap_sim` and `fake_branch_detector` work with the new model and floor. In a hover the detector's branch pose matches one worked out from mocap to 1 mm. Mocap still publishes at about 50 Hz, Gazebo's pose rate (open question 1 in `mocap_sim.md`).

## Risks

- **Drift.** With GPS off, PX4's position slowly drifts, like the real drone's indoors. `goto` targets are in PX4's frame, so they drift with it. Yaw also drifts a few degrees (4° seen), since only the magnetometer fixes it.
- **Sim flow is easier than real flow.** The sim floor is evenly lit, high contrast and flat.

## Out of scope

- The camera IMU and payload mass (later).
- Outdoor worlds, wind and GPS degradation (later steps of the outdoor plan).
- An upward sensor for the branch during the final rise (a team question).
