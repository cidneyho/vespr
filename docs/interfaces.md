# Interfaces

Topics, message types and coordinate frames shared between the perception, flight control, simulation and motion capture subsystems.

## Coordinate frames

| Frame | Definition | Used by |
|---|---|---|
| World | TBD (origin, axis convention) | TBD |
| Drone body | TBD | TBD |
| Camera | TBD | TBD |

## Topics

| Interface | Topic | Message type | Publisher → Subscriber | Frame / units / rate |
|---|---|---|---|---|
| Branch location | TBD | TBD | perception → flight control | TBD |
| Camera images | TBD | TBD | camera driver / simulation → perception | TBD |
| Drone pose | TBD (`/fmu/out/vehicle_odometry`) | TBD | PX4 onboard estimate → flight control, perception | TBD |
| Ground truth (validation only) | TBD | TBD | motion capture → logging | TBD |
| PX4 interface | `/fmu/in/*`, `/fmu/out/*` | `px4_msgs` (`release/1.16`) | flight control ↔ PX4 | NED frame, via Micro XRCE-DDS Agent |
