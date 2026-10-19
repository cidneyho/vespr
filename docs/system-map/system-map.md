# The system map

Board box 06. See the [index](README.md). Link details (cables, protocols) are in [wiring.md](wiring.md).

On the board, every box is a sticky coloured by owner and every arrow is one connection coloured by chain. Flight and legs are the only chains that reach an actuator. Mocap dead-ends at the evaluator. The radio bypasses everything.

## Diagram

```mermaid
flowchart LR
  subgraph GM["Ground · measurement"]
    OPTI["OptiTrack Prime 13W ×10<br/>850 nm IR · 120 Hz · hw-synced<br/>2D blob centroids only"]
    MOCAP["Mocap PC (mocap repo)<br/>calibration · triangulation · rigid bodies<br/>/mocap/vespr/pose · /mocap/branch/pose"]
    CLOCK["Shared clock (chrony / PTP)"]
    ANALYSIS["Analysis (offline)"]
  end
  subgraph GH["Ground · humans"]
    DEV["Dev laptop"]
    QGC["Ground station (QGroundControl)"]
    TX["Pilot radio TX16S"]
  end
  subgraph JET["Aircraft · Jetson Orin Nano (ROS 2 Jazzy)"]
    RS["RealSense D435i"]
    RSD["realsense_driver<br/>/vespr/camera/*"]
    subgraph PERC["perception (John)"]
      BD["branch_detector"]
      VIO["vio"]
    end
    MISSION["mission (state machine)<br/>→ /vespr/mission/state"]
    EVAL["evaluator<br/>→ /vespr/eval/* · no path to flight"]
    BAG["bag_recorder<br/>subscribes to everything"]
    XRCE["micro_xrce_agent<br/>/fmu/in/* · /fmu/out/*"]
    LB["legs_bridge<br/>serial ↔ ROS"]
  end
  subgraph PXH["Aircraft · Pixhawk 6C"]
    PX4["PX4<br/>EKF · offboard · failsafes<br/>only thing that commands a propeller"]
    XR1["XR1 receiver<br/>ELRS in, SBUS out"]
    ESC["ESCs → 4 motors<br/>PWM / DShot · 10×4.5 props"]
    IMU["IMU + baro"]
    FLOW["Optical flow"]
    RF["Rangefinder (I2C)"]
    GPS["GPS M10 (idle indoors)"]
  end
  subgraph LEGS["Aircraft · legs"]
    ESP["ESP32 leg controller<br/>PWM · encoders · stall cutoff"]
    DRV["DRV8833 ×2"]
    N20["N20 motors ×2–4<br/>quadrature encoders"]
    GRIP["Gripped sensor"]
    CLAW["Handcuff claw<br/>non-backdrivable"]
  end

  RS -- "USB 3" --> RSD
  RSD -- "camera topics" --> PERC
  BD -- "branch_pose" --> MISSION
  VIO -- "odom" --> MISSION
  MISSION <-->|"setpoints + heartbeat / PX4 state"| XRCE
  XRCE <-->|"TELEM2 serial · uXRCE-DDS"| PX4
  MISSION <-->|"grasp, release / legs/state"| LB
  LB <-->|"USB serial (or micro-ROS)"| ESP
  PX4 --> ESC
  IMU --> PX4
  FLOW --> PX4
  RF --> PX4
  GPS --> PX4
  ESP --> DRV -- "motor leads" --> N20 -- "lead screw" --> CLAW
  N20 -- "encoders" --> ESP
  GRIP -- "gripped?" --> ESP
  OPTI -- "Ethernet + PoE" --> MOCAP
  MOCAP -. "Wi-Fi 5 GHz · DDS · shared clock" .-> EVAL
  PERC -. "for comparison" .-> EVAL
  TX -- "2.4 GHz ELRS" --> XR1 --> PX4
  QGC -. "MAVLink" .-> PX4
  DEV -. "SSH" .-> JET
```

Line style follows the board: solid = flight / legs / override, dashed = measurement and ground.

## Box contents

### Ground · measurement
- **OptiTrack Prime 13W ×10**: 850 nm IR · 120 Hz · hardware-synced · sends 2D blob centroids only.
- **Mocap PC (mocap repo)**: intrinsics · extrinsics · ground plane · triangulation · labeling · rigid bodies. Publishes `/mocap/vespr/pose`, `/mocap/branch/pose`. The only part of that repo that knows about ROS.
- **Shared clock (chrony / PTP)**: serves time to the Jetson over Wi-Fi.
- **Analysis (offline)**: in = rosbags + mocap sessions + PX4 .ulg. Out = accuracy, success rate, failure taxonomy, energy saved, endurance.

### Ground · humans
- **Dev laptop**: run and debug on the Jetson.
- **Ground station (QGroundControl)**: config · telemetry · arm · pull .ulg logs.
- **Pilot radio TX16S**: override switch.

### Aircraft · Jetson Orin Nano (ROS 2 Jazzy)
- **RealSense D435i**: color · depth · camera IMU.
- **realsense_driver**: `/vespr/camera/{color,depth,imu,info}`.
- **perception (John)**: `branch_detector` → `perception/branch_pose`; `vio` → `perception/odom`.
- **mission (state machine)** (Cidney): reads branch_pose, odom, PX4 state, rangefinder, legs/state. Sends setpoints + heartbeat, arm / disarm. Calls grasp, release. Publishes `/vespr/mission/state`. **The only node with arrows to actuators.**
- **evaluator** (Rushav): vision vs mocap, relative pose first → `/vespr/eval/*`. No path to flight.
- **bag_recorder** (Dom): subscribes to everything → rosbag.
- **micro_xrce_agent**: `/fmu/out/*`, `/fmu/in/*`.
- **legs_bridge**: serial ↔ ROS translator. grasp / release actions, `/vespr/legs/state`.

### Aircraft · Pixhawk 6C (PX4)
- **PX4**: EKF on IMU + flow + rangefinder (+VIO later). Offboard mode · failsafes · pre-arm checks. Only thing that ever commands a propeller. Publishes best effort (QoS trap).
- **XR1 receiver**: ELRS in, SBUS out.
- **ESCs → 4 motors**: PWM / DShot, 10×4.5 props.
- **Sensors**: IMU + baro (built in), optical flow (indoor hold), rangefinder (I2C, altitude), GPS M10 (idle indoors).

### Aircraft · legs
- **ESP32 (leg controller)**: PWM out, encoder interrupts, stall / end-stop cutoff. Commands: grasp · release · status. Wi-Fi + BT off (2.4 GHz).
- **DRV8833 ×2**: PWM in from ESP32, dual drivers.
- **N20 motors ×2–4**: with quadrature encoders, drive the handcuff latch.
- **Gripped sensor**: is the branch in the claw?
- **Handcuff claw**: top-mounted, closes like a cuff. Non-backdrivable drive, zero power to hold.
