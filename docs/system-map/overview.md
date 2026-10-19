# Overview: the components and what lives where

Board boxes 01 and 02. See the [index](README.md).

## The five components

| Component | Role | Job |
|---|---|---|
| Flight controller (Pixhawk 6C, PX4) | Reflexes | Keeps the aircraft level and in the air hundreds of times a second. Never sees the camera, never knows what a branch is. It just flies. |
| Companion computer (Jetson Orin Nano) | Brain | Looks at the camera, finds the branch, decides approach / land / grip / shut down. Hands the Pixhawk simple instructions ("move here"). |
| Leg controller (ESP32) | Hands | Drives the N20 motors, counts encoder pulses, stops a motor the instant it stalls. Jetson says *what* ("grasp"); ESP32 handles *how*. |
| Mocap system (OptiTrack + Linux PC) | Referee | Outside the arena. The only thing that knows the true answer. The only thing not on the drone. |
| Pilot radio (TX16S) | Override | A human can take over at any instant. The Pixhawk obeys the radio over the Jetson. |

## The one rule: mocap measures, it never steers

The drone must find the branch and land on it using only what it carries. Mocap records where the drone actually was so you can compare "I think I am 40 cm away" against "you were 43 cm". If mocap ever fed the flight decisions, the project would prove nothing. Everything about how the two sides connect is designed to keep that wall intact.

One deliberate, temporary exception: stage 1 of the mocap phase-out (see [mocap.md](mocap.md#phasing-out-mocap)).

## What lives where

Pixhawk flies · Jetson thinks · ESP32 grips · mocap PC measures · ground station watches · radio overrides.

### Pixhawk 6C (PX4): flight computer, low level
- IMU + barometer, read at hundreds of Hz.
- Four motors via the ESCs. The only thing that ever commands a propeller.
- RC receiver on the SBUS port: the pilot override happens inside the Pixhawk.
- Optical flow + rangefinder: holds position indoors with no Jetson running.
- GPS: idle indoors.
- EKF fuses IMU, flow, rangefinder (and VIO later) into "where am I".
- Accepts position/velocity targets in offboard mode. Silence for a fraction of a second = failsafe.

### Jetson Orin Nano: mission computer, high level (everything we write, in ROS 2)
- RealSense D435i on USB 3: color, depth, camera IMU.
- Perception (John): where is the drone, where is the branch.
- Mission node (Cidney): the state machine, the hub.
- Bridges: micro XRCE agent to the Pixhawk, `legs_bridge` to the ESP32.
- Evaluator: the ONLY node that hears mocap. No output to flight.
- `bag_recorder`: everything to disk.
- Runs Ubuntu 24.04 (JetPack 7.2), which ROS 2 Jazzy needs.

### ESP32 (QT Py S3): leg controller, plain Arduino sketch on USB
- N20 motors through DRV8833 drivers (PWM, 20 kHz).
- Encoder pulses counted in an interrupt.
- Stall / end-of-travel cutoff: the job where timing matters most.
- Gripped sensor: is the branch in the claw?
- Commands from the Jetson: `grasp`, `release`, `status`.
- Wi-Fi + Bluetooth OFF (2.4 GHz, same band as ELRS).
- A wiring mistake costs a $10 board, not the Jetson.

### Mocap PC (Linux): the referee, on the ground, on its own wired network
- 10 OptiTrack Prime 13W over PoE Ethernet.
- Our code: calibration, blob detection, triangulation, rigid bodies.
- Single output: drone pose + branch pose, timestamped.
- Never receives anything from the drone except a clock reference.
- Full pipeline in [mocap.md](mocap.md).

### Ground + humans
- Ground station: QGroundControl, MAVLink. A window into the Pixhawk, not a controller.
- Dev laptop: SSH into the Jetson.
- Pilot radio TX16S: override switch, kill switch. Always beats the Jetson.
- Shared clock (chrony / PTP) so both logs agree on time.

## Why three computers: the timing rule

Fast-and-dumb vs slow-and-smart. Ask "what breaks if this runs late?"

Some jobs must happen on time, every time, or the aircraft falls or a motor drives into its end stop: keeping level, reading the gyro, counting encoder pulses, cutting power on a stall. That is real-time: not fast, but predictable. Linux on the Jetson juggles dozens of processes and cannot promise that, so anything that must run on time lives on a small dedicated chip (Pixhawk, ESP32). Heavy thinking that can be 40 ms late (finding a branch, estimating position, deciding to abort) lives on the Jetson.

The Jetson sends the small chips **intentions, never motor commands**. "Low level / high level" is the right label for the layers; timing is the reason the layers exist.
