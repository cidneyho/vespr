# Fly the sim with the RadioMaster TX16S

This guide covers flying the Gazebo sim by hand with the TX16S Mk3. The radio plugs into the host computer by USB and acts as a joystick. QGroundControl (QGC) reads the joystick and sends the stick positions to the simulated PX4 over MAVLink.

```mermaid
flowchart LR
  R["TX16S<br/>(USB Joystick)"] -- USB --> Q["QGC on the host<br/>(Joystick page)"] -- MAVLink --> P["PX4 SITL + Gazebo"]
```

## Where settings live

| Where | What | Shared with the real drone? |
|---|---|---|
| Radio: Inputs, Mixes, Outputs | Which switch drives which channel, rates, endpoints | **Yes.** Don't change these for the sim. |
| Radio: Model setup → USB Joystick | How the channels appear to the computer | No, this only affects USB |
| QGC Joystick page (saved on the host) | Calibration, and what each switch does | No |
| Pixhawk Radio / Flight Modes pages | RC calibration and switches for the real drone | No, the sim doesn't use them |

To make the sim fly like the real drone, set QGC's button actions to match the switches on the real drone's **Flight Modes** page. There is no way to copy them over automatically.

## Safety: turn off the radio's transmitter

While the radio is plugged in for the sim, its sticks still go to the real drone if the transmitter is on.

- Before you plug in: **Model setup → Internal RF → OFF** (and External RF, if a module is fitted), and **unplug the real drone's battery**.
- Before you fly the real drone: turn **Internal RF back ON** and check the RC link in QGC.

## One-time setup

### 1. Radio: USB Joystick

Go to **MDL → Model setup → USB Joystick → Mode: Advanced**.

| Channel | Set to |
|---|---|
| CH1 | Axis **X** |
| CH2 | Axis **Y** |
| CH3 (throttle) | Axis **rotX** |
| CH4 | Axis **rotY** |
| Switches | **Button**, button mode **Normal**. Set Positions to 2POS or 3POS to match the switch, or Push for a momentary switch. |
| Sliders (LS/RS), tilt (TltX/TltY) | **None**, or one free axis each (Slider, Dial, Wheel). Never reuse a stick's axis. |

- **Btn no.** must not overlap: a 3POS switch uses its number and the next two. Once QGC is set up, don't change these numbers, or QGC's assignments will point at the wrong switches.
- Don't use Classic mode. In Classic mode the throttle reaches QGC with only half its range, and calibration gets stuck at "move the Throttle stick all the way up".

### 2. QGC: Joystick page

Plug in the radio, choose **USB Joystick (HID)**, start the sim, then open QGC. Go to **Q → Vehicle Setup → Joystick**.

1. **Joystick dropdown:** pick *OpenTX TX16SMK3 Joystick*. Choose **Mode 2** if the throttle is on the left stick.
2. **Calibrate:** keep the radio flat. For the centre step, put the throttle in the **middle** by hand. Hold each stick at its end until QGC moves on. At "write the new parameters to your board", click **Next**. QGC saves the calibration on the host.
3. **Settings tab:** turn **Center stick is zero throttle** off, so that throttle down means zero.
4. **Buttons tab:** flip each switch, note which button number lights up, and pick an action for it. Our setup:

| Switch | Action |
|---|---|
| SC (3-position, buttons 2/3/4) | Stabilized / Altitude / Offboard |
| Arm switch | Arm / Disarm |
| Kill switch | Emergency Stop |

Give each action to only one button. QGC changes the mode at the moment you flip a switch, so the last switch you moved decides the mode.

5. Tick **Enable**.

PX4 already accepts joystick input in the sim (`COM_RC_IN_MODE = 1` by default), so nothing needs to change on the PX4 side.

## Each session

1. In the dev container, start the sim:
   ```bash
   ~/vespr/src/perch_sim/scripts/run_sim.sh perch_rod gz_x500_hook
   # outdoors with GPS:  GPS=1 ~/vespr/src/perch_sim/scripts/run_sim.sh baylands gz_x500_hook
   ```
   If no Gazebo window appears, `DISPLAY` points to an old display. Run `export DISPLAY=:4` (or `:5`) and start the sim again.
2. On the host, start QGC: `~/Downloads/QGroundControl-x86_64.AppImage`
3. Check that Internal RF is OFF, plug in the radio, and choose **USB Joystick (HID)**. QGC loads the saved joystick setup.

## Fly

1. Set SC to **Altitude** and put the throttle **down**, then **Arm**.
2. Take off within **10 s**. Otherwise PX4 disarms itself (`COM_DISARM_PRFLT`).
3. Throttle above the middle climbs, the middle holds height, and below the middle descends.
4. To land, hold the throttle low until the drone is down. PX4 disarms automatically.

- **Stabilized:** the throttle sets the power directly. The sim hook drone needs about **76 %** throttle to hover (`MPC_THR_HOVER`), so raise the throttle as you switch into Stabilized.
- **Offboard:** only for the perch mission (`sim_mission.sh`). With no program sending commands, PX4 refuses Offboard.
- **Kill:** stops the motors immediately. It has been tested on the ground and during the mission.

## Known issue: joystick dropouts

PX4 keeps reporting `Manual control lost` / `regained after 0.7 s`. QGC sends the sticks only about 5 times a second, and some gaps are longer than PX4's 0.5 s limit (`COM_RC_LOSS_T`). After a dropout PX4 failsafes: it returns home with GPS on, or lands without GPS. In testing this ended manual flights after about 30 s. The cause hasn't been found yet. It's likely the same problem behind the 2026-11-02 failsafe noted in [flight_accuracy.md](../../docs/design/flight_accuracy.md).

Before running the flight accuracy eval, untick **Enable** in QGC or unplug the radio.
