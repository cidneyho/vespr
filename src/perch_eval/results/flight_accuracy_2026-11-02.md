# Flight accuracy eval: sim, 10 runs (2026-11-02)

How far the drone strays from its setpoints, split into PX4's estimate (estimation) and PX4 following its own estimate (control). Method: [docs/design/flight_accuracy.md](../../../docs/design/flight_accuracy.md). This page reports the measurements only. Causes and fixes: [flight_accuracy_fixes_2026-11-05.md](flight_accuracy_fixes_2026-11-05.md). These numbers were taken with the sim problems found there in place.

| File | What it is |
|---|---|
| [`paths.png`](flight_accuracy_2026-11-02/paths.png) | all 10 actual paths over the preferred one |
| [`run_01.png`](flight_accuracy_2026-11-02/run_01.png) … [`run_10.png`](flight_accuracy_2026-11-02/run_10.png) | one run each: path, altitude, error sizes over time |
| [`summary.md`](flight_accuracy_2026-11-02/summary.md) | every table the analysis printed |
| [`flight_accuracy.json`](flight_accuracy_2026-11-02/flight_accuracy.json) | every number, per run and pooled |

**Raw data (not in git):** `~/vespr_recordings/flight_accuracy/2026-11-02T14-33-22/` (one bag and the logs per run).

## Setup

| | |
|---|---|
| Sim | PX4 SITL + Gazebo, world `perch_rod`, vehicle `gz_x500_hook`, GPS off (optical flow + rangefinder), a fresh sim each run |
| Truth | `mocap_sim`, no noise |
| Path | `perch_eval flight_path`: take off to 1 m; north, east, turn +90°, south, west (1 m each); up and down 0.5 m; turn −90°. 0.3 m/s, 5 s hold after each step |
| Code | branch `feat/legs-sim-bridge` at 8b1d5f1, with the eval code uncommitted |
| Runs | 10 flown, 10 analysed, none skipped. Path 67.7 to 67.9 s each |
| Alignment | mocap to PX4 heading offset 91.0° to 91.6° across the runs |

## Path error

Pooled over all 10 runs, in cm (heading in degrees). Total = actual − preferred, estimation = actual − estimate, control = estimate − preferred. Along: + ahead of the setpoint. Altitude: + above.

| While | Part | Total mean / max | Estimation mean / max | Control mean / max |
|---|---|---|---|---|
| Moving | along | −8.7 / 32.7 | −2.6 / 22.1 | −6.1 / 18.5 |
| Moving | off the line | 12.8 / 27.4 | 11.8 / 24.9 | 1.9 / 9.9 |
| Moving | altitude | −5.1 / 14.3 | −4.9 / 7.3 | −0.3 / 9.8 |
| Moving | heading (deg) | −3.2 / 9.4 | −3.2 / 9.5 | 0.0 / 4.4 |
| Holding | horizontal | 15.0 / 35.6 | 14.1 / 33.2 | 4.3 / 20.8 |
| Holding | altitude | −4.6 / 14.5 | −4.9 / 7.2 | 0.3 / 11.3 |
| Holding | heading (deg) | −4.6 / 14.7 | −4.6 / 9.7 | 0.1 / 10.2 |
| Turning | horizontal | 11.9 / 28.0 | 12.5 / 26.4 | 4.5 / 12.5 |
| Turning | heading (deg) | −4.9 / 11.1 | −4.9 / 9.9 | −0.1 / 10.1 |

(max = largest size either way.)

## Settling

Within 5 cm of the setpoint, after each step. Mean / max over the 10 runs.

| # | After | Actual: settled | Actual: error at end of hold (cm) | Estimate: settled | Estimate: settle time (s) | Estimate: overshoot |
|---|---|---|---|---|---|---|
| 0 | takeoff (up 0.9 m) | 0 of 10 | 16.6 / 18.5 | 1 of 10 | 4.74 | 0 cm |
| 1 | north 1 m | 0 of 10 | 10.8 / 12.8 | 10 of 10 | 2.00 / 2.12 | 17.3 / 20.4 cm |
| 2 | east 1 m | 0 of 10 | 8.9 / 10.6 | 10 of 10 | 1.98 / 2.04 | 17.4 / 19.6 cm |
| 3 | turn +90 | 0 of 10 | 24.4 / 31.8 | 10 of 10 | 2.27 / 2.34 | 7.8 / 9.6° |
| 4 | south 1 m | 0 of 10 | 21.7 / 27.4 | 10 of 10 | 1.84 / 1.90 | 16.4 / 20.7 cm |
| 5 | west 1 m | 0 of 10 | 13.6 / 18.7 | 10 of 10 | 2.01 / 2.06 | 17.7 / 19.8 cm |
| 6 | up 0.5 m | 0 of 10 | 17.6 / 23.4 | 10 of 10 | 0.35 / 0.92 | 5.6 / 11.3 cm |
| 7 | down 0.5 m | 0 of 10 | 15.2 / 20.3 | 10 of 10 | 0.36 / 0.66 | 5.2 / 8.1 cm |
| 8 | turn −90 | 1 of 10 | 10.6 / 20.9 | 10 of 10 | 2.09 / 2.30 | 7.5 / 10.2° |

Wobble while holding (RMS about its own mean, second half of the hold): 0.9 to 1.3 cm for the actual position after every step except the takeoff (5.9 cm). The actual position's overshoot is in [`summary.md`](flight_accuracy_2026-11-02/summary.md).

## What the data shows

- **The actual position never settled within 5 cm** in 89 of 90 holds. It ended its holds 9 to 24 cm (mean per step) from the setpoint.
- **PX4's estimate settled within 5 cm after every step** in all 10 runs, except the takeoff. That took about 2.0 s after the 1 m moves and turns, and about 0.35 s after the 0.5 m climbs and descents. The mission currently waits `settle_time` 0.5 s.
- **Most of the total error is estimation error.** Off the line while moving: 11.8 of 12.8 cm. Horizontal while holding: 14.1 of 15.0 cm. Control error is mostly in along-track while moving (−6.1 cm mean, behind the setpoint) and in overshoot when stopping (16 to 18 cm after each 1 m move).
- **The heading error is all estimation:** about −3 to −5° mean, up to 9 to 10°. Control heading error averages 0°.
- **Altitude:** the actual drone held about 5 cm below the setpoint for the whole flight (estimation −4.9 cm, control +0.3 cm). See the altitude panel in `paths.png`.
- **Takeoff:** the setpoint climbs to 1 m in about 3 s. The drone reaches about 0.95 m roughly 10 s after the climb starts, in every run.
- **The paths repeat closely across runs:** the same shape, offset and corner loops in all 10 (`paths.png`). In run 1 the estimation error grows from about 5 cm to about 30 cm partway through the flight, then comes back down to about 7 cm (`run_01.png`, right panel).
