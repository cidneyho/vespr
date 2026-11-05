# Design: flight accuracy eval

Status: implemented (2026-11-02), sim and real drone. First results, 10 runs in the sim: [flight_accuracy_2026-11-02.md](../../src/perch_eval/results/flight_accuracy_2026-11-02.md). What those showed was wrong with the sim, and the fixes: [flight_accuracy_fixes_2026-11-05.md](../../src/perch_eval/results/flight_accuracy_fixes_2026-11-05.md).

## Goal

Find out how far the drone strays from the path it's told to fly, and why. Flying alone: no branch, no perch. The same path, recording and analysis in the sim and on the real drone with mocap. The numbers decide what to work on next: the state estimate, PX4's tuning, or the mission's settings.

**The preferred path** is what Commander sends to PX4 on `/fmu/in/trajectory_setpoint`: a position, velocity and yaw, 10 times a second.

## What we measure, and why

| # | Measure | Why |
|---|---|---|
| 1 | **Path error**, split into along-track (ahead of or behind the setpoint), off the line (distance from the line of travel), altitude and heading. Moving and holding reported apart | One "how far off" number doesn't say what to fix. Each part has its own causes |
| 2 | **Where the error comes from**: estimation error (PX4's estimate vs the truth) and control error (the setpoint vs PX4's estimate) | Both look like "off the path", but the fixes differ: a better estimate (flow drift, later VIO) or PX4 tuning |
| 3 | **Settling** after each move: overshoot, time to settle within 5 cm, error at the end of the hold, wobble while holding | The mission waits `settle_time` (0.5 s) after a move before trusting a sighting. This tells us whether that's enough |
| 4 | **Spread over 10 runs**: mean, worst case and spread of each number above | One run can be luck. A steady 2 cm is easier to plan around than 0 cm ± 8 cm |
| 5 | **The flight paths**, preferred and actual, plotted from above and as altitude over time | Patterns (a steady lean, drift growing over the flight, corners cut) are easier to see than to read from tables |

## How we measure

**Three signals, recorded together in one rosbag per run:**

| Signal | Topic | Role |
|---|---|---|
| Setpoint | `/fmu/in/trajectory_setpoint` | where it should be |
| PX4's estimate | `/fmu/out/vehicle_local_position` (position and heading) | where PX4 thinks it is |
| PX4's state | `/fmu/out/vehicle_status_v1` | when it's armed and in offboard |
| Truth | `/mocap/vespr/pose`, from `mocap_sim` in the sim or the mocap PC | where it really is |

The errors are differences between them:

```
total error      = truth    − setpoint
estimation error = truth    − estimate
control error    = estimate − setpoint      (total = estimation + control)
```

**The flight** (`perch_eval/flight_path.py`): the same scripted path every run, through Commander, not the mission. Wait on the ground 10 s after PX4 is ready, take off to 1 m, then 1 m north, 1 m east, turn +90°, 1 m south, 1 m west, up 0.5 m, down 0.5 m, turn −90°, land. It holds 5 s after each step, flies at 0.3 m/s, and stays within 1.5 m of the takeoff spot, well clear of the rod 3 m away in the sim. In the sim each run starts a fresh sim, so PX4's estimate starts from scratch every time. Altitude, climb, turn and wait are options, for narrowing a problem down (no turns, a different height).

**Lining up frames:** setpoints and PX4's estimate are in PX4's local frame (north-east-down, origin where PX4 started). Mocap is in its own frame (Gazebo's world in the sim, the room on the real drone), z up. The eval finds the fixed transform between them from the drone sitting still in the 3 s before arming, then puts everything in PX4's frame. Drift after that shows up as estimation error, which is the point.

**Blind spot:** the alignment takes PX4's heading at rest as correct, so a heading error that is constant over the whole flight is invisible. It shows only indirectly, as sideways drift on every move (about 4 to 5 cm per metre for 2.6°). In the sim, PX4's ulog (copied into each run's logs folder) has Gazebo's true heading in `vehicle_attitude_groundtruth`; on the real drone, compare mocap's heading with PX4's at rest by hand.

**Lining up time:** all three are timed by when the bag received them. Commander stamps wall time and mocap stamps sim time, so the message stamps can't be compared directly. Setpoints and the estimate are resampled onto the mocap times (about 50 Hz), so settling isn't limited to the setpoints' 10 Hz.

**Moving, turning or holding:** from the setpoint at that moment. *Moving* when its velocity isn't zero, *turning* when only its yaw rate isn't, otherwise *holding*. While moving, the error splits into along-track (+ ahead) and off the line; while turning or holding, into the horizontal distance off the target. Altitude (+ above) and heading are reported in every case. Settling is measured from the moment a step's setpoint stops, twice: for the actual position (total) and for PX4's estimate (control). The estimate settling while the actual position doesn't points at estimation.

**Which part of the flight counts:** the error stats start at the first step after the takeoff hold. The drone is still climbing from the ground well into that hold (seen in the first trial), so the climb isn't mixed in. The takeoff hold's settling is still reported.

## Running it

```bash
src/perch_eval/scripts/flight_accuracy_runs.sh            # sim, 10 runs; RUNS=3 for fewer
REAL=1 RUNS=5 src/perch_eval/scripts/flight_accuracy_runs.sh   # the real drone
BEFORE=<an earlier output folder> src/perch_eval/scripts/flight_accuracy_runs.sh   # and compare
```

**Sim:** it flies each run in a fresh sim with its Gazebo window (the hook drone's optical flow needs the camera rendered), records a bag, then runs the analysis. Nothing else should send PX4 RC or joystick input: on 2026-11-02 manual control came and went every half second, and PX4 failsafed to land right after arming. Suspected source: QGroundControl's joystick (not confirmed). `run_sim.sh` clears PX4's saved parameters before each run; see the fixes write-up for why that matters.

**Real drone:** PX4's DDS agent and the mocap PC must already be publishing; the script checks for `/fmu/out/vehicle_status_v1` and `/mocap/vespr/pose`, then asks before each run. The path needs 1.5 m clear round the takeoff spot and 2 m of height. If the pilot takes over, `flight_path` stops commanding and waits for them to land, and the analysis uses the run up to the takeover. PX4's own flight log stays on the SD card.

**Testing a change:** fly a set, make one change, fly another set with `BEFORE` pointing at the first. The analysis then also writes `before_after.png` (the two sets' paths and their estimate-vs-actual error over time, side by side) and `before_after.md` (the headline numbers). Three runs per set were enough to see each fix in the sim (runs repeat to within about 1 cm); use 10 for a baseline. To compare sets already flown: `ros2 run perch_eval flight_accuracy <after>/run_?? --before <before>/run_?? --names Before After --out <dir>`.

## Output

Bags and logs go to `~/vespr_recordings/flight_accuracy/<date and time>/` (not in git). The analysis, `ros2 run perch_eval flight_accuracy <bags> --out <dir>`, writes to `results/` there:

- `paths.png`: every run's actual path over the preferred one, from above and as altitude over time.
- `run_NN.png`: one run's preferred path, actual path and PX4's estimate, and the size of each error over time.
- `summary.md`: the tables. `flight_accuracy.json`: every number, per run and pooled.
- With `--before`: `before_after.png` and `before_after.md`, as above.

The write-up of a set of runs goes in `src/perch_eval/results/`, with the setup (commit, world, settings), like the IMU results.

## Decided

1. **Settling tolerance:** 5 cm, the approach planner's `standoff_tolerance`.
2. **Runs:** 10.
3. **The path:** scripted with Commander, not the mission's phases.
4. **Sim first.** The real drone uses the same path and analysis (added 2026-11-05, not flown yet).

## Later

- **Settling over time** (a separate test): how much closer it gets the longer it holds, to pick the best `settle_time`. Graph the distance to the target against time held.
