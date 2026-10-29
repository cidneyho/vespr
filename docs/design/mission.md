# Design: mission loop

Status: work in progress (2026-10-28). Implemented and unit tested. In the sim the full cycle worked in 4 runs (3 of 3 on 2026-10-26, 1 on 2026-10-28), but leaving the branch is unreliable: on 2026-10-28 one run released and backed out and stayed hooked on the rod (open issue A). See "Open issues" at the end.

Walk-through with diagrams, and how to run it in the sim: [mission_flow.md](mission_flow.md).

## Goal

Run the full perch cycle from the board ([perch-cycle.md](../system-map/perch-cycle.md)) when an operator asks: take off, fly to the position, find the branch, perch, rest, leave, land. The mission uses the planners for the flying (fly-to, search, approach), Commander to fly, and the legs to grip.

This step is sim only, flying on flow and rangefinder alone. The mocap stages come after, on top of this.

## Starting it: the `/vespr/perch` action

An operator sends a goal, watches progress, gets a result, and can cancel.

New type, `perch_interfaces/action/Perch.action`:

```
# Goal: where to go, in Commander's frame (m from the takeoff spot), and how long to stay
float32 north
float32 east
float32 altitude
float32 heading       # deg, 0 = north, 90 = east. Faced on arrival, as a first guess at the branch
float32 perch_time    # s to rest on the branch before leaving
---
bool success          # perched and came back down
string state          # where it ended: "land" on success, else the state it aborted in
string message        # why it aborted, or a short summary
---
string state          # the current state
```

From the command line:
```
ros2 action send_goal /vespr/perch perch_interfaces/action/Perch \
  "{north: 0.3, east: 0.8, altitude: 1.3, heading: 60.0, perch_time: 10.0}" --feedback
```

**One goal at a time.** A new goal is refused while one is running.

**Cancel** runs the abort for the current state (below), so the drone always ends somewhere safe.

## States

The board's states, with fly-to added. Also published on `/vespr/mission/state` (`std_msgs/String`, latched) whenever the state changes.

| # | State | What the mission does | Next |
|---|---|---|---|
| 1 | `ground` | Waits for a goal. Legs disabled | `takeoff` |
| 2 | `takeoff` | `commander.takeoff(altitude)` and waits until it is flying and has reached the altitude | `fly_to` |
| — | `fly_to` | Runs the fly-to planner with the goal's target and heading | `search` |
| 3 | `search` | Runs the search planner | `approach` |
| 4–6 | `approach` | Enables the legs (`set_enabled`) and opens them, then runs the approach planner up to its `Grasp` step | `grasp` |
| 7 | `grasp` | `/vespr/legs/grasp`. Needs `success` and `gripped` | `motors_off` |
| 8 | `motors_off` | `commander.land()`, then `commander.disarm()` (see below). Waits for disarmed | `perched` |
| 9 | `perched` | Waits `perch_time` | `pre_launch` |
| 10 | `pre_launch` | `commander.takeoff(current altitude)`: arm and hold where it hangs, **still latched** | `release` |
| 11 | `release` | `/vespr/legs/release` | `climb_out` |
| 12 | `climb_out` | Back straight out through the hook's opening (the reverse of the final move), then hold | `land` |
| 13 | `land` | `commander.land()`, waits for disarmed, then disables the legs. Success | `ground` |

The board's loop from climb-out back to takeoff (perch again) is left for later.

## Motors off: land, then disarm

PX4's land mode alone doesn't disarm while hanging from the branch. Tested in `perch_rod`: thrust dropped to 0.12 and the drone hung steady for 30 s, still armed. PX4's land check needs the rangefinder to see the ground close by, and it read 1.1 m. The real drone has the same rangefinder, so it would do the same.

So `motors_off` is:
1. `commander.land()`: PX4 lowers the thrust and the hook takes the weight gradually.
2. After `land_settle` (default 2 s), `commander.disarm()`: PX4's forced disarm, which Commander only allows in land mode.
3. Wait until disarmed, at most `motors_off_timeout` (default 5 s).

Tested in `perch_rod`: disarmed in 0.5 s, and the drone stayed hanging on the hook.

## Aborts

When a planner returns `Abort`, a step fails, or the operator cancels, the mission makes the abort move for the state it is in, then ends with `success = false`.

| Abort in | Abort move |
|---|---|
| `takeoff`, `fly_to`, `search` | Land where it is |
| `approach` | Back straight out (from where the branch was last seen), then land |
| `grasp` | Release, back straight out, then land |
| `motors_off` (not disarmed in time) | Stay hanging on the branch in land mode, at low thrust. Report it. Commander has no way back from land mode to flying, so it can't back out |
| `perched` (cancel) | Leave normally: `pre_launch`, `release`, `climb_out`, `land` |
| `pre_launch` (can't arm) | Stay latched, disarmed. Report it; someone has to come |
| `release` (jammed) | Land and disarm, stay on the branch. Report it |
| `climb_out` | Land |

**Pilot takeover.** If Commander reports the pilot has taken control (PX4 left offboard), the mission stops commanding at once and ends with "pilot took control". No abort move.

**Commander refuses a step** with anything other than "still moving": abort.

## Inside the node

- **One process, two nodes.** `mission` and Commander's `flight_commander`, on a multi-threaded executor. Commander is used as it is: its methods, plus reading its `state` (flying, landed, pilot), `arrived` (takeoff done) and `position` (for the fly-to planner's `Situation`).
- **The loop** runs in the action's thread. While a planner runs, every 0.1 s it builds a `Situation`, asks for the next step and carries it out:

  | Step | Mission does |
  |---|---|
  | `Goto`, `MoveBody`, `YawBy` | The Commander call, retried every 0.1 s while Commander says "still moving" |
  | `Wait` | Nothing |
  | `Grasp` | Moves on to the `grasp` state |
  | `Done` | Moves on to the next state |
  | `Abort` | The abort move for this state |

- **Branch sightings** come from `/vespr/perception/branch_pose`. They are timed by when they arrive (Commander's clock), not by their stamp, since the detector stamps sim time and Commander uses wall time.
- **No mocap.** The mission doesn't subscribe to `/mocap/*`. Mocap is only for measuring how the flight went.

## Parameters

| Name | Default | Meaning |
|---|---|---|
| `fly_to_planner`, `search_planner`, `approach_planner` | `straight`, `turn`, `standoff` | Which implementation each phase uses |
| `fly_to.*`, `search.*`, `approach.*` | the planners' defaults | Any planner setting, e.g. `approach.step_length` |
| `takeoff_timeout` | 30.0 | s to reach the altitude after taking off |
| `land_settle` | 2.0 | s in land mode before disarming, so the hook takes the weight |
| `motors_off_timeout` | 5.0 | s for PX4 to disarm on the branch |
| `land_timeout` | 60.0 | s to land and disarm |
| `fall_height` | 0.5 | m: after motors off and while perched, a rangefinder reading below this means the drone fell |

The back-out move uses the approach planner's `standoff_back` and `final_speed`, so it reverses the final move.

## Package: `perch_mission`

| File | Contents |
|---|---|
| `perch_mission/mission.py` | The `mission` node: the action, the states, the aborts |
| `launch/mission.launch.py` | The mission, every parameter as a launch argument |
| `perch_interfaces/action/Perch.action` | The action type |

## Testing

- **Unit**, with a fake Commander and fake legs, no sim: the full cycle in order; each abort row; cancel in flight and when perched; pilot takeover; a refused step.
- **Sim**, in `perch_rod`, with the fake detector and the sim legs bridge:
  1. The full cycle from the command above. Check in Gazebo that the drone hangs from the rod with the motors off, then leaves and lands.
  2. Motors off: PX4 disarms on the branch and the drone stays hanging.
  3. Search abort: a target with the rod out of the detector's range, so it lands.
  4. Cancel during the approach: backs out and lands.

## Out of scope

- The mocap stages (next).
- Perching again after climb-out.
- Retrying a failed grasp or approach.
- The sim can't tell whether the rod is really inside the hook: "gripped" only means the gate closed (from the legs design).

## Open issues

Found in sim runs on 2026-10-25. To decide before going on.

**A. Leaving the branch (pre-launch, release, climb-out).** While hanging, PX4's height estimate drifts 1 to 2 cm/s though the drone is still. On re-arming, PX4 believes it has already lifted and holds thrust near 0.5, but this drone needs 0.76 to hover, so it never lifts off the hook. (PX4 also learned a wrong hover thrust while hanging; fixed for the sim drone by a fixed `MPC_THR_HOVER` 0.76 with `MPC_USE_HTE` 0 in its airframe. The real drone needs its own measured value.) The code now releases first, then backs out while rising `climb_out_lift`. That is safe in the sim, where the hook's fixed overhang carries the weight, but it reverses the board's order (thrust before release), which protects the real drone from dropping. Decide: release first (sim only for now), or find a way to build thrust before release that doesn't depend on PX4's height estimate.

Seen 2026-10-28: one run perched, released and backed out, but the hook never rose clear of the rod's overhang. Land mode then can't finish while hanging, so the mission ended "did not land in time. did not finish landing" after `land_timeout`, with the drone still armed in land mode on the rod (PX4: not landed, rangefinder 1.11 m). Two other runs left cleanly, so it is intermittent.

**B. A missed grasp reports success.** Fixed 2026-10-26 by the fall check: after motors-off and while perched, a rangefinder reading below `fall_height` ends the run with "fell off the branch" and lands. The sim's "gripped" still only means the gate closed.

**C. Approach accuracy.** One miss in four approaches; the hook hole is 10 cm across. One run turned back and forth about 20 times (±7°) before moving. PX4's heading was about 11° off the truth on flow alone. Not yet investigated.

Seen 2026-10-28: one approach turned back and forth for 160 s, the turns growing from ±6° to ±19° (the `keep_in_view` limit), until it lost the branch. That run had extra load (mocap_sim, fake_vio and a photo camera running). The planners time their moves by wall clock, so a slower sim may let them act on sightings taken mid-turn. Not confirmed. Nothing limits how long a phase may run, so the drone hovered until the branch was lost. Proposed: a time limit per phase.

**D. The camera now sits lower.** The sim camera was 0.24 m too high (fixed 2026-10-26: it is level with the body, where the RealSense will go). At the capture point the branch is now 0.30 m above the camera, and the camera sees only 21° up, so the branch leaves the view about 0.9 m out. The default `standoff_back` of 0.6 m is too close: the approach will lose the branch. Options: `standoff_back` about 1.0 m (a longer blind final move), or tilt the camera up (`tilt_up_deg` in `perch_sim/config/x500_hook.yaml`), which depends on the real mount. Done 2026-10-26: `standoff_back` is 1.0 m; at the standoff the branch is 18.7° up, in view.
