# Design: flight path planning

Status: implemented (2026-10-25). In `perch_rod` the three phases took the drone from takeoff to the rod sitting within 4 cm of the hook hole's centre (hole radius 5 cm).

## Goal

Plan the drone's path in three phases, each with implementations we can swap by a parameter:

1. **Fly to the position:** fly to a spot the user gives.
2. **Search:** look around there until the detector sees a branch.
3. **Approach and perch:** approach the branch slowly until the hook is around it, then grasp.

Planners **only plan**. They never call Commander or the legs. The mission loop (next step) asks the planner for the next step and carries it out. That keeps planners testable without a sim, and makes them easy to swap.

## Where this sits on the board

The board's perch cycle ([perch-cycle.md](../system-map/perch-cycle.md)), with "fly to the position" added:

| Board state | Who |
|---|---|
| 2 Takeoff / hover | Mission (Commander `takeoff`) |
| (new) Fly to the position | Phase 1 planner |
| 3 Search | Phase 2 planner |
| 4 Approach (to a standoff point) | Phase 3 planner |
| 5 Final alignment | Phase 3 planner |
| 6 Touchdown | Phase 3 planner |
| 7 Grasp | Phase 3 planner says "grasp now"; the mission calls the legs |
| Aborts (hold, back out, land) | Mission, when a planner gives up |

**For the team:** the board has no "fly to the position". It assumes the drone takes off within sight of the branch. This adds a step between states 2 and 3, for "go and monitor this area"

## What a planner sees and returns

```python
@dataclass(frozen=True)
class Situation:
    time: float                    # s, the same clock as Commander's
    position: tuple | None         # (north, east, altitude), Commander's frame. Phase 1 only
    heading: float | None          # deg, Commander's frame. Phase 1 only
    branch: Pose | None            # newest /vespr/perception/branch_pose, in base_link
    branch_time: float | None      # s, when that sighting was taken, on the same clock as `time`

class Planner(Protocol):
    def start(self, situation: Situation) -> None: ...   # a fresh run of this phase
    def next_step(self, situation: Situation) -> Step: ...
```

A step is one thing for the mission to do. Each maps onto a Commander call we already have, or onto the legs:

| Step | Mission does |
|---|---|
| `Goto(north, east, altitude, speed)` | `commander.goto(...)` |
| `MoveBody(forward, right, up, speed)` | `commander.move_body(...)` |
| `YawBy(degrees)` | `commander.yaw_by(...)` |
| `Wait()` | Nothing this time; ask again shortly (e.g. waiting for the detector) |
| `Grasp()` | Call `/vespr/legs/grasp` (phase 3's last step) |
| `Done()` | This phase is over; go to the next |
| `Abort(reason)` | The planner gives up; the mission picks the abort move |

**Drone position only in phase 1.** Phase 1 needs the drone's position and heading in Commander's frame, to time its `Goto` and to know how far to turn. The mission fills them in from PX4's position estimate. Phases 2 and 3 move relative to the drone (`MoveBody`, `YawBy`), using only where the detector sees the branch. So planners never touch the mocap / VIO / PX4 frame question.

**One move at a time.** Commander refuses a new move while one is under way, and has no way to ask whether a move has finished. So:
- The mission retries a step Commander refuses with "still moving" until it is accepted.
- Each planner works out when its own moves end, from their length and speed (and Commander's 30°/s turn rate), and returns `Wait` until then plus `settle_time`.
- Planners only act on sightings taken after that, never on one taken mid-move. Re-aiming between steps is how phase 3 corrects its course.

**Clocks.** `time` and `branch_time` must be on the same clock. The detector stamps sim time while Commander uses wall time, so for now the mission times sightings by when they arrive.

## Phase 1: fly to the position

**Input:** a target the user enters, in Commander's frame (north, east, altitude from the takeoff spot), as a parameter or in code. Also a heading to face there, as a first guess at where branches are.

**First implementation, `straight`:**
1. `Goto` the target at cruise speed.
2. Turn to the given heading.
3. `Done`.

## Phase 2: search

Board state 3: "hover, yaw slowly, wait for `branch_pose`".

**First implementation, `turn`:**
1. If the branch has been seen in `seen_frames` sightings in a row: `Done`.
2. Otherwise `Wait` `search_dwell` seconds for the detector, then turn `search_step` degrees, and repeat.
3. `Abort("no branch found")` after a full turn without one. The board's abort here is to land.

## Phase 3: approach and perch

**Capture point:** where the branch should be, relative to `base_link`, at the moment of grasping. For the sim hook that is the centre of the hook's hole, 0.30 m straight above the body (from `hook.yaml`).

**Standoff point:** a spot the drone flies to first, from which the branch is still in the camera's view. It's set by how far back and down from the capture point the drone waits:
- **Hook** (opening facing forward): straight back, same height, so the drone then flies forward into the branch.
- **Legs** (rising from under the branch, as on the board): straight down.

**First implementation, `standoff`:**
1. **Align and approach** (board state 4), one step per fresh sighting:
   - The standoff point is squared up to the branch: straight out from it, as if the drone already faced across it.
   - If the branch doesn't yet run straight across the nose (within `yaw_tolerance`), turn towards that, but never so far that the branch ends up more than `keep_in_view` off the nose. Turning first could lose the branch when it's off to one side.
   - Otherwise, or if the turn allowed is under 1°, `MoveBody` towards the standoff point: at most `step_length` at `approach_speed`.
   - Stop once within `standoff_tolerance` and aligned.
2. **Final** (states 5–6): one `MoveBody` from the standoff point to the capture point at `final_speed`, from the last sighting. The branch leaves the camera's view on the way in (the expected blind spot), so this part is flown blind.
3. `Grasp`, once the final move has ended.

**Gives up** with `Abort`:
- if the branch hasn't been seen for `lost_timeout` before the final move;
- if the branch is further than `max_range` away when the phase starts.

## Parameters

Every number is a placeholder to tune by experiment. None are known yet.

| Name | Default | Meaning |
|---|---|---|
| `fly_to_planner` | `straight` | Phase 1 implementation |
| `search_planner` | `turn` | Phase 2 implementation |
| `approach_planner` | `standoff` | Phase 3 implementation |
| `target_north`, `target_east`, `target_altitude` | 0.0, 0.0, 1.5 | Phase 1 target, m from the takeoff spot |
| `target_heading` | 90.0 | Heading to face there, deg (0 = north, 90 = east) |
| `cruise_speed` | 0.5 | Phase 1 speed, m/s |
| `search_step`, `search_dwell`, `seen_frames` | 30.0, 1.0, 5 | Search: deg per turn, s to look, sightings in a row |
| `capture_x`, `capture_y`, `capture_z` | 0.0, 0.0, 0.30 | Where the branch should end up in `base_link`, m |
| `standoff_back`, `standoff_down` | 1.0, 0.0 | Standoff point from the capture point, m. 1.0 keeps the branch in the camera's view (it sits 0.30 m below the capture point) |
| `approach_speed`, `final_speed` | 0.2, 0.1 | m/s |
| `step_length` | 0.3 | Longest approach step, m |
| `yaw_tolerance`, `standoff_tolerance` | 5.0, 0.05 | deg, m |
| `keep_in_view` | 25.0 | Most a turn may leave the branch off the nose, deg (camera half-width 34.5°) |
| `lost_timeout`, `max_range` | 2.0, 3.0 | s, m |
| `settle_time` | 0.5 | Wait after each move before trusting a sighting, s (every planner) |

## Package: `vespr_flight`

The naming doc puts mission logic in `vespr_flight`. New, pure Python, no ROS in the planners:

| File | Contents |
|---|---|
| `vespr_flight/planning/steps.py` | `Situation`, the step types |
| `vespr_flight/planning/planner.py` | The `Planner` protocol, and a table from name to implementation |
| `vespr_flight/planning/fly_to.py` | `straight` |
| `vespr_flight/planning/search.py` | `turn` |
| `vespr_flight/planning/approach.py` | `standoff` |

Commander stays in `perch_flight` as it is.

## Testing

- **Unit:** feed planners made-up sightings and check their steps. Covered: fly-to goes to the target and faces the heading; the search turns and gives up; align turns the right way; sightings taken mid-move are ignored; `Grasp` waits for the final move; aborts.
- **Closed loop:** a fake drone carries out the steps and a fake detector reports the branch. From 200 random starts (branch 1 to 2.8 m away, up to 30° off the nose, at any skew), the approach always ends with the branch within 6 cm of the capture point.
- **Sim**, with a test-only script carrying out the steps with Commander and the fake detector in `perch_rod`: takeoff, fly to 2.2 m from the rod facing 36° away, search (one turn), approach (a turn and 6 steps), final move. The rod ended 2 to 4 cm from the hook hole's centre, measured from Gazebo.

## Out of scope

- The mission loop and its aborts (next step).
- Obstacles, other paths, picking between several branches.
- Retrying a failed grasp.
