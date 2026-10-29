# Mission logic: what the mission decides, and from what data

The [system diagram](system_diagram.md) shows where the mission's data comes from. This page shows what the mission *does* with it: every decision, the test behind it, and the data the test reads. It is drawn from the code in `perch_mission`: [perch_cycle.py](../../src/perch_mission/perch_mission/perch_cycle.py), [mission.py](../../src/perch_mission/perch_mission/mission.py) and the planners in [planning/](../../src/perch_mission/perch_mission/planning/).

Read it top to bottom:

1. [The inputs, and the decision each one drives](#1-the-inputs-and-the-decision-each-one-drives)
2. [The states, and the test that moves the mission on](#2-the-states-and-the-test-that-moves-the-mission-on)
3. [One control loop: how a planner flies](#3-one-control-loop-how-a-planner-flies)
4. [The three planners' decisions](#4-the-three-planners-decisions)
5. [From a camera frame to a move, in order](#5-from-a-camera-frame-to-a-move-in-order)
6. [When something goes wrong: the abort for each state](#6-when-something-goes-wrong-the-abort-for-each-state)

Two layers make the decisions. **PerchCycle** runs the states in order and decides when a state has *failed*. The **planners** decide *what to fly next* in the three flying phases (fly_to, search, approach), and when that phase is done. PerchCycle never looks at the branch itself; only the planners do.

## 1. The inputs, and the decision each one drives

| Input | Arrives as | Held as | Read by | Decides |
|---|---|---|---|---|
| Operator's goal | `/vespr/perch` action goal: north, east, altitude, heading, perch_time | `goal` | PerchCycle, fly_to planner | Takeoff altitude, where fly_to flies and which way it faces, how long to hang |
| Operator's cancel / Ctrl+C | action cancel, or SIGINT | `cancelled()` | PerchCycle `check()`, every 0.1 s | Abort the current state. While perched, leave the branch normally instead |
| Branch sighting | `/vespr/perception/branch_pose`, in base_link | levelled with the latest attitude, stamped on arrival: `Situation.branch`, `.branch_time` | search and approach planners | Search: found or keep turning. Approach: which way to turn, where to step, when to make the final move, or abort as lost or too far |
| PX4 attitude | `/fmu/out/vehicle_attitude` | `self.attitude` | `on_branch()` → `level()` | Takes the drone's roll and pitch out of each sighting. With no attitude yet, sightings are dropped |
| PX4 position and heading | `/fmu/out/vehicle_local_position`, through Commander | `Situation.position`, `.heading` | fly_to planner | How far the Goto is and how much to turn to the goal heading. No position yet: wait |
| Rangefinder height | `dist_bottom` in the same message | `flight.position.dist_bottom` | PerchCycle `check_hanging()` | Fell off the branch: below `fall_height` (0.5 m) while it should be hanging |
| Commander's state | Commander, from `/fmu/out/vehicle_status_v1` | `flight.state`: idle, starting, flying, landing, pilot | PerchCycle | `pilot`: stop, the pilot has it. `idle`: landed, or a takeoff that failed. `flying` + `arrived`: takeoff done |
| Commander's `arrived` | Commander, from position against its target | `flight.arrived` | PerchCycle `wait_flying()` | Takeoff and re-arm finished: within 0.15 m of the target |
| Commander refusing | `Refused` raised by a command | | PerchCycle `call()` | "still moving": retry every 0.1 s for up to 60 s. Anything else: the state fails |
| Legs' answers | `/vespr/legs/set_enabled`, `/grasp`, `/release` | success, gripped, message | PerchCycle | Open before the approach, gripped after grasp, opened on release. Any no, or no answer in 10 s: the state fails |
| Time | Commander's clock | `Situation.time`, `flight.now()` | everything | Every timeout, and the planners' wait after each move |

## 2. The states, and the test that moves the mission on

Each arrow is labelled with the test that has to pass to move on, and in brackets the data the test reads. Any state can also end in an abort (section 6); that isn't drawn here to keep the success path readable.

```mermaid
stateDiagram-v2
    direction TB
    [*] --> ground
    ground --> takeoff : /vespr/perch goal accepted (none already running)
    takeoff --> fly_to : flying and arrived, within 30 s (Commander state, arrived)
    fly_to --> search : planner says Done (PX4 position, heading)
    search --> approach : planner says Done, branch seen 5 times in a row (branch_pose)
    approach --> grasp : planner says Grasp, final move ended (branch_pose)
    grasp --> motors_off : legs say success and gripped (legs grasp result)
    motors_off --> perched : land, disarm, still high (Commander state, rangefinder)
    perched --> pre_launch : perch_time over, rangefinder high the whole time (goal, rangefinder)
    pre_launch --> release : re-armed and arrived, then 5 s for thrust to build (Commander state)
    release --> climb_out : legs say success (legs release result)
    climb_out --> land : backed out and rose 0.15 m, timed
    land --> [*] : Commander idle within 60 s, legs disabled

    note right of approach
        Before the planner starts, legs enabled
        and opened. Either fails, the state fails
    end note
    note right of perched
        A cancel here doesn't abort.
        It ends the hang early and leaves
        through pre_launch as normal
    end note
```

The tests for the flying phases (fly_to, search, approach) are the planners' decisions in section 4. A phase also fails when it runs past its time limit: 60 s for fly_to and search, 90 s for approach. (The time limits are in the working tree, not committed yet.)

## 3. One control loop: how a planner flies

`PerchCycle.fly()` runs the same loop for all three flying phases. The planner never flies the drone itself: it returns a step, and the loop carries it out through Commander.

```mermaid
flowchart TD
    START["fly(planner)<br/>planner.start(situation)"] --> CHECK
    CHECK{"Commander state<br/>== pilot?"} -->|yes| PILOT["stop: pilot took over<br/>(no abort move)"]
    CHECK -->|no| CANCEL{"cancel asked?"}
    CANCEL -->|yes| ABORT["fail: cancelled → abort move (section 6)"]
    CANCEL -->|no| TIME{"phase past its<br/>time limit?"}
    TIME -->|yes| ABORT2["fail: took longer than N s → abort move"]
    TIME -->|no| SIT["build a Situation<br/>time · PX4 position, heading ·<br/>newest levelled sighting + when"]
    SIT --> NEXT["step = planner.next_step(situation)"]
    NEXT --> KIND{"which step?"}
    KIND -->|"Done or Grasp"| END["phase over:<br/>go to the next state"]
    KIND -->|"Abort(reason)"| ABORT3["fail: reason → abort move"]
    KIND -->|Wait| SLEEP["sleep 0.1 s"]
    KIND -->|"Goto · MoveBody · YawBy"| MOVE["Commander.goto / move_body / yaw_by<br/>(MoveBody remembered as the final move,<br/>to reverse it when backing out)"]
    MOVE --> REFUSED{"refused?"}
    REFUSED -->|"still moving"| RETRY["retry every 0.1 s,<br/>up to 60 s"] --> REFUSED
    REFUSED -->|"other reason"| ABORT4["fail: refused → abort move"]
    REFUSED -->|accepted| CHECK
    SLEEP --> CHECK
```

**The planners pace themselves.** The mission can't tell when Commander has finished a move, only that Commander has accepted it. So after each move the planner works out when the move should end, from its length and speed (yaw at 30°/s), plus a `settle_time` of 0.5 s. Until then it returns Wait. After that it only trusts sightings taken *after* that time (`Pacer.fresh`). This is why a slow sim can upset it: if the drone takes longer than worked out, the planner acts on a sighting taken mid-move.

## 4. The three planners' decisions

Each planner is called every 0.1 s with the newest Situation and answers with one step. Which planner each phase uses is set in [mission.yaml](../../src/perch_mission/config/mission.yaml) (`fly_to_planner`, `search_planner`, `approach_planner`); these are the only ones written so far.

### fly_to: straight

Reads the PX4 position and heading, never the branch.

```mermaid
flowchart TD
    A{"still settling<br/>after the last move?"} -->|yes| W1["Wait"]
    A -->|no| B{"PX4 position and<br/>heading known?"}
    B -->|no| W2["Wait"]
    B -->|yes| C{"stage"}
    C -->|goto| G["Goto goal north, east, altitude<br/>at cruise_speed (0.5 m/s)<br/>→ stage turn"]
    C -->|turn| D{"goal heading − heading<br/>more than 0.5°?"}
    D -->|yes| Y["YawBy the short way round<br/>→ stage done"]
    D -->|no| DONE["Done"]
    C -->|done| DONE
```

### search: turn

Reads the branch sightings and their times. Counts a *run* of sightings; a gap of more than 0.5 s between two sightings breaks the run.

```mermaid
flowchart TD
    A{"still settling<br/>after the last turn?"} -->|yes| W1["Wait"]
    A -->|no| B{"a new sighting,<br/>taken after the turn?"}
    B -->|yes| R["run + 1<br/>(reset to 1 if > 0.5 s<br/>since the last one)"]
    R --> F{"run ≥ seen_frames (5)?"}
    F -->|yes| DONE["Done: found"]
    F -->|no| L
    B -->|no| L{"still in the 1 s look,<br/>or partway through a run?"}
    L -->|yes| W2["Wait"]
    L -->|no| T{"another 30° turn would<br/>go past 360°?"}
    T -->|yes| AB["Abort: no branch found"]
    T -->|no| Y["YawBy 30°, run = 0,<br/>look for 1 s after it ends"]
```

### approach: standoff

The main decision-maker. It reads the newest levelled sighting: where the branch is from base_link (x, y, z) and which way it runs (its orientation). From those it works out two numbers every step:

- **turn**: how far to yaw so the branch runs straight across the nose. That is the angle of the branch's x axis, minus 90°. The detector always points that axis left, so squared up is 0.
- **to_go**: where the drone should move to reach the *standoff point*, 1.0 m behind the capture point (`standoff_back`), squared up to the branch. At the standoff the camera still sees the branch.

```mermaid
flowchart TD
    A{"still settling<br/>after the last move?"} -->|yes| W1["Wait<br/>(also after the final move:<br/>grasp only once it has ended)"]
    A -->|no| S{"final move<br/>already made?"}
    S -->|yes| GR["Grasp, then Done"]
    S -->|no| FR{"a sighting taken<br/>after the last move?"}
    FR -->|no| LOST{"none for more than<br/>2 s (lost_timeout)?"}
    LOST -->|yes| AB1["Abort: lost the branch"]
    LOST -->|no| W2["Wait"]
    FR -->|yes| FIRST{"first sighting, and the<br/>branch more than 3 m away?"}
    FIRST -->|yes| AB2["Abort: branch too far away"]
    FIRST -->|no| CALC["work out turn and to_go<br/>from the branch's position<br/>and orientation"]
    CALC --> YAW{"abs(turn) more than<br/>5° (yaw_tolerance)?"}
    YAW -->|yes| KEEP["allowed = turn, cut so the branch stays<br/>within 25° of the nose (keep_in_view)"]
    KEEP --> K2{"allowed ≥ 1°, or<br/>already at the standoff?"}
    K2 -->|yes| Y["YawBy allowed"]
    K2 -->|"no: turning would<br/>lose the branch"| DIST
    YAW -->|no| DIST{"more than 5 cm from<br/>the standoff point?"}
    DIST -->|yes| STEP["MoveBody towards it, at most<br/>0.3 m (step_length), 0.2 m/s"]
    DIST -->|no| FINAL["final move: MoveBody so the branch<br/>lands on the capture point (the hook hole),<br/>0.1 m/s, blind → stage grasp"]
```

So the approach is a loop of **turn a bit, step a bit, look again**, until it is squared up and within 5 cm of the standoff. Then it makes one move it can't check, because the branch is too close to see, and grasps.

## 5. From a camera frame to a move, in order

One approach step, as it happens in time. The callbacks and the mission's loop run on different threads: the newest sighting is just a value that `on_branch()` overwrites and the loop reads.

```mermaid
sequenceDiagram
    autonumber
    participant DET as branch_detector
    participant PX4 as PX4 (via XRCE-DDS)
    participant MN as mission.py callbacks
    participant PC as PerchCycle.fly()
    participant PL as Standoff planner
    participant CMD as Commander

    PX4->>MN: /fmu/out/vehicle_attitude
    Note over MN: on_attitude(): keep the newest q
    DET->>MN: /vespr/perception/branch_pose (base_link)
    Note over MN: on_branch(): level(pose, attitude),<br/>stamp with Commander's clock, keep the newest
    loop every 0.1 s
        PC->>PC: check(): pilot? cancel? past the time limit?
        PC->>MN: sighting()
        MN-->>PC: levelled pose, when it arrived
        PC->>PL: next_step(Situation)
        alt still settling, or no fresh sighting
            PL-->>PC: Wait
        else needs to turn
            PL-->>PC: YawBy(deg)
            PC->>CMD: yaw_by(deg)
        else needs to step
            PL-->>PC: MoveBody(forward, right, up, speed)
            PC->>CMD: move_body(...)
        end
        CMD->>PX4: /fmu/in/trajectory_setpoint, streamed until it gets there
    end
```

## 6. When something goes wrong: the abort for each state

A state fails when one of its tests fails, a timeout runs out, or the operator cancels. The mission then does the abort move for the state it was in, and the goal ends unsuccessful with the reason and what the abort did. If PX4 reports the pilot has taken over (`pilot`), the mission stops at once and does nothing more.

```mermaid
flowchart LR
    subgraph AIR["In the air, not on the branch"]
        T["takeoff"] & F["fly_to"] & S["search"] & C["climb_out"] --> L1["land, disable legs"]
        A["approach"] --> L2["back out the way the final move<br/>came in (or 0.6 m back), land"]
        G["grasp"] --> L3["open the legs, back out, land"]
    end
    subgraph BR["On the branch"]
        FELL["fell<br/>(rangefinder < 0.5 m)"] --> L4["land: it's on the floor"]
        MO["motors_off"] --> H1["leave it hanging in land mode:<br/>someone has to come"]
        PER["perched"] -->|cancel| LEAVE["leave as normal:<br/>pre_launch → release → climb_out → land"]
        PL["pre_launch"] --> H2["leave it latched, disarmed:<br/>someone has to come"]
        RE["release"] --> H3["motors off, stuck:<br/>someone has to come"]
    end
    LA["land"] --> N["nothing more to do:<br/>did not finish landing"]
```

Why the branch states don't try to fly away: once the drone is hanging, PX4's height estimate can't be trusted (open issue A in [mission.md](mission.md)), so trying to fly off on a failure could drop it. Leaving it on the branch for someone to collect is the safe choice.
