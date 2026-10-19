# VESPR system map

Markdown copy of the team's FigJam board, split into one file per topic so you only need to open the part you care about.

- **Source of truth:** [VESPR System Map (FigJam)](https://www.figma.com/board/UN69kT0kE1ImB1Mm2sNLco/VESPR-System-Map?node-id=0-1)
- **Last synced from Figma:** 2026-10-19
- If this folder and the board disagree, the board wins. Re-sync these files rather than editing them by hand.

## Files

| File | Board box | Read it when you need |
|---|---|---|
| [overview.md](overview.md) | 01, 02 | The five components, what runs where, the "mocap never steers" rule, why there are three computers |
| [wiring.md](wiring.md) | 03, 10 (power) | Every physical link and protocol between parts, power rails |
| [ros2-primer.md](ros2-primer.md) | 04 | Topics vs services vs actions, QoS trap, namespaces, NED vs ENU, debug commands |
| [topics-and-interfaces.md](topics-and-interfaces.md) | 05 | Every topic and message type, the PX4 `/fmu` topics we use, the legs interface |
| [system-map.md](system-map.md) | 06 | The full block diagram: nodes, hardware, owners, connections |
| [mocap.md](mocap.md) | 07 | Mocap pipeline, what the evaluator computes, the two-repo boundary, mocap phase-out stages |
| [perch-cycle.md](perch-cycle.md) | 08 | Mission state machine: the 13 states with their exits and aborts |
| [naming.md](naming.md) | 09 | Frames and the name tree (full rules live in [/NAMING.md](../../NAMING.md)) |
| [sim-and-logging.md](sim-and-logging.md) | 10 | SITL/Gazebo setup, what sim is for, logs, the five result numbers |

## Owners

| Person | Area |
|---|---|
| Rushav | mechanical, mocap, eval |
| John | perception |
| Cidney | flight system, ROS |
| Dom | electronics, sim, data |

## Package names

Packages are named `vespr_*`: `vespr_bringup`, `vespr_interfaces`, `vespr_perception`, `vespr_flight`, `vespr_legs`, `vespr_eval`. The `perch_*` packages currently in `src/` are temporary test code; name new packages, nodes and topics with `vespr`.

## Related docs

- [docs/interfaces.md](../interfaces.md): the interface contract between packages, taken from [topics-and-interfaces.md](topics-and-interfaces.md) and [naming.md](naming.md).
- [/NAMING.md](../../NAMING.md): full naming rules.
