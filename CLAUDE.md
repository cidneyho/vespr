# VESPR

Perching drone. ROS 2 Jazzy workspace for the Jetson; mocap lives in a separate repo. Setup and versions: [README.md](README.md).

## Project docs

- **System architecture**: [docs/system-map/](docs/system-map/README.md). Read the index first, then open only the file you need (components, wiring, topics, mocap, perch state machine, sim). It is a copy of the team's FigJam board; the board is the source of truth.
- **Naming** (topics, nodes, packages, frames): [docs/NAMING.md](docs/NAMING.md). Follow it for any new name.
- **Flight accuracy**: how to measure it (sim or real drone): [docs/design/flight_accuracy.md](docs/design/flight_accuracy.md). What was wrong with the sim's estimate and how it was found and fixed (saved params, declination, flow sensor): [src/perch_eval/results/flight_accuracy_fixes_2026-11-05.md](src/perch_eval/results/flight_accuracy_fixes_2026-11-05.md). Read it before debugging drift or heading errors in the sim.
- **Topics, services, actions and message types**: [docs/system-map/topics-and-interfaces.md](docs/system-map/topics-and-interfaces.md). Frames: [docs/system-map/naming.md](docs/system-map/naming.md).

## Package names

Packages use the `perch_` prefix (`perch_flight`, `perch_mission`, `perch_sim`). Topic and action names keep `/vespr/...` (`/vespr/legs/state`, `/vespr/perch`); don't rename them. Package list: [docs/NAMING.md](docs/NAMING.md) section 6.
