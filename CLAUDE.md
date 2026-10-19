# VESPR

Perching drone. ROS 2 Jazzy workspace for the Jetson; mocap lives in a separate repo. Setup and versions: [README.md](README.md).

## Project docs

- **System architecture**: [docs/system-map/](docs/system-map/README.md). Read the index first, then open only the file you need (components, wiring, topics, mocap, perch state machine, sim). It is a copy of the team's FigJam board; the board is the source of truth.
- **Naming** (topics, nodes, packages, frames): [NAMING.md](NAMING.md). Follow it for any new name.
- **Interface contract between packages**: [docs/interfaces.md](docs/interfaces.md).

## Package names

Use `vespr` for packages, nodes and topics (`vespr_flight`, `/vespr/legs/state`). The `perch_*` packages in `src/` are temporary test code, not the naming to copy.
