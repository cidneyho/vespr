#!/usr/bin/env bash
# Start PX4 SITL in one of our perch_sim worlds.
# Usage: run_sim.sh [world] [vehicle]   (defaults: perch_rod gz_x500)
set -euo pipefail

WORLD=${1:-perch_rod}
VEHICLE=${2:-gz_x500}
PERCH_SIM=$(cd "$(dirname "$0")/.." && pwd)
PX4_DIR=${PX4_DIR:-$HOME/PX4-Autopilot}

# PX4 only loads worlds from its own worlds directory, so link ours in
for world in "$PERCH_SIM"/worlds/*.sdf; do
  ln -sf "$world" "$PX4_DIR/Tools/simulation/gz/worlds/"
done

# Let Gazebo resolve model://<name> for our models
export GZ_SIM_RESOURCE_PATH="$PERCH_SIM/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

cd "$PX4_DIR"
PX4_GZ_WORLD=$WORLD exec make px4_sitl "$VEHICLE"
