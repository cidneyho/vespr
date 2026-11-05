#!/usr/bin/env bash
# Start PX4 SITL in one of our perch_sim worlds.
# Usage: run_sim.sh [world] [vehicle]   (defaults: perch_rod gz_x500)
# vehicle is a PX4 one (gz_x500, gz_x500_depth, ...) or one of ours in models/ (gz_x500_hook).
# Our vehicles use their own airframe from airframes/ when there is one (else PX4's plain X500).
# gz_x500_hook flies indoors on optical flow and its rangefinder; GPS=1 turns GPS back on:
#   GPS=1 run_sim.sh perch_rod gz_x500_hook
# Each run starts from PX4's default parameters. PX4 otherwise keeps every parameter it saved
# last run, including magnetometer offsets it learns in flight: Gazebo's magnetometer has no
# offset, so the saved ones turn the heading by several degrees (perch_eval, 2026-11-05).
# KEEP_PARAMS=1 keeps them, e.g. after setting a parameter by hand to use in the next run.
set -euo pipefail

WORLD=${1:-perch_rod}
VEHICLE=${2:-gz_x500}
PERCH_SIM=$(cd "$(dirname "$0")/.." && pwd)
PX4_DIR=${PX4_DIR:-$HOME/PX4-Autopilot}

if [ -z "${KEEP_PARAMS:-}" ]; then
  rm -f "$PX4_DIR"/build/px4_sitl_default/rootfs/parameters.bson \
        "$PX4_DIR"/build/px4_sitl_default/rootfs/parameters_backup.bson
fi

# PX4 only loads worlds from its own worlds directory, so link ours in
for world in "$PERCH_SIM"/worlds/*.sdf; do
  ln -sf "$world" "$PX4_DIR/Tools/simulation/gz/worlds/"
done

# Let Gazebo resolve model://<name> for our models
export GZ_SIM_RESOURCE_PATH="$PERCH_SIM/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

MODEL=${VEHICLE#gz_}
if [ -f "$PERCH_SIM/models/$MODEL/model.sdf" ]; then
  # One of our vehicles. PX4 spawns vehicles from its own models directory, so link it in.
  # The make target for an airframe forces its own model, so run PX4 directly with
  # the airframe and our model instead.
  ln -sfn "$PERCH_SIM/models/$MODEL" "$PX4_DIR/Tools/simulation/gz/models/$MODEL"
  cd "$PX4_DIR"
  [ -x build/px4_sitl_default/bin/px4 ] || make px4_sitl
  # Our airframes too: PX4 reads them from its build folder
  for airframe in "$PERCH_SIM"/airframes/*; do
    ln -sf "$airframe" build/px4_sitl_default/etc/init.d-posix/airframes/
  done
  # Our Gazebo plugins (gz_plugins/), built against PX4's build: the flow sensor
  PLUGINS=build/px4_sitl_default/perch_gz_plugins
  cmake -S "$PERCH_SIM/gz_plugins/optical_flow" -B "$PLUGINS/optical_flow" -DPX4_DIR="$PX4_DIR" > /dev/null
  cmake --build "$PLUGINS/optical_flow" > /dev/null
  export GZ_SIM_SYSTEM_PLUGIN_PATH="$PX4_DIR/$PLUGINS/optical_flow${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
  AIRFRAME=4001
  for airframe in "$PERCH_SIM"/airframes/*_"$VEHICLE"; do
    if [ -e "$airframe" ]; then AIRFRAME=$(basename "$airframe" | cut -d_ -f1); fi
  done
  export PX4_GPS=${GPS:-}  # read by our airframe files
  echo "run_sim: $VEHICLE in $WORLD, airframe $AIRFRAME${GPS:+, GPS=$GPS}"
  PX4_SYS_AUTOSTART=$AIRFRAME PX4_SIM_MODEL=$VEHICLE PX4_GZ_WORLD=$WORLD \
    exec build/px4_sitl_default/bin/px4
fi

cd "$PX4_DIR"
PX4_GZ_WORLD=$WORLD exec make px4_sitl "$VEHICLE"
