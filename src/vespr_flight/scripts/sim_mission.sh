#!/usr/bin/env bash
# Run the whole perch mission in the sim, in one terminal:
#   DDS agent, PX4 + Gazebo (with its window), fake branch detector, sim legs, mission node,
#   then send one /vespr/perch goal. Ctrl+C stops everything, including anything still flying.
#
# Usage (build first: colcon build, from ~/vespr):
#   src/vespr_flight/scripts/sim_mission.sh                 # perch_rod, the default goal
#   NO_GOAL=1 src/vespr_flight/scripts/sim_mission.sh       # start everything, send goals yourself
#   PARAMS=~/my_settings.yaml src/vespr_flight/scripts/sim_mission.sh   # your mission settings
#   HEADLESS=1 ...                                          # no Gazebo window
# Goal: GOAL='{north: 0.3, east: 0.8, altitude: 1.3, heading: 60.0, perch_time: 10.0}'
# Logs of every part: /tmp/vespr_sim_mission/ (LOGS=... to change).
VESPR=$(cd "$(dirname "$0")/../../.." && pwd)
# shellcheck disable=SC1091
source "$VESPR/install/setup.bash"
set -u
WORLD=${WORLD:-perch_rod}
LOGS=${LOGS:-/tmp/vespr_sim_mission}
GOAL=${GOAL:-'{north: 0.3, east: 0.8, altitude: 1.3, heading: 60.0, perch_time: 10.0}'}
mkdir -p "$LOGS"

if pgrep -x MicroXRCEAgent > /dev/null || pgrep -f '[b]in/px4' > /dev/null; then
  echo "A DDS agent or PX4 is already running. Stop it first (it would hold port 8888)."
  exit 1
fi

groups=()
# Start a command in its own process group, so stopping it stops everything it started.
# Background jobs start with SIGINT ignored, and a ros2 launch ignoring SIGINT never stops its
# nodes, so SIGINT is restored before running the command.
start() {
  local name=$1; shift
  setsid python3 -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])' \
    "$@" > "$LOGS/$name.log" 2>&1 &
  groups+=("$!")
  echo "started $name   (log: $LOGS/$name.log)"
}

stop() {
  trap - EXIT INT TERM
  echo; echo "stopping everything..."
  local g
  for ((i = ${#groups[@]} - 1; i >= 0; i--)); do kill -INT -- "-${groups[i]}" 2> /dev/null; done
  for _ in $(seq 10); do
    local alive=0
    for g in "${groups[@]}"; do kill -0 -- "-$g" 2> /dev/null && alive=1; done
    [ $alive = 0 ] && break
    sleep 1
  done
  for g in "${groups[@]}"; do kill -KILL -- "-$g" 2> /dev/null; done
  echo "stopped"
}
trap stop EXIT
trap 'exit 130' INT TERM

wait_for() {  # wait_for <log> <text> <seconds>
  for _ in $(seq "$3"); do grep -q "$2" "$LOGS/$1.log" 2> /dev/null && return 0; sleep 1; done
  echo "timed out waiting for '$2' in $LOGS/$1.log"; exit 1
}

start agent MicroXRCEAgent udp4 -p 8888
# PX4 needs a stdin that stays open, or its shell spins and fills the log
start sim bash -c "tail -f /dev/null | exec '$VESPR/src/perch_sim/scripts/run_sim.sh' $WORLD gz_x500_hook"
echo "waiting for PX4 (Ready for takeoff)..."
wait_for sim "Ready for takeoff" 180
start detector ros2 launch vespr_sim fake_branch_detector.launch.py world:="$WORLD"
start legs ros2 launch vespr_legs sim.launch.py
start mission ros2 launch vespr_flight mission.launch.py ${PARAMS:+params_file:="$PARAMS"}
echo "waiting for the mission node..."
wait_for mission "PX4 ready" 60

# Show the mission's own log here: each state, and each step it carries out
tail -n 0 -f "$LOGS/mission.log" | grep --line-buffered -E "State:|: (Goto|MoveBody|YawBy)|Perch|WARN|ERROR" \
  | sed -u 's/^\[mission-1\] //; s/\[[0-9.]*\] //' &
groups+=("$!")

if [ -n "${NO_GOAL:-}" ]; then
  echo; echo "Ready. Send a goal from another terminal:"
  echo "  ros2 action send_goal /vespr/perch vespr_interfaces/action/Perch \"$GOAL\""
else
  echo; echo "sending goal: $GOAL"
  ros2 action send_goal /vespr/perch vespr_interfaces/action/Perch "$GOAL" 2>&1 | grep -A3 "success"
  echo "Goal finished. Everything keeps running so you can look around."
fi
echo "Ctrl+C to stop everything."
while true; do sleep 1; done
