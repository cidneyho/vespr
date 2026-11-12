#!/usr/bin/env bash
# Flight accuracy eval (docs/design/flight_accuracy.md): fly the test path N times, recording a
# rosbag of each, then run the analysis on them all. In the sim or on the real drone.
#
# Sim (default): each run in a fresh sim: DDS agent, PX4 + Gazebo, mocap_sim, ros2 bag record,
# then perch_eval flight_path.
# Real drone (REAL=1): starts nothing. PX4 with its DDS agent and the mocap PC must already be
# publishing (/fmu/out/vehicle_status_v1, /mocap/vespr/pose). It asks before each run, then records
# and flies. The path needs 1.5 m clear round the takeoff spot and 2 m of height (1.5 m with
# CLIMB=0). A pilot on the RC can take over at any time: flight_path then stops commanding and
# waits for them to land. The analysis uses that run up to the takeover.
#
# Usage (build first: colcon build, from ~/vespr):
#   src/perch_eval/scripts/flight_accuracy_runs.sh            # 10 runs in the sim
#   RUNS=3 src/perch_eval/scripts/flight_accuracy_runs.sh     # 3 runs
#   REAL=1 RUNS=5 ...         # 5 runs on the real drone
#   ALTITUDE=2 CLIMB=0 ...    # fly the path at 2 m, without the climb and descent
#   TURN=0 ...                # no turns: the nose stays north
#   WAIT=30 ...               # seconds on the ground after PX4 is ready, before taking off (10)
#   GPS=1 ...                 # sim: GPS on (run_sim.sh); off by default
#   BEFORE=<an earlier output folder> ...   # also compare with that set: before_after.png
# Sim: Gazebo opens its window each run: the hook drone's camera (and so its optical flow) needs
# rendering, which headless only gives with GPU rendering (perch_sim/README.md).
# Bags and logs: ~/vespr_recordings/flight_accuracy/<date and time>/ (OUT=... to change).
# Results (JSON, summary.md, plots): <that folder>/results/
VESPR=$(cd "$(dirname "$0")/../../.." && pwd)
# shellcheck disable=SC1091
source "$VESPR/install/setup.bash"
set -u
RUNS=${RUNS:-10}
ALTITUDE=${ALTITUDE:-1.0}
CLIMB=${CLIMB:-0.5}
TURN=${TURN:-90}
WAIT=${WAIT:-10}
REAL=${REAL:-}
BEFORE=${BEFORE:-}
export GPS=${GPS:-}
ROOTFS=${PX4_DIR:-$HOME/PX4-Autopilot}/build/px4_sitl_default/rootfs  # PX4's ulogs are here
WORLD=perch_rod
OUT=${OUT:-$HOME/vespr_recordings/flight_accuracy/$(date +%Y-%m-%dT%H-%M-%S)}
TOPICS=(/fmu/in/trajectory_setpoint /fmu/out/vehicle_local_position /fmu/out/vehicle_status_v1
        /mocap/vespr/pose /fmu/out/estimator_status_flags)
mkdir -p "$OUT"
echo "${REAL:+real drone, }altitude $ALTITUDE m, climb $CLIMB m, turn $TURN deg, wait $WAIT s, GPS ${GPS:-off}" \
  | tee "$OUT/settings.txt"

if [ -n "$REAL" ]; then
  for topic in /fmu/out/vehicle_status_v1 /mocap/vespr/pose; do
    if ! timeout 10 ros2 topic echo --once "$topic" > /dev/null 2>&1; then
      echo "Nothing on $topic. Start PX4's DDS agent and the mocap PC first."
      exit 1
    fi
  done
elif pgrep -x MicroXRCEAgent > /dev/null || pgrep -f '[b]in/px4' > /dev/null; then
  echo "A DDS agent or PX4 is already running. Stop it first (it would hold port 8888)."
  exit 1
fi

groups=()
# As in sim_mission.sh: each command in its own process group, with SIGINT restored
start() {
  local name=$1; shift
  setsid python3 -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])' \
    "$@" > "$LOGS/$name.log" 2>&1 &
  groups+=("$!")
}

stop_all() {
  local g
  for ((i = ${#groups[@]} - 1; i >= 0; i--)); do kill -INT -- "-${groups[i]}" 2> /dev/null; done
  for _ in $(seq 15); do
    local alive=0
    for g in "${groups[@]}"; do kill -0 -- "-$g" 2> /dev/null && alive=1; done
    [ $alive = 0 ] && break
    sleep 1
  done
  for g in "${groups[@]}"; do kill -KILL -- "-$g" 2> /dev/null; done
  groups=()
}
trap 'stop_all' EXIT
trap 'exit 130' INT TERM

wait_for() {  # wait_for <log> <text> <seconds>
  for _ in $(seq "$3"); do grep -q "$2" "$LOGS/$1.log" 2> /dev/null && return 0; sleep 1; done
  echo "  timed out waiting for '$2' in $LOGS/$1.log"; return 1
}

# Record, fly the path once, stop recording. Run's logs in $LOGS
record_and_fly() {
  start bag ros2 bag record -o "$OUT/run_$run" "${TOPICS[@]}" && wait_for bag "Recording..." 30 \
    || return 1
  sleep 3  # the bag's subscriptions match, and mocap and PX4 rest before arming
  ros2 run perch_eval flight_path --altitude "$ALTITUDE" --climb "$CLIMB" --turn "$TURN" --wait "$WAIT" > "$LOGS/flight.log" 2>&1
  echo "  flight_path exited $? (log: $LOGS/flight.log)"
  sleep 1
}

bags=()
for run in $(seq -f '%02g' 1 "$RUNS"); do
  LOGS="$OUT/run_$run.logs"
  mkdir -p "$LOGS"
  echo "run $run of $RUNS (logs: $LOGS)"
  if [ -n "$REAL" ]; then
    read -r -p "  Drone on the takeoff spot, area clear, pilot ready? Enter to fly, q to stop: " answer
    [ "$answer" = q ] && break
    record_and_fly && bags+=("$OUT/run_$run")
    stop_all
    continue  # PX4's flight log (ulog) is on the flight controller's SD card
  fi
  touch "$LOGS/.started"
  start agent MicroXRCEAgent udp4 -p 8888
  # PX4 needs a stdin that stays open, or its shell spins and fills the log
  start sim bash -c "tail -f /dev/null | exec '$VESPR/src/perch_sim/scripts/run_sim.sh' $WORLD gz_x500_hook"
  if wait_for sim "Ready for takeoff" 180 \
      && start mocap ros2 launch perch_sim mocap_sim.launch.py world:="$WORLD" \
      && wait_for mocap "Branch perch_point at" 60 \
      && record_and_fly; then
    bags+=("$OUT/run_$run")
  else
    echo "  run $run did not start; skipping it"
  fi
  stop_all
  # PX4's own flight log (ulog) of this run, for the estimator's internals
  find "$ROOTFS/log" -name '*.ulg' -newer "$LOGS/.started" -exec cp {} "$LOGS/" \; 2> /dev/null
  sleep 2
done

[ ${#bags[@]} = 0 ] && { echo "no runs recorded"; exit 1; }
echo "analysing ${#bags[@]} bags..."
compare=()
if [ -n "$BEFORE" ]; then
  compare=(--before "$BEFORE"/run_?? --names "$(basename "$BEFORE")" "$(basename "$OUT")")
fi
ros2 run perch_eval flight_accuracy "${bags[@]}" --out "$OUT/results" "${compare[@]}"
echo "results: $OUT/results"
