"""Fly the flight accuracy eval's test path once, through Commander, then land.

The same path every run (docs/design/flight_accuracy.md): straight moves, a climb and descent, and
turns in place, with a hold after each. Positions are relative to the takeoff spot; the path stays
within 1.5 m of it and climbs to altitude + climb. In the sim it fits the perch_rod world without
reaching the rod. Start PX4 and the Micro XRCE-DDS Agent first (sim or real drone);
scripts/flight_accuracy_runs.sh does it all, N times, with recording.
    ros2 run perch_eval flight_path [--altitude 1.0] [--climb 0.5] [--turn 90] [--wait 10]
--climb 0 leaves out the climb and descent, --turn 0 the turns (the nose stays north), --wait
holds on the ground that long after PX4 is ready, before taking off. If the pilot takes over, it
stops commanding and waits for them to land. Exits 0 once landed and disarmed, 1 if any step
failed (the drone is landed first if flying).
"""
import argparse
import sys
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor

from perch_flight.commander import Commander, Refused, State

ALTITUDE = 1.0   # m above the takeoff spot (default)
CLIMB = 0.5      # m, the climb and descent near the end (default)
TURN = 90.0      # deg, the turn after the east move and back at the end (default)
WAIT = 10.0      # s on the ground after PX4 is ready, before taking off (default)
SPEED = 0.3      # m/s, in the middle of the mission's 0.1 to 0.5
HOLD = 5.0       # s to hold after each step, for settling
PX4_TIMEOUT = 60.0
STEP_TIMEOUT = 30.0
LAND_TIMEOUT = 60.0
PILOT_TIMEOUT = 600.0


def path(altitude, climb, turn):
    """[(name, Commander method, arguments)]. goto is (north, east, altitude)"""
    steps = [
        ('north 1 m', 'goto', (1.0, 0.0, altitude)),
        ('east 1 m', 'goto', (1.0, 1.0, altitude)),
        ('south 1 m', 'goto', (0.0, 1.0, altitude)),
        ('west 1 m', 'goto', (0.0, 0.0, altitude)),
    ]
    if turn:
        steps.insert(2, (f'turn +{turn:g}', 'yaw_by', (turn,)))
    if climb:
        steps += [
            (f'up {climb:g} m', 'goto', (0.0, 0.0, altitude + climb)),
            (f'down {climb:g} m', 'goto', (0.0, 0.0, altitude)),
        ]
    if turn:
        steps.append((f'turn -{turn:g}', 'yaw_by', (-turn,)))
    return steps


class Failed(Exception):
    pass


def wait_for(condition, timeout, what):
    deadline = time.monotonic() + timeout
    while not condition():
        if time.monotonic() > deadline:
            raise Failed(f'timed out after {timeout:.0f} s waiting for {what}')
        time.sleep(0.05)


def state(commander):
    with commander.lock:
        return commander.state


def step_done(commander):
    """The current move or turn has reached its end (its target, not the drone)."""
    with commander.lock:
        if commander.state != State.FLYING:
            raise Failed(f'left flying mid-path (state: {commander.state.value})')
        return commander.trajectory.done(commander.now())


def fly(commander, log, altitude, climb, turn, wait):
    wait_for(lambda: state(commander) == State.IDLE, PX4_TIMEOUT, 'PX4')
    if wait:
        log(f'PX4 ready: waiting {wait:.0f} s before taking off')
        time.sleep(wait)
    log(commander.set_speed(SPEED))
    log(commander.takeoff(altitude))
    wait_for(lambda: state(commander) == State.FLYING, STEP_TIMEOUT, 'arming and offboard')
    wait_for(lambda: step_done(commander), STEP_TIMEOUT, 'the climb')
    log(f'takeoff: holding {HOLD:.0f} s')
    time.sleep(HOLD)
    for name, method, args in path(altitude, climb, turn):
        log(f'{name}: {getattr(commander, method)(*args)}')
        wait_for(lambda: step_done(commander), STEP_TIMEOUT, name)
        time.sleep(HOLD)


def land(commander, log):
    if state(commander) == State.PILOT:
        # The pilot took over (RC switch or failsafe): leave it to them, don't command a landing
        log('pilot has control: waiting for them to land and disarm')
        wait_for(lambda: state(commander) == State.IDLE, PILOT_TIMEOUT, 'the pilot to disarm')
    else:
        if state(commander) in (State.STARTING, State.FLYING):
            log(commander.land())
        wait_for(lambda: state(commander) == State.IDLE, LAND_TIMEOUT, 'landing and disarming')
    log('landed and disarmed')


def main():
    parser = argparse.ArgumentParser(description='Fly the flight accuracy test path once.')
    parser.add_argument('--altitude', type=float, default=ALTITUDE, help='m above takeoff')
    parser.add_argument('--climb', type=float, default=CLIMB, help='m; 0 leaves it out')
    parser.add_argument('--turn', type=float, default=TURN, help='deg; 0 leaves the turns out')
    parser.add_argument('--wait', type=float, default=WAIT,
                        help='s on the ground before taking off')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init()
    commander = Commander()
    executor = MultiThreadedExecutor()
    executor.add_node(commander)
    threading.Thread(target=executor.spin, daemon=True).start()
    log = commander.get_logger().info
    ok = True
    try:
        fly(commander, log, args.altitude, args.climb, args.turn, args.wait)
    except (Failed, Refused) as e:
        commander.get_logger().error(f'path failed: {e}')
        ok = False
    try:
        land(commander, log)
    except Failed as e:
        commander.get_logger().error(str(e))
        ok = False
    executor.shutdown()
    commander.destroy_node()
    rclpy.try_shutdown()
    sys.exit(0 if ok else 1)
