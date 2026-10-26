"""The perch cycle: the mission's states and aborts, without ROS. See docs/design/mission.md.

PerchCycle.run() flies one goal from takeoff to landing. It uses:
    flight    the flight Commander: its methods, and its state, arrived and position
    legs      enable(on), grasp() -> (success, gripped, message), release() -> (success, message)
    planners  {'fly_to': ..., 'search': ..., 'approach': ...}, fresh for this run
    sighting  () -> (branch pose in base_link, time on Commander's clock), or (None, None)
    report    (state) -> None, called on every state change
    cancelled () -> bool, true once the operator has asked to cancel
    log       (text) -> None, for each step carried out
"""
from dataclasses import dataclass
import math
import time

from perch_flight.commander import Refused
from vespr_flight.planning.steps import (Abort, Done, Goto, Grasp, MoveBody, Situation, Wait,
                                         YawBy, duration)

POLL = 0.1          # s between checks while waiting
MOVE_TIMEOUT = 60.0  # s a move may stay refused as "still moving" before giving up


@dataclass
class Settings:
    takeoff_timeout: float = 30.0     # s to reach the altitude after taking off
    land_settle: float = 2.0          # s in land mode before disarming on the branch
    motors_off_timeout: float = 5.0   # s for PX4 to disarm on the branch
    land_timeout: float = 60.0        # s to land and disarm
    back_out: float = 0.6             # m to back straight out when no final move was made
    back_out_speed: float = 0.1       # m/s
    pre_launch_lift: float = 0.03     # m above where it hangs to aim for on re-arming. Any climb
                                      # starts PX4's takeoff; the latch stops the drone rising far
    pre_launch_time: float = 5.0      # s for the thrust to build, still latched, before releasing
    climb_out_lift: float = 0.15      # m to rise while backing out, to clear the hook's overhang


class Failed(Exception):
    """This state can't go on. The message says why."""


class Cancelled(Exception):
    pass


class PilotTookOver(Exception):
    pass


class PerchCycle:

    def __init__(self, flight, legs, planners, sighting, report, cancelled, settings=None,
                 sleep=time.sleep, log=lambda text: None):
        self.flight = flight
        self.legs = legs
        self.planners = planners
        self.sighting = sighting
        self.report = report
        self.cancelled = cancelled
        self.settings = settings or Settings()
        self.sleep = sleep
        self.log = log
        self.state = 'ground'
        self.final_move = None       # the approach's last move, reversed to climb out
        self.ignore_cancel = False   # once a cancel is being acted on
        self.legs_on = False

    def run(self, goal):
        """(success, state, message) for one goal: north, east, altitude, heading, perch_time."""
        try:
            self.cycle(goal)
            return True, 'land', 'perched and came back down'
        except PilotTookOver:
            return False, self.state, 'pilot took control'
        except (Failed, Cancelled) as e:
            reason = 'cancelled' if isinstance(e, Cancelled) else str(e)
            if self.state == 'perched' and isinstance(e, Cancelled):
                return self.leave_after_cancel()
            return False, self.state, f'{reason}. {self.abort()}'

    def cycle(self, goal):
        self.enter('takeoff')
        self.call(self.flight.takeoff, goal.altitude)
        self.wait_flying('could not take off')

        self.enter('fly_to')
        self.fly(self.planners['fly_to'])
        self.enter('search')
        self.fly(self.planners['search'])

        self.enter('approach')
        self.legs_enable(True)
        success, message = self.legs.release()  # open, ready to grasp
        if not success:
            raise Failed(f'legs did not open: {message}')
        self.fly(self.planners['approach'])

        self.enter('grasp')
        success, gripped, message = self.legs.grasp()
        if not (success and gripped):
            raise Failed(f'grasp failed: {message}')

        self.enter('motors_off')
        self.motors_off()

        self.enter('perched')
        self.wait(goal.perch_time)
        self.leave()

    def leave(self):
        """pre_launch, release, climb_out, land: from hanging disarmed back to the ground."""
        # Arm and aim a little higher, still latched, so PX4 takes the weight before the release
        self.enter('pre_launch')
        with self.flight.lock:
            altitude = -self.flight.position.z
        self.call(self.flight.takeoff, altitude + self.settings.pre_launch_lift)
        self.wait_flying('could not arm on the branch')
        self.wait(self.settings.pre_launch_time)

        self.enter('release')
        success, message = self.legs.release()
        if not success:
            raise Failed(f'release failed: {message}')

        self.enter('climb_out')
        self.back_out(self.settings.climb_out_lift)

        self.enter('land')
        self.land()
        self.legs_enable(False)

    def leave_after_cancel(self):
        self.ignore_cancel = True
        try:
            self.leave()
            return False, 'land', 'cancelled while perched: left the branch and landed'
        except PilotTookOver:
            return False, self.state, 'cancelled while perched, then the pilot took control'
        except Failed as e:
            return False, self.state, f'cancelled while perched, then {e}. {self.abort()}'

    # ---- aborts ----

    def abort(self):
        """The abort move for the state we're in. Returns what was done."""
        try:
            if self.state in ('takeoff', 'fly_to', 'search', 'climb_out'):
                return self.land_and_disable('landed')
            if self.state == 'approach':
                self.back_out()
                return self.land_and_disable('backed out and landed')
            if self.state == 'grasp':
                self.legs.release()
                self.back_out()
                return self.land_and_disable('released, backed out and landed')
            if self.state == 'motors_off':
                return 'still hanging on the branch in land mode. Someone has to come'
            if self.state == 'pre_launch':
                return 'still latched on the branch, disarmed. Someone has to come'
            if self.state == 'release':
                self.motors_off()
                return 'stuck on the branch, motors off. Someone has to come'
            if self.state == 'land':
                return 'did not finish landing'
            return 'nothing to undo'
        except PilotTookOver:
            return 'the pilot took control during the abort'
        except Failed as e:
            return f'the abort failed too: {e}'

    def land_and_disable(self, done):
        if self.flight.state.value in ('starting', 'flying'):
            self.call(self.flight.land)
        if self.flight.state.value != 'idle':
            self.wait_until(lambda: self.flight.state.value == 'idle', self.settings.land_timeout,
                            'did not land in time')
        self.legs_enable(False)
        return done

    # ---- pieces ----

    def fly(self, planner):
        """Carry out a planner's steps until it says Done (or Grasp). Abort raises Failed."""
        planner.start(self.situation())
        while True:
            self.check()
            step = planner.next_step(self.situation())
            if isinstance(step, (Done, Grasp)):
                return
            if isinstance(step, Abort):
                raise Failed(step.reason)
            if isinstance(step, Wait):
                self.sleep(POLL)
                continue
            if isinstance(step, MoveBody):
                self.final_move = step  # the last one before Grasp is the final move
            self.move(step)

    def move(self, step):
        self.log(f'{self.state}: {step}')
        if isinstance(step, Goto):
            self.call(self.flight.goto, step.north, step.east, step.altitude, step.speed)
        elif isinstance(step, MoveBody):
            self.call(self.flight.move_body, step.forward, step.right, step.up, step.speed)
        elif isinstance(step, YawBy):
            self.call(self.flight.yaw_by, step.degrees)

    def back_out(self, lift=0.0):
        """Back out the way the final move came in, rising by `lift` on the way, and wait for it.

        Hanging, the branch rests under the hook's overhang: rising while backing out clears it.
        """
        if self.flight.state.value != 'flying':
            return
        self.flight.hold()
        if self.final_move is not None:
            m = self.final_move
            back = MoveBody(-m.forward, -m.right, -m.up + lift, m.speed)
        else:
            back = MoveBody(-self.settings.back_out, 0.0, lift, self.settings.back_out_speed)
        self.move(back)
        self.wait(duration(back) + 0.5)

    def motors_off(self):
        self.call(self.flight.land)
        self.wait(self.settings.land_settle)
        self.call(self.flight.disarm)
        self.wait_until(lambda: self.flight.state.value == 'idle', self.settings.motors_off_timeout,
                        'did not disarm on the branch')

    def land(self):
        self.call(self.flight.land)
        self.wait_until(lambda: self.flight.state.value == 'idle', self.settings.land_timeout,
                        'did not land in time')

    def wait_flying(self, failure):
        def flying():
            state = self.flight.state.value
            if state == 'idle':
                raise Failed(failure)
            return state == 'flying' and self.flight.arrived
        self.wait_until(flying, self.settings.takeoff_timeout, failure)

    def legs_enable(self, on):
        if self.legs_on != on:
            self.legs.enable(on)
            self.legs_on = on

    # ---- helpers ----

    def enter(self, state):
        self.state = state
        self.report(state)

    def situation(self):
        with self.flight.lock:
            p = self.flight.position
            position = None if p is None else (p.x, p.y, -p.z)
            heading = None if p is None else math.degrees(p.heading) % 360.0
        branch, branch_time = self.sighting()
        return Situation(time=self.flight.now(), position=position, heading=heading,
                         branch=branch, branch_time=branch_time)

    def call(self, command, *args):
        """Run a Commander command, retrying while it's refused as still moving."""
        end = self.flight.now() + MOVE_TIMEOUT
        while True:
            self.check()
            try:
                return command(*args)
            except Refused as e:
                if 'still moving' not in str(e) or self.flight.now() > end:
                    raise Failed(f'{command.__name__} refused: {e}') from None
            self.sleep(POLL)

    def check(self):
        if self.flight.state.value == 'pilot':
            raise PilotTookOver()
        if self.cancelled() and not self.ignore_cancel:
            self.ignore_cancel = True  # act on it once
            raise Cancelled()

    def wait(self, seconds):
        end = self.flight.now() + seconds
        while self.flight.now() < end:
            self.check()
            self.sleep(POLL)

    def wait_until(self, condition, timeout, failure):
        end = self.flight.now() + timeout
        while not condition():
            self.check()
            if self.flight.now() > end:
                raise Failed(failure)
            self.sleep(POLL)
