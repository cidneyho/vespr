"""The perch cycle's states and aborts, with a fake Commander, legs and planners. No ROS, no sim."""
from dataclasses import dataclass
import threading
from types import SimpleNamespace

from perch_flight.commander import Refused

from perch_mission.perch_cycle import PerchCycle, Settings
from perch_mission.planning.steps import Abort, Done, Goto, Grasp, MoveBody, Wait, YawBy


class FakeFlight:
    """Commander's interface. Moves happen at once; landing takes `land_time` s of the fake clock."""

    def __init__(self, land_time=3.0, can_disarm=True, can_rearm=True, falls=False):
        self.lock = threading.Lock()
        self.state = SimpleNamespace(value='idle')
        self.arrived = True
        self.position = SimpleNamespace(x=0.0, y=0.0, z=0.0, heading=0.0,
                                        dist_bottom=1.1, dist_bottom_valid=True)
        self.falls = falls  # the hook isn't holding it: it drops once disarmed
        self.time = 0.0
        self.land_time = land_time
        self.can_disarm = can_disarm
        self.can_rearm = can_rearm
        self.calls = []
        self.landing_until = None
        self.refuse_next = None  # a message to refuse the next command with

    def now(self):
        if self.landing_until is not None and self.time >= self.landing_until:
            self.state.value, self.landing_until = 'idle', None
        return self.time

    def record(self, *call):
        if self.refuse_next:
            message, self.refuse_next = self.refuse_next, None
            raise Refused(message)
        self.calls.append(call)

    def takeoff(self, altitude):
        self.record('takeoff', round(altitude, 2))
        armed_before = self.calls.count(('disarm',)) > 0
        if armed_before and not self.can_rearm:
            return 'arming'  # stays idle: never arms
        self.state.value = 'flying'
        self.position.z = -altitude

    def goto(self, *args):
        self.record('goto', *args)

    def move_body(self, *args):
        self.record('move_body', *args)

    def yaw_by(self, degrees):
        self.record('yaw_by', degrees)

    def hold(self):
        self.record('hold')

    def land(self):
        self.record('land')
        self.state.value = 'landing'
        self.landing_until = self.time + self.land_time

    def disarm(self):
        self.record('disarm')
        if self.can_disarm:
            self.landing_until = self.time  # disarmed at once
            if self.falls:
                self.position.dist_bottom = 0.18


class FakeLegs:
    def __init__(self, gripped=True, release_ok=(True,)):
        self.gripped = gripped
        self.release_ok = list(release_ok)  # per release call; the last one repeats
        self.calls = []

    def enable(self, on):
        self.calls.append(('enable', on))

    def grasp(self):
        self.calls.append(('grasp',))
        return True, self.gripped, 'gripped' if self.gripped else 'missed'

    def release(self):
        self.calls.append(('release',))
        ok = self.release_ok.pop(0) if len(self.release_ok) > 1 else self.release_ok[0]
        return ok, 'released' if ok else 'jammed'


class Script:
    """A planner that returns the given steps in order, then Done."""

    def __init__(self, *steps):
        self.steps = list(steps)

    def start(self, situation):
        pass

    def next_step(self, situation):
        return self.steps.pop(0) if self.steps else Done()


FINAL = MoveBody(0.6, 0.02, 0.01, 0.1)


@dataclass
class Goal:
    north: float = 0.3
    east: float = 0.8
    altitude: float = 1.3
    heading: float = 60.0
    perch_time: float = 10.0


def make_cycle(flight=None, legs=None, planners=None, cancel_at=None, pilot_at=None):
    flight = flight or FakeFlight()
    legs = legs or FakeLegs()
    planners = planners or {
        'fly_to': Script(Goto(0.3, 0.8, 1.3, 0.5), YawBy(-30.0)),
        'search': Script(Wait(), YawBy(30.0)),
        'approach': Script(MoveBody(0.3, 0.0, 0.0, 0.2), FINAL, Grasp()),
    }
    states = []

    def report(state):
        states.append(state)
        if state == pilot_at:
            flight.state.value = 'pilot'

    def sleep(seconds):
        flight.time += seconds

    cycle = PerchCycle(flight, legs, planners, sighting=lambda: (None, None), report=report,
                       cancelled=lambda: cancel_at is not None and cancel_at in states,
                       settings=Settings(), sleep=sleep)
    return cycle, flight, legs, states


def test_full_cycle():
    cycle, flight, legs, states = make_cycle()
    success, state, message = cycle.run(Goal())
    assert (success, state) == (True, 'land')
    assert states == ['takeoff', 'fly_to', 'search', 'approach', 'grasp', 'motors_off', 'perched',
                      'pre_launch', 'release', 'climb_out', 'land']
    assert flight.calls == [
        ('takeoff', 1.3), ('goto', 0.3, 0.8, 1.3, 0.5), ('yaw_by', -30.0), ('yaw_by', 30.0),
        ('move_body', 0.3, 0.0, 0.0, 0.2), ('move_body', 0.6, 0.02, 0.01, 0.1),
        ('land',), ('disarm',),                       # motors off on the branch
        ('takeoff', 1.33),                            # pre-launch: arm, aim a little higher
        ('hold',), ('move_body', -0.6, -0.02, -0.01 + 0.15, 0.1),  # climb out: reversed, rising
        ('land',)]
    # Legs enabled only from the approach on, opened first, disabled once landed
    assert legs.calls == [('enable', True), ('release',), ('grasp',), ('release',), ('enable', False)]
    assert flight.state.value == 'idle'


def test_search_abort_lands_and_never_enables_the_legs():
    planners = {'fly_to': Script(), 'search': Script(Abort('no branch found')), 'approach': Script()}
    cycle, flight, legs, states = make_cycle(planners=planners)
    success, state, message = cycle.run(Goal())
    assert (success, state) == (False, 'search')
    assert message == 'no branch found. landed'
    assert flight.calls[-1] == ('land',) and flight.state.value == 'idle'
    assert legs.calls == []


def test_approach_abort_backs_out_then_lands():
    planners = {'fly_to': Script(), 'search': Script(),
                'approach': Script(MoveBody(0.3, 0.1, 0.0, 0.2), Abort('lost the branch'))}
    cycle, flight, legs, states = make_cycle(planners=planners)
    success, state, message = cycle.run(Goal())
    assert (success, state) == (False, 'approach')
    assert message == 'lost the branch. backed out and landed'
    assert flight.calls[-3:] == [('hold',), ('move_body', -0.3, -0.1, -0.0, 0.2), ('land',)]
    assert legs.calls[-1] == ('enable', False)


def test_missed_grasp_releases_backs_out_and_lands():
    cycle, flight, legs, states = make_cycle(legs=FakeLegs(gripped=False))
    success, state, message = cycle.run(Goal())
    assert (success, state) == (False, 'grasp')
    assert message == 'grasp failed: missed. released, backed out and landed'
    assert legs.calls[-2:] == [('release',), ('enable', False)]
    assert flight.calls[-3:] == [('hold',), ('move_body', -0.6, -0.02, -0.01, 0.1), ('land',)]


def test_not_disarming_stays_on_the_branch():
    cycle, flight, legs, states = make_cycle(flight=FakeFlight(land_time=1000.0, can_disarm=False))
    success, state, message = cycle.run(Goal())
    assert (success, state) == (False, 'motors_off')
    assert message.startswith('did not disarm on the branch. still hanging')
    assert flight.calls[-2:] == [('land',), ('disarm',)]


def test_cannot_rearm_stays_latched():
    cycle, flight, legs, states = make_cycle(flight=FakeFlight(can_rearm=False))
    success, state, message = cycle.run(Goal())
    assert (success, state) == (False, 'pre_launch')
    assert 'still latched' in message
    assert ('enable', False) not in legs.calls  # the legs keep holding


def test_jammed_release_turns_the_motors_off_again():
    # Opens fine before the approach, jams when leaving the branch
    cycle, flight, legs, states = make_cycle(legs=FakeLegs(release_ok=(True, False)))
    success, state, message = cycle.run(Goal())
    assert (success, state) == (False, 'release')
    assert 'stuck on the branch, motors off' in message
    assert flight.calls[-2:] == [('land',), ('disarm',)]


def test_cancel_in_flight_lands():
    cycle, flight, legs, states = make_cycle(cancel_at='search')
    success, state, message = cycle.run(Goal())
    assert (success, state, message) == (False, 'search', 'cancelled. landed')
    assert flight.state.value == 'idle'


def test_cancel_when_perched_leaves_normally():
    cycle, flight, legs, states = make_cycle(cancel_at='perched')
    success, state, message = cycle.run(Goal(perch_time=1000.0))
    assert (success, state) == (False, 'land')
    assert message == 'cancelled while perched: left the branch and landed'
    assert states[-4:] == ['pre_launch', 'release', 'climb_out', 'land']
    assert flight.time < 100.0  # didn't sit out the perch time


def test_pilot_takeover_stops_commanding():
    cycle, flight, legs, states = make_cycle(pilot_at='approach')
    success, state, message = cycle.run(Goal())
    assert (success, state, message) == (False, 'approach', 'pilot took control')
    assert ('land',) not in flight.calls


def test_still_moving_is_retried():
    cycle, flight, legs, states = make_cycle()
    flight.refuse_next = 'still moving; wait for it to arrive, or hold to stop'
    success, _, _ = cycle.run(Goal())
    assert success and flight.calls[0] == ('takeoff', 1.3)


def test_other_refusals_abort():
    cycle, flight, legs, states = make_cycle()

    def goto(*args):
        raise Refused('not flying (state: idle)')
    flight.goto = goto
    success, state, message = cycle.run(Goal())
    assert (success, state) == (False, 'fly_to')
    assert message.startswith('goto refused: not flying')


def test_missed_grasp_shows_as_a_fall():
    cycle, flight, legs, states = make_cycle(flight=FakeFlight(falls=True))
    success, state, message = cycle.run(Goal())
    assert (success, state) == (False, 'motors_off')
    assert message == 'fell off the branch: rangefinder reads 0.18 m. on the floor'
    assert 'perched' not in states
    assert legs.calls[-1] == ('enable', False)


def test_falling_while_perched_is_caught():
    cycle, flight, legs, states = make_cycle()
    original = flight.now

    def now():
        if 'perched' in states and flight.time > 20.0:
            flight.position.dist_bottom = 0.18  # slipped off partway through the perch
        return original()
    flight.now = now
    success, state, message = cycle.run(Goal(perch_time=100.0))
    assert (success, state) == (False, 'perched')
    assert message.startswith('fell off the branch')
