"""The planners, fed made-up situations. No ROS, no sim."""
import math

import pytest

from vespr_flight.planning.planner import make
from vespr_flight.planning.steps import (Abort, Done, Goto, Grasp, MoveBody, Situation, Wait,
                                         YawBy, duration)


def yaw_q(degrees):
    """Quaternion for a turn about z, anticlockwise positive (ROS)."""
    a = math.radians(degrees)
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


class FakeWorld:
    """A drone that carries out steps exactly, and a detector that sees the branch.

    World frame: x forward at the start, y left, z up. The branch's axis runs along `branch_yaw`.
    """

    def __init__(self, drone=(0.0, 0.0, 1.0), drone_yaw=0.0, branch=(2.0, 0.0, 1.5),
                 branch_yaw=90.0, fov=34.5):
        self.drone, self.yaw = list(drone), drone_yaw
        self.branch, self.branch_yaw, self.fov = branch, branch_yaw, fov
        self.time = 0.0

    def sighting(self):
        """The branch in base_link, x pointing left, or None if outside the camera's view."""
        dx, dy, dz = (b - d for b, d in zip(self.branch, self.drone))
        a = math.radians(self.yaw)
        x, y = dx * math.cos(a) + dy * math.sin(a), -dx * math.sin(a) + dy * math.cos(a)
        if x < 0.3 or abs(math.degrees(math.atan2(y, x))) > self.fov:
            return None
        axis = math.remainder(self.branch_yaw - self.yaw, 360.0)
        if math.sin(math.radians(axis)) < 0:  # the detector points x left
            axis += 180.0
        return (x, y, dz), yaw_q(axis)

    def situation(self):
        branch = self.sighting()
        return Situation(time=self.time, branch=branch, branch_time=self.time if branch else None)

    def carry_out(self, step):
        if isinstance(step, MoveBody):
            a = math.radians(self.yaw)
            self.drone[0] += step.forward * math.cos(a) + step.right * math.sin(a)
            self.drone[1] += step.forward * math.sin(a) - step.right * math.cos(a)
            self.drone[2] += step.up
        elif isinstance(step, YawBy):
            self.yaw -= step.degrees  # yaw_by is clockwise positive
        self.time += duration(step)


def run(planner, world, limit=200):
    """Ask for steps every 0.1 s until Done or Abort. Returns the steps that moved the drone."""
    planner.start(world.situation())
    moves = []
    for _ in range(limit):
        step = planner.next_step(world.situation())
        if isinstance(step, (Done, Abort)):
            return moves, step
        if isinstance(step, Wait):
            world.time += 0.1
            continue
        moves.append(step)
        world.carry_out(step)
    raise AssertionError('no Done or Abort')


# ---- make ----

def test_make_by_name():
    assert make('approach', 'standoff', step_length=0.2).step_length == 0.2
    with pytest.raises(ValueError, match='one of standoff'):
        make('approach', 'zigzag')
    with pytest.raises(ValueError, match='unknown phase'):
        make('land', 'standoff')


# ---- Phase 1: straight ----

def test_fly_to_goes_then_turns_then_done():
    planner = make('fly_to', 'straight', target_north=3.0, target_east=4.0, target_altitude=2.0,
                   target_heading=90.0, cruise_speed=0.5, settle_time=0.5)
    here = dict(position=(0.0, 0.0, 1.5), heading=350.0)
    planner.start(Situation(time=0.0, **here))
    assert planner.next_step(Situation(time=0.0, **here)) == Goto(3.0, 4.0, 2.0, 0.5)
    # 5.02 m at 0.5 m/s is about 10 s, plus 0.5 s to settle
    assert planner.next_step(Situation(time=9.0, **here)) == Wait()
    there = dict(position=(3.0, 4.0, 2.0), heading=350.0)
    assert planner.next_step(Situation(time=10.6, **there)) == YawBy(pytest.approx(100.0))
    assert planner.next_step(Situation(time=11.0, **there)) == Wait()
    assert planner.next_step(Situation(time=15.0, **there)) == Done()


def test_fly_to_waits_for_a_position():
    planner = make('fly_to', 'straight')
    planner.start(Situation(time=0.0))
    assert planner.next_step(Situation(time=0.0)) == Wait()


# ---- Phase 2: turn ----

def test_search_done_when_seen_in_a_row():
    world = FakeWorld(drone_yaw=0.0, branch=(2.0, 0.0, 1.5))
    moves, end = run(make('search', 'turn', seen_frames=5), world)
    assert end == Done() and moves == []


def test_search_turns_until_it_sees_the_branch():
    # Branch behind and to the right: clockwise turns of 30° bring it into the ±34.5° view
    world = FakeWorld(drone_yaw=0.0, branch=(-2.0, -1.0, 1.5))
    moves, end = run(make('search', 'turn', search_step=30.0), world)
    assert end == Done()
    assert all(m == YawBy(30.0) for m in moves)
    assert world.sighting() is not None


def test_search_gives_up_after_a_full_turn():
    world = FakeWorld(branch=(10.0, 0.0, 1.5), fov=-1.0)  # never in view
    moves, end = run(make('search', 'turn', search_step=90.0), world)
    assert end == Abort('no branch found')
    assert sum(m.degrees for m in moves) == 360.0


# ---- Phase 3: standoff ----

def capture_error(world, capture=(0.0, 0.0, 0.30)):
    """Where the branch ended up in base_link, minus where it should be."""
    dx, dy, dz = (b - d for b, d in zip(world.branch, world.drone))
    a = math.radians(world.yaw)
    x, y = dx * math.cos(a) + dy * math.sin(a), -dx * math.sin(a) + dy * math.cos(a)
    return math.dist((x, y, dz), capture)


@pytest.mark.parametrize('drone_yaw, branch_yaw, branch', [
    (0.0, 90.0, (2.0, 0.0, 1.5)),     # straight ahead, already across the nose
    (0.0, 70.0, (2.0, 0.4, 1.6)),     # off to the left and skewed
    (10.0, 120.0, (2.2, -0.3, 1.3)),  # to the right, skewed the other way
])
def test_approach_ends_with_the_branch_on_the_capture_point(drone_yaw, branch_yaw, branch):
    world = FakeWorld(drone_yaw=drone_yaw, branch=branch, branch_yaw=branch_yaw)
    moves, end = run(make('approach', 'standoff'), world)
    assert end == Done()
    assert moves[-1] == Grasp()
    assert capture_error(world) < 0.06
    # Branch straight across the nose at the end
    assert math.remainder(world.branch_yaw - world.yaw, 180.0) == pytest.approx(90.0, abs=5.0) \
        or math.remainder(world.branch_yaw - world.yaw, 180.0) == pytest.approx(-90.0, abs=5.0)
    # Every approach move is short and slow; only the final one is at final speed
    approach = [m for m in moves if isinstance(m, MoveBody)]
    assert all(math.dist((0, 0, 0), (m.forward, m.right, m.up)) <= 0.3 + 1e-9 for m in approach[:-1])
    assert all(m.speed == 0.2 for m in approach[:-1]) and approach[-1].speed == 0.1


def test_approach_turns_the_short_way():
    planner = make('approach', 'standoff')
    world = FakeWorld(branch_yaw=70.0)  # branch axis 20° clockwise of across-the-nose
    planner.start(world.situation())
    step = planner.next_step(world.situation())
    assert step == YawBy(pytest.approx(20.0))


def test_final_move_is_from_the_standoff_point():
    # Drone already at the standoff point: branch 0.6 m ahead of the capture point
    world = FakeWorld(drone=(0.0, 0.0, 1.0), branch=(0.6, 0.0, 1.3))
    planner = make('approach', 'standoff')
    planner.start(world.situation())
    step = planner.next_step(world.situation())
    assert step == MoveBody(pytest.approx(0.6), pytest.approx(0.0), pytest.approx(0.0), 0.1)
    # Grasp only once the 6 s move has ended and the drone has settled
    assert planner.next_step(Situation(time=world.time + 3.0)) == Wait()
    world.carry_out(step)
    world.time += 0.5
    assert planner.next_step(world.situation()) == Grasp()
    assert planner.next_step(world.situation()) == Done()


def test_approach_waits_for_a_sighting_after_each_move():
    world = FakeWorld()
    planner = make('approach', 'standoff', settle_time=0.5)
    planner.start(world.situation())
    step = planner.next_step(world.situation())
    assert isinstance(step, MoveBody)
    # A sighting taken during the move is not used
    mid = Situation(time=world.time + duration(step), branch=world.sighting(), branch_time=world.time + 0.1)
    assert planner.next_step(mid) == Wait()


def test_approach_aborts_when_the_branch_is_lost():
    world = FakeWorld()
    planner = make('approach', 'standoff', lost_timeout=2.0)
    planner.start(world.situation())
    assert planner.next_step(Situation(time=1.0)) == Wait()
    assert planner.next_step(Situation(time=2.5)) == Abort('lost the branch')


def test_approach_aborts_when_the_branch_is_too_far():
    world = FakeWorld(branch=(5.0, 0.0, 1.5))
    _, end = run(make('approach', 'standoff', max_range=3.0), world)
    assert isinstance(end, Abort) and 'too far' in end.reason


def test_approach_from_many_starts():
    """Branch anywhere in view, at any skew: the approach always ends on the capture point."""
    import random
    rng = random.Random(5)
    for _ in range(200):
        distance, bearing = rng.uniform(1.0, 2.8), rng.uniform(-30.0, 30.0)
        branch = (distance * math.cos(math.radians(bearing)), distance * math.sin(math.radians(bearing)),
                  1.0 + rng.uniform(-0.3, 0.5))
        world = FakeWorld(branch=branch, branch_yaw=rng.uniform(0.0, 180.0))
        moves, end = run(make('approach', 'standoff'), world, limit=1000)
        assert end == Done(), (branch, world.branch_yaw, end)
        assert capture_error(world) < 0.06, (branch, world.branch_yaw)
