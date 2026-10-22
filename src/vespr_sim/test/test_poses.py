import math
import random

import pytest

from vespr_sim.poses import add_noise, compose, rotate

IDENTITY = (0.0, 0.0, 0.0, 1.0)


def yaw(angle):
    return (0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2))


def pitch(angle):
    return (0.0, math.sin(angle / 2), 0.0, math.cos(angle / 2))


def test_rotate():
    assert rotate(yaw(math.pi / 2), (1, 0, 0)) == pytest.approx((0, 1, 0))


def test_compose_drone_base_link():
    # x500_hook_0 in the world, base_link 0.24 m up in the model
    drone = ((1.0, 2.0, 0.5), yaw(math.pi / 2))
    position, orientation = compose(drone, ((0.0, 0.0, 0.24), IDENTITY))
    assert position == pytest.approx((1.0, 2.0, 0.74))
    assert orientation == pytest.approx(yaw(math.pi / 2))


def test_perch_rod_point():
    # perch_rod world: model at (3, 0, 0), perch_point at the rod centre, x along the rod
    position, orientation = compose(((3.0, 0.0, 0.0), IDENTITY), ((0.0, 0.0, 1.5), yaw(1.5708)))
    assert position == pytest.approx((3.0, 0.0, 1.5))
    assert rotate(orientation, (1, 0, 0)) == pytest.approx((0, 1, 0), abs=1e-4)


def test_perch_branch_point():
    # perch_branch world: tree at (5, 0, 0) turned 180 deg, perch_point pitched 4.2 deg up the limb
    position, orientation = compose(
        ((5.0, 0.0, 0.0), yaw(3.14159)), ((1.3838, 0.0, 1.8017), pitch(-0.07369)))
    assert position == pytest.approx((3.6162, 0.0, 1.8017), abs=1e-4)
    x_axis = rotate(orientation, (1, 0, 0))
    assert x_axis == pytest.approx((-0.9973, 0.0, 0.0736), abs=1e-3)  # along the limb, climbing
    assert rotate(orientation, (0, 0, 1))[2] > 0.99  # z up


def test_no_noise_is_exact():
    pose = ((1.0, 2.0, 3.0), yaw(0.3))
    assert add_noise(pose, 0.0, 0.0, random.Random(0)) == pose


def test_noise_has_the_requested_spread():
    rng = random.Random(0)
    pose = ((0.0, 0.0, 0.0), IDENTITY)
    xs = [add_noise(pose, 0.01, 0.0, rng)[0][0] for _ in range(5000)]
    spread = math.sqrt(sum(x * x for x in xs) / len(xs))
    assert spread == pytest.approx(0.01, rel=0.1)

    angles = [2 * math.acos(min(1.0, add_noise(pose, 0.0, 0.02, rng)[1][3])) for _ in range(2000)]
    # The angle of a 3-axis Gaussian rotation averages sigma * sqrt(8 / pi)
    assert sum(angles) / len(angles) == pytest.approx(0.02 * math.sqrt(8 / math.pi), rel=0.1)
