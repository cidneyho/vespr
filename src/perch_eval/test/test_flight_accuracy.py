"""Tests for flight_accuracy.py's maths: lining mocap up with PX4, the parts of an error, settling,
and the before/after comparison.

No bags and no sim: the inputs are built by hand. Skipped without numpy.
"""
import math

import pytest

np = pytest.importorskip('numpy')

from perch_eval import flight_accuracy as fa  # noqa: E402  (after the skip)


def test_align_recovers_the_frame():
    """Mocap (z up) put through a known turn and offset lines up with PX4's NED again."""
    yaw_offset, offset = math.radians(37.0), np.array([0.4, -1.2, 0.08])
    rng = np.random.default_rng(1)
    truth = rng.normal(0.0, 1.0, (50, 3))
    truth_yaw = rng.uniform(-math.pi, math.pi, 50)
    estimate = fa.to_ned(truth, yaw_offset, offset)
    est_yaw = fa.yaw_to_ned(truth_yaw, yaw_offset)
    # align() sees only the drone at rest: one place, one heading
    found_yaw, found_offset = fa.align(truth[:1].repeat(5, axis=0), truth_yaw[:1].repeat(5),
                                       estimate[:1].repeat(5, axis=0), est_yaw[:1].repeat(5))
    assert math.isclose(found_yaw, yaw_offset, abs_tol=1e-9)
    assert np.allclose(fa.to_ned(truth, found_yaw, found_offset), estimate)
    assert np.allclose(fa.wrap(fa.yaw_to_ned(truth_yaw, found_yaw) - est_yaw), 0.0)


def test_mocap_up_is_ned_down():
    assert np.allclose(fa.to_ned([[0.0, 0.0, 1.0]], 0.0, np.zeros(3)), [[0.0, 0.0, -1.0]])


def test_align_averages_heading_across_the_wrap():
    found, _ = fa.align(np.zeros((2, 3)), [0.0, 0.0], np.zeros((2, 3)),
                        [math.pi - 0.01, -math.pi + 0.01])
    assert math.isclose(abs(found), math.pi, abs_tol=1e-9)


def test_classify():
    velocity = np.array([[0.3, 0, 0], [0, 0, 0], [0, 0, 0]], dtype=float)
    yawspeed = np.array([0.0, 0.5, 0.0])
    assert list(fa.classify(velocity, yawspeed)) == ['moving', 'turning', 'holding']


def test_components_moving_and_holding():
    # Flying north; 2 cm ahead, 3 cm to the east, 1 cm low. Then holding, 3-4-5 off target
    error = np.array([[0.02, 0.03, 0.01], [0.03, 0.04, -0.02]])
    velocity = np.array([[0.3, 0.0, 0.0], [0.0, 0.0, 0.0]])
    kind = fa.classify(velocity, np.zeros(2))
    parts = fa.components(error, velocity, kind)
    assert np.allclose(parts['altitude'], [-0.01, 0.02])
    assert np.allclose(parts['along'][1], [0.02])
    assert np.allclose(parts['off_line'][1], [math.hypot(0.03, 0.01)])
    assert np.allclose(parts['horizontal'][1], [0.05])


def test_settling():
    t = np.arange(0.0, 5.0, 0.02)
    target, previous = np.array([1.0, 0.0, -1.0]), np.array([0.0, 0.0, -1.0])
    # Overshoots 8 cm north, then decays: inside 5 cm from about t = 0.47 s
    north = 1.0 + 0.08 * np.exp(-t)
    truth = np.column_stack([north, np.zeros_like(t), -np.ones_like(t)])
    h = fa.settling(t, truth, np.zeros_like(t), target, 0.0, previous, 0.0, 0.05)
    assert math.isclose(h['overshoot'], 0.08, rel_tol=1e-6)
    assert math.isclose(h['settle_time'], math.log(0.08 / 0.05), abs_tol=0.03)
    assert h['overshoot_deg'] is None


def test_settling_never_settles_and_turns():
    t = np.arange(0.0, 2.0, 0.02)
    truth = np.tile([0.1, 0.0, -1.0], (len(t), 1))  # 10 cm off the whole time
    yaw = np.full_like(t, math.radians(93.0))      # turned 3 deg past a +90 turn
    h = fa.settling(t, truth, yaw, np.array([0.0, 0.0, -1.0]), math.radians(90.0),
                    np.array([0.0, 0.0, -1.0]), 0.0, 0.05)
    assert h['settle_time'] is None
    assert math.isclose(h['overshoot_deg'], 3.0, abs_tol=1e-9)
    assert math.isclose(h['final_error'], 0.1)
    assert math.isclose(h['wobble'], 0.0, abs_tol=1e-12)


def test_describe():
    assert fa.describe(np.array([1.0, 0.0, 0.0]), 0.0) == 'north 1.0 m'
    assert fa.describe(np.array([0.0, -1.0, -0.5]), 0.0) == 'west 1.0 m, up 0.5 m'
    assert fa.describe(np.zeros(3), math.radians(-90.0)) == 'turn -90'


def test_held_and_runs_of():
    assert list(fa.held(np.array([0.0, 1.0]), np.array([5, 7]), np.array([-1.0, 0.5, 1.0, 9.0]))) \
        == [5, 5, 7, 7]
    assert fa.runs_of(np.array(['a', 'a', 'b', 'a'])) == [(0, 2), (2, 3), (3, 4)]


def synthetic_flight(truth_east_offset):
    """Rest, climb to 1 m, hold, 1 m north, hold. PX4's estimate follows the setpoint exactly;
    mocap (z up, y = west) sees the drone truth_east_offset m east of it once in the air."""
    t = np.arange(0.0, 30.0, 0.02)
    climb, move = (t >= 6) & (t < 9), (t >= 14) & (t < 17)
    velocity = np.zeros((len(t), 3))
    velocity[climb, 2] = -1 / 3
    velocity[move, 0] = 1 / 3
    position = np.cumsum(velocity, axis=0) * 0.02
    setpoint = dict(t=t, position=position, velocity=velocity, yaw=np.zeros(len(t)),
                    yawspeed=np.zeros(len(t)))
    estimate = dict(t=t, position=position.copy(), yaw=np.zeros(len(t)))
    status = dict(t=t, offboard=t >= 5, armed=t >= 5)
    north, east, down = position.T
    east = east + truth_east_offset * (t >= 6)
    truth = dict(t=t, position=np.column_stack([north, -east, -down]), yaw=np.zeros(len(t)))
    return fa.evaluate(setpoint, estimate, status, truth, 0.05)


def test_compare_before_after(tmp_path):
    before, after = synthetic_flight(0.10), synthetic_flight(0.0)
    assert after['holds'][1]['after'] == 'north 1.0 m'
    table = fa.compare([('Before', [before]), ('After', [after])])
    assert table.splitlines()[0] == '| | Before | After |'
    row = next(line for line in table.splitlines() if 'while holding, mean / max' in line
               and 'estimate' in line)
    assert row.endswith('| 10.0 / 10.0 | 0.0 / 0.0 |')
    settled = next(line for line in table.splitlines() if 'settled' in line)
    assert settled.endswith('| 0 of 1 | 1 of 1 |')
    fa.plot_before_after([('Before', [before]), ('After', [after])], tmp_path / 'b.png')
    assert (tmp_path / 'b.png').stat().st_size > 0
