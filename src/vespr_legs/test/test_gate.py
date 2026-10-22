import math

from vespr_legs.gate import CLOSED, OPEN, latch_position, reached


def test_latch_position():
    assert latch_position(OPEN) == 0.0
    assert latch_position(CLOSED) == 1.0
    assert latch_position(math.pi / 2) == 0.5


def test_latch_position_clamps_overshoot():
    # The joint limits are -0.1 .. 3.3 rad, a little past open and closed
    assert latch_position(-0.1) == 0.0
    assert latch_position(3.3) == 1.0


def test_reached():
    assert reached(3.10, CLOSED, 0.05)
    assert not reached(3.0, CLOSED, 0.05)
    assert reached(-0.02, OPEN, 0.05)
