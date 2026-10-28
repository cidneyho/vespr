"""The hook gate on the Gazebo x500_hook model: angle 0 is open, pi is closed."""
import math

OPEN = 0.0
CLOSED = math.pi


def latch_position(angle):
    """Gate angle as a latch position: 0 open .. 1 closed."""
    return min(max(angle / CLOSED, 0.0), 1.0)


def reached(angle, target, tolerance):
    return abs(angle - target) <= tolerance
