"""What a planner sees (Situation) and what it returns (steps). See docs/design/planning.md.

A pose is (position, orientation): position (x, y, z) in m, orientation a unit quaternion
(x, y, z, w). The branch is in base_link: x forward, y left, z up.
"""
from dataclasses import dataclass
import math

YAW_RATE = 30.0  # deg/s, how fast Commander turns (perch_flight.commander.YAW_RATE)


@dataclass(frozen=True)
class Situation:
    time: float                     # s, the same clock as Commander's (sim time in the sim)
    # The drone in Commander's frame, from PX4's position estimate. Only phase 1 uses them
    position: tuple | None = None   # (north, east, altitude), m from the takeoff spot
    heading: float | None = None    # deg, 0 = north, 90 = east
    branch: tuple | None = None     # newest /vespr/perception/branch_pose, in base_link
    branch_time: float | None = None  # s, when that sighting was taken


# ---- Steps: each is one Commander call, or the legs ----

@dataclass(frozen=True)
class Goto:
    north: float
    east: float
    altitude: float
    speed: float


@dataclass(frozen=True)
class MoveBody:
    forward: float
    right: float
    up: float
    speed: float


@dataclass(frozen=True)
class YawBy:
    degrees: float  # positive is clockwise seen from above, as Commander's yaw_by


@dataclass(frozen=True)
class Wait:
    pass


@dataclass(frozen=True)
class Grasp:
    pass


@dataclass(frozen=True)
class Done:
    pass


@dataclass(frozen=True)
class Abort:
    reason: str


def duration(step, distance=None):
    """Roughly how long Commander takes to carry out a move, s. Goto needs the distance flown."""
    if isinstance(step, MoveBody):
        return math.dist((0, 0, 0), (step.forward, step.right, step.up)) / step.speed
    if isinstance(step, YawBy):
        return abs(step.degrees) / YAW_RATE
    if isinstance(step, Goto):
        return (distance or 0.0) / step.speed
    return 0.0


def rotate(q, v):
    """Rotate vector v by unit quaternion q."""
    qx, qy, qz, qw = q
    # v + 2 w (q x v) + 2 q x (q x v)
    cx, cy, cz = qy * v[2] - qz * v[1], qz * v[0] - qx * v[2], qx * v[1] - qy * v[0]
    return (v[0] + 2 * (qw * cx + qy * cz - qz * cy),
            v[1] + 2 * (qw * cy + qz * cx - qx * cz),
            v[2] + 2 * (qw * cz + qx * cy - qy * cx))


class Pacer:
    """Tracks the planner's own last move, so it acts only on sightings taken after it ended.

    The mission can't tell when a move has finished, only that Commander accepts the next one.
    So the planner works out when its move should end, from the move's length and speed.
    """

    def __init__(self, settle_time):
        self.settle_time = settle_time
        self.ready_at = -math.inf  # s, sightings from this time on show the drone at rest

    def moved(self, step, now, distance=None):
        """Record a move issued at `now`, and return it."""
        self.ready_at = now + duration(step, distance) + self.settle_time
        return step

    def busy(self, now):
        return now < self.ready_at

    def fresh(self, situation):
        """The sighting was taken after the last move ended and the drone settled."""
        return (situation.branch is not None and situation.branch_time is not None
                and situation.branch_time >= self.ready_at)
