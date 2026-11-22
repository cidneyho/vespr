"""external_bridge's maths: an outside pose (mocap or VIO) in ROS terms, to what PX4 wants.

No ROS here, so it can be tested on its own. Design: docs/design/external_bridge.md.

A pose is (position, orientation): position (x, y, z) in m, orientation a unit quaternion
(x, y, z, w), as in ROS. Frames are ROS's: x forward, y left, z up.
"""
import math
import random

NAN3 = (math.nan, math.nan, math.nan)

# ---- Which clock a stamp is on ----------------------------------------------------------------
#
# PX4's estimator lines each sample up with its IMU by the sample's time, and that time must be on
# PX4's own clock. PX4 uses the stamp as it arrives; it never converts it. Three clocks are in play:
#
#   system clock    time of day: seconds since 1970 (about 1.76e9 s in 2026). The real drone's
#                   mocap PC and Jetson stamp with it (the two kept together by chrony).
#   PX4's clock     on the real drone, a stopwatch: seconds since the Pixhawk powered on.
#                   In the sim (SITL), Gazebo's clock: SITL sets its clock from Gazebo's.
#   Gazebo's clock  seconds since the sim started. mocap_sim and fake_vio stamp with it.
#
# So in the sim the stamps are already on PX4's clock and pass through unchanged. On the real drone
# they are on the system clock, and PX4's measured gap (/fmu/out/timesync_status) is added.
#
# Nothing has to be set to say which: the two are told apart by size. A system-clock stamp is over
# 1e9 s (that was September 2001); a Gazebo or PX4 time would need 32 years of uptime to reach it.
# docs/design/external_bridge.md, "Timestamps", has the full story.
SYSTEM_CLOCK_MIN_NS = 1_000_000_000 * 1_000_000_000  # 1e9 s


def on_system_clock(stamp_ns):
    """True if a stamp is on the system clock (real drone), False if on PX4's clock (sim)."""
    return stamp_ns >= SYSTEM_CLOCK_MIN_NS


def px4_time_us(stamp_ns, offset_us):
    """A stamp on PX4's clock, in µs.

    offset_us is TimesyncStatus.estimated_offset: PX4's clock minus the system clock (PX4 v1.16,
    src/lib/timesync/Timesync.cpp). Used only for system-clock stamps; may be None for the others.
    """
    stamp_us = stamp_ns // 1000
    if not on_system_clock(stamp_ns):
        return stamp_us
    if offset_us is None:
        raise ValueError('system-clock stamp, but no time offset from PX4 yet')
    return stamp_us + offset_us


# ---- Quaternions, (x, y, z, w) ----

def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def normalise(q):
    n = math.sqrt(sum(c * c for c in q))
    return tuple(c / n for c in q)


def rotate(q, v):
    """Rotate vector v by quaternion q."""
    x, y, z, _ = quat_multiply(quat_multiply(q, (*v, 0.0)), (-q[0], -q[1], -q[2], q[3]))
    return (x, y, z)


def yaw(q):
    """Heading of quaternion q about z, rad."""
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def from_yaw(angle):
    """Quaternion for a turn of angle rad about z."""
    return (0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2))


# ---- Zero at start ----

class Zeroer:
    """Poses from where the drone was at the start: level, x along its heading then.

    The same frame as fake_vio's odom (and real VIO's), so PX4's local frame is the same whether
    it flies on mocap or VIO.

    The start is the average of the samples in the first window_ns of input (by their stamps), not
    one sample: a single sample's noise would stay in PX4's frame for the whole flight, as a fixed
    shift and turn (the 2026-11-21 noise sweep: up to 14 cm and 5° from one sample with 5 cm and
    0.05 rad of noise). Averaging n samples cuts that by √n, whatever rate the input arrives at.
    The drone should sit still meanwhile; if it moves, the start is where it was on average.

    restart() (after a sim reset) collects a new start; the new frame raises reset_counter, which
    tells PX4 the jump is deliberate.
    """

    def __init__(self, window_ns=0, rng=random):
        self.window_ns = window_ns
        # PX4 spots a new frame by reset_counter changing. A restarted bridge starts from a random
        # value, so it most likely differs from the last one PX4 saw (1 in 256 that it doesn't).
        self.reset_counter = rng.randrange(256)
        self.start = None     # (position, heading) of the start, once collected
        self.samples = []     # (stamp ns, position, heading) collected toward the start
        self.spread = None    # RMS distance of the samples from their average, m
        self.had_start = False

    @property
    def zeroed(self):
        return self.start is not None

    def add(self, pose, stamp_ns):
        """Collect a sample toward the start. True once zeroed: then apply() can be used."""
        if self.zeroed:
            return True
        position, orientation = pose
        self.samples.append((stamp_ns, position, yaw(orientation)))
        if stamp_ns - self.samples[0][0] < self.window_ns:
            return False
        n = len(self.samples)
        mean = tuple(sum(s[1][i] for s in self.samples) / n for i in range(3))
        # Headings averaged as directions, so 179° and -179° average to 180°, not 0°
        heading = math.atan2(sum(math.sin(s[2]) for s in self.samples),
                             sum(math.cos(s[2]) for s in self.samples))
        self.spread = math.sqrt(sum(sum((p - m) ** 2 for p, m in zip(s[1], mean))
                                    for s in self.samples) / n)
        if self.had_start:
            self.reset_counter = (self.reset_counter + 1) % 256
        self.start, self.had_start = (mean, heading), True
        self.samples = []
        return True

    def restart(self):
        """Drop the start (and any samples collected): add() collects a new one."""
        self.start, self.samples = None, []

    def apply(self, pose):
        """The pose in the start frame. Only once zeroed."""
        if not self.zeroed:
            raise ValueError('not zeroed yet: add() samples first')
        (start_position, start_yaw), (position, orientation) = self.start, pose
        unturn = from_yaw(-start_yaw)
        offset = tuple(p - s for p, s in zip(position, start_position))
        return rotate(unturn, offset), normalise(quat_multiply(unturn, orientation))

    def turn_variances(self, variances):
        """Variances along x, y, z of the input frame, seen along the start frame's axes.

        Only the diagonal is kept, so x and y mix by the start heading (exact when they're equal).
        """
        if not self.zeroed or any(math.isnan(v) for v in variances):
            return variances
        c, s = math.cos(self.start[1]), math.sin(self.start[1])
        vx, vy, vz = variances
        return (c * c * vx + s * s * vy, s * s * vx + c * c * vy, vz)


# ---- ROS (x forward, y left, z up) to PX4 (x forward, y right, z down) ----
#
# Both frames are "fixed, any heading" (PX4's FRD), so the change is a 180° turn about x:
# y and z change sign. For a quaternion that is (x, y, z, w) -> PX4's [w, x, -y, -z].

def flip_vector(v):
    return (v[0], -v[1], -v[2])


def flip_quaternion(q):
    """ROS (x, y, z, w), body FLU to world FLU -> PX4's [w, x, y, z], body FRD to world FRD."""
    x, y, z, w = normalise(q)
    return (w, x, -y, -z)


def covariance_diagonal(matrix):
    """((x, y, z), (rx, ry, rz)) variances from a 6 x 6 row-major ROS covariance.

    NaN where ROS says unknown: all zeros, or a -1 first element (REP 147).
    """
    diagonal = [matrix[i * 7] for i in range(6)]
    if matrix[0] < 0 or all(v == 0 for v in diagonal):
        return NAN3, NAN3
    return tuple(diagonal[:3]), tuple(diagonal[3:])


def odometry_fields(position, orientation, velocity=None, angular_velocity=None,
                    position_variance=NAN3, orientation_variance=NAN3, velocity_variance=NAN3):
    """VehicleOdometry's pose, velocity and variance fields, from a pose already in the start frame.

    velocity and angular_velocity are in the body frame (ROS base_link), or None if not measured.
    Returns a dict of field name to value; frames are left to the caller (FRD, BODY_FRD).
    """
    return {
        'position': flip_vector(position),
        'q': flip_quaternion(orientation),
        'velocity': NAN3 if velocity is None else flip_vector(velocity),
        'angular_velocity': NAN3 if angular_velocity is None else flip_vector(angular_velocity),
        # Variances are squares, so the sign flip doesn't change them
        'position_variance': tuple(position_variance),
        'orientation_variance': tuple(orientation_variance),
        'velocity_variance': tuple(velocity_variance),
    }
