"""Pose maths for the perch_sim nodes, without numpy.

A pose is (position, orientation): position (x, y, z) in m, orientation a unit quaternion (x, y, z, w).
"""
import math


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


def compose(parent, child):
    """Pose of child in the parent's frame, expressed in the frame the parent is given in."""
    (p, q), (cp, cq) = parent, child
    offset = rotate(q, cp)
    return (
        (p[0] + offset[0], p[1] + offset[1], p[2] + offset[2]),
        normalise(quat_multiply(q, cq)),
    )


def inverse(pose):
    """The pose that undoes `pose`: compose(inverse(a), b) is b seen from a."""
    (x, y, z), (qx, qy, qz, qw) = pose
    q = (-qx, -qy, -qz, qw)
    p = rotate(q, (x, y, z))
    return (-p[0], -p[1], -p[2]), q


def from_rotation_vector(r):
    """Quaternion for a rotation of |r| rad about the axis r."""
    angle = math.sqrt(sum(c * c for c in r))
    if angle < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    s = math.sin(angle / 2) / angle
    return (r[0] * s, r[1] * s, r[2] * s, math.cos(angle / 2))


def to_rotation_vector(q):
    """Axis times angle (rad) of quaternion q, taking the short way round."""
    x, y, z, w = q
    if w < 0:
        x, y, z, w = -x, -y, -z, -w
    s = math.sqrt(x * x + y * y + z * z)
    if s < 1e-12:
        return (0.0, 0.0, 0.0)
    angle = 2 * math.atan2(s, w)
    return (x / s * angle, y / s * angle, z / s * angle)


def yaw(q):
    """Heading of quaternion q about z, rad."""
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def from_yaw(angle):
    """Quaternion for a turn of angle rad about z."""
    return (0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2))


def add_noise(pose, position_sigma, orientation_sigma, rng):
    """Gaussian noise: position_sigma m on each axis, orientation_sigma rad about each body axis."""
    position, orientation = pose
    if position_sigma > 0:
        position = tuple(c + rng.gauss(0.0, position_sigma) for c in position)
    if orientation_sigma > 0:
        twist = from_rotation_vector([rng.gauss(0.0, orientation_sigma) for _ in range(3)])
        orientation = normalise(quat_multiply(orientation, twist))
    return position, orientation
