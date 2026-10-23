"""fake_vio: the Odometer maths, then the node fed fake Gazebo pose messages. No sim needed."""
import math
import os
import random
import threading
import time
import uuid

# Keep gz-transport traffic to this test. Must be set before any gz node exists
os.environ['GZ_PARTITION'] = f'fake_vio_test_{uuid.uuid4().hex[:8]}'

from gz.msgs10.pose_v_pb2 import Pose_V  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
import pytest  # noqa: E402
import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

from vespr_sim.fake_vio import FakeVio, Odometer  # noqa: E402
from vespr_sim.poses import from_yaw, quat_multiply  # noqa: E402

LEVEL = (0.0, 0.0, 0.0, 1.0)
S = 1_000_000_000  # ns


def pitch(angle):
    return (0.0, math.sin(angle / 2), 0.0, math.cos(angle / 2))


# ---- Odometer ----

def test_origin_is_the_start_pose_level_along_its_heading():
    odo = Odometer(0.0, 0.0, random.Random(1))
    # Start 2 m east, facing north (yaw 90°), tilted nose down a little
    start = ((0.0, 2.0, 0.5), quat_multiply(from_yaw(math.pi / 2), pitch(0.1)))
    (position, q), linear, angular = odo.update(start, 10 * S)
    assert position == pytest.approx((0, 0, 0))
    assert q == pytest.approx(pitch(0.1))  # the tilt stays: odom is level, the drone isn't
    assert linear == angular == (0.0, 0.0, 0.0)
    # 1 m north of the start is 1 m along odom's x
    (position, _), _, _ = odo.update(((0.0, 3.0, 0.5), from_yaw(math.pi / 2)), 11 * S)
    assert position == pytest.approx((1, 0, 0))


def test_velocity_is_in_the_body_frame():
    odo = Odometer(0.0, 0.0, random.Random(1))
    facing_north = from_yaw(math.pi / 2)
    odo.update(((0.0, 0.0, 1.0), facing_north), 0)
    # 5 cm north and 0.01 rad of left turn in 0.1 s: 0.5 m/s forward, 0.1 rad/s turning left
    _, linear, angular = odo.update(((0.0, 0.05, 1.0), from_yaw(math.pi / 2 + 0.01)), S // 10)
    assert linear == pytest.approx((0.5, 0, 0), abs=0.01)  # it has turned 0.01 rad by now
    assert angular == pytest.approx((0, 0, 0.1))


def test_position_drift_grows_with_distance_flown():
    odo = Odometer(0.01, 0.0, random.Random(1))
    odo.update(((0.0, 0.0, 1.0), LEVEL), 0)
    for i in range(1, 11):  # 10 m out along x
        (position, _), _, _ = odo.update(((float(i), 0.0, 1.0), LEVEL), i * S)
    error = [a - b for a, b in zip(position, (10, 0, 0))]
    assert math.dist(error, (0, 0, 0)) == pytest.approx(0.1)  # 1 % of 10 m
    for i in range(9, -1, -1):  # and back: drift keeps growing, it doesn't undo
        (position, _), _, _ = odo.update(((float(i), 0.0, 1.0), LEVEL), (20 - i) * S)
    assert math.dist(position, (0, 0, 0)) == pytest.approx(0.2)


def test_yaw_drift_turns_the_frame():
    rate = math.radians(6) / 60  # 6 degrees a minute
    odo = Odometer(0.0, rate, random.Random(1))
    odo.update(((0.0, 0.0, 1.0), LEVEL), 0)
    (position, q), _, _ = odo.update(((10.0, 0.0, 1.0), LEVEL), 60 * S)
    turned = odo.yaw_sign * math.radians(6)
    assert q == pytest.approx(from_yaw(turned))
    # 10 m out, a 6° turn moves the reported position about 1 m sideways
    assert position == pytest.approx((10 * math.cos(turned), 10 * math.sin(turned), 0))


def test_sim_reset_zeroes_again():
    odo = Odometer(0.0, 0.0, random.Random(1))
    odo.update(((0.0, 0.0, 0.2), LEVEL), 100 * S)
    odo.update(((5.0, 0.0, 1.0), LEVEL), 101 * S)
    (position, _), linear, _ = odo.update(((7.0, 1.0, 0.2), LEVEL), 0)  # sim time back to 0
    assert position == pytest.approx((0, 0, 0))
    assert linear == (0.0, 0.0, 0.0)


# ---- The node ----

WORLD = 'test_world'


def dynamic(stamp_s, x=1.0, q=LEVEL):
    msg = Pose_V()
    msg.header.stamp.sec = int(stamp_s)
    msg.header.stamp.nsec = round((stamp_s - int(stamp_s)) * 1e9)
    entities = (('x500_hook_0', (x, 2, 0), q), ('base_link', (0, 0, 0.24), LEVEL))
    for name, (px, py, pz), (qx, qy, qz, qw) in entities:
        p = msg.pose.add()
        p.name = name
        p.position.x, p.position.y, p.position.z = px, py, pz
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
    return msg


class Rig:
    def __init__(self, **params):
        self.context = rclpy.Context()
        rclpy.init(context=self.context, domain_id=random.randint(100, 200))
        self.vio = FakeVio(
            context=self.context,
            parameter_overrides=[Parameter('world', value=WORLD)]
            + [Parameter(k, value=v) for k, v in params.items()])
        self.listener = Node('listener', context=self.context)
        self.odom = []
        self.listener.create_subscription(Odometry, '/vespr/perception/odom', self.odom.append, 100)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.vio)
        self.executor.add_node(self.listener)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        wait_until(lambda: self.vio.pub.get_subscription_count() > 0)

    def close(self):
        self.executor.shutdown()
        rclpy.shutdown(context=self.context)
        self.thread.join(timeout=5)


def wait_until(condition, timeout=5.0):
    end = time.monotonic() + timeout
    while not condition():
        assert time.monotonic() < end, 'timed out waiting'
        time.sleep(0.01)


@pytest.fixture
def rig():
    rig = Rig()
    yield rig
    rig.close()


def test_publishes_odometry(rig):
    rig.vio.on_dynamic(dynamic(10.0, x=1.0))
    rig.vio.on_dynamic(dynamic(10.5, x=1.5))
    wait_until(lambda: len(rig.odom) == 2)
    msg = rig.odom[1]
    assert (msg.header.frame_id, msg.child_frame_id) == ('odom', 'base_link')
    assert (msg.header.stamp.sec, msg.header.stamp.nanosec) == (10, 500_000_000)
    p, v = msg.pose.pose.position, msg.twist.twist.linear
    assert (p.x, p.y, p.z) == pytest.approx((0.5, 0, 0))
    assert (v.x, v.y, v.z) == pytest.approx((1.0, 0, 0))
    # Perfect output: tiny but non-zero covariance on the diagonal, zero elsewhere
    assert msg.pose.covariance[0] == msg.twist.covariance[35] == pytest.approx(1e-6)
    assert msg.pose.covariance[1] == 0.0


def test_never_faster_than_rate(rig):
    # Gazebo at 50 Hz for 1 s, rate 30 Hz
    for i in range(50):
        rig.vio.on_dynamic(dynamic(i * 0.02, x=float(i)))
    time.sleep(0.2)
    assert 24 <= len(rig.odom) <= 31


def test_noise_sets_the_covariance():
    rig = Rig(position_noise=0.01, linear_velocity_noise=0.1, seed=3)
    try:
        rig.vio.on_dynamic(dynamic(1.0))
        wait_until(lambda: rig.odom)
        msg = rig.odom[0]
        assert msg.pose.covariance[0] == pytest.approx(1e-4)
        assert msg.twist.covariance[0] == pytest.approx(1e-2)
        assert msg.pose.pose.position.x != 0.0
    finally:
        rig.close()


def test_seed_repeats_a_run():
    runs = []
    for _ in range(2):
        rig = Rig(position_noise=0.01, position_drift=0.05, seed=42)
        try:
            for i in range(5):
                rig.vio.on_dynamic(dynamic(i * 0.1, x=float(i)))
            wait_until(lambda: len(rig.odom) == 5)
            runs.append([m.pose.pose.position.x for m in rig.odom])
        finally:
            rig.close()
    assert runs[0] == runs[1]
