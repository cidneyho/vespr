"""external_bridge: the conversion maths, then the node fed fake mocap, VIO and PX4 messages."""
import math
import random
import threading
import time

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import TimesyncStatus, VehicleOdometry
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from perch_flight.commander import PX4_QOS
from perch_flight.external_bridge import ExternalBridge
from perch_flight.external_convert import (SYSTEM_CLOCK_MIN_NS, Zeroer, covariance_diagonal,
                                           flip_quaternion, from_yaw, normalise, odometry_fields,
                                           on_system_clock, px4_time_us, quat_multiply, rotate)

LEVEL = (0.0, 0.0, 0.0, 1.0)
S = 1_000_000_000  # ns
SYSTEM_2026 = 1_763_000_000 * S  # a time of day in November 2026, ns


def isnan3(v):
    return all(math.isnan(c) for c in v)


# ---- Clocks ----

def test_gazebo_stamps_pass_through():
    stamp = 125 * S + 400_000  # 125.0004 s of sim time
    assert not on_system_clock(stamp)
    assert px4_time_us(stamp, None) == 125_000_400
    assert px4_time_us(stamp, 999) == 125_000_400  # the offset is never used for them


def test_system_stamps_get_the_offset():
    assert on_system_clock(SYSTEM_2026)
    # PX4 booted 400 s ago: its clock is the system clock minus (now - 400 s)
    offset = -(SYSTEM_2026 // 1000 - 400_000_000)
    assert px4_time_us(SYSTEM_2026, offset) == 400_000_000


def test_system_stamp_without_offset_is_refused():
    with pytest.raises(ValueError):
        px4_time_us(SYSTEM_2026, None)


def test_the_line_between_clocks():
    assert not on_system_clock(SYSTEM_CLOCK_MIN_NS - 1)
    assert on_system_clock(SYSTEM_CLOCK_MIN_NS)
    assert not on_system_clock(30 * 24 * 3600 * S)  # a month of uptime is still PX4's clock


# ---- Zero at start ----

def test_zero_at_the_first_pose_level_along_its_heading():
    zeroer = Zeroer(random.Random(1))
    tilt = (0.0, math.sin(0.05), 0.0, math.cos(0.05))  # 0.1 rad nose down
    facing_north = from_yaw(math.pi / 2)
    start = ((3.0, 2.0, 0.24), normalise(quat_multiply(facing_north, tilt)))
    position, q = zeroer.apply(start)
    assert position == pytest.approx((0, 0, 0))
    assert q == pytest.approx(tilt)  # heading taken out, tilt kept: the frame is level
    # 1 m north of the start (+y in the room) is 1 m along the start heading
    position, _ = zeroer.apply(((3.0, 3.0, 0.24), facing_north))
    assert position == pytest.approx((1, 0, 0))
    # Turning 90° left from the start heading reads as yaw +90° in the start frame
    _, q = zeroer.apply(((3.0, 3.0, 0.24), from_yaw(math.pi)))
    assert q == pytest.approx(from_yaw(math.pi / 2))


def test_zero_again_raises_the_reset_counter():
    zeroer = Zeroer(random.Random(1))
    zeroer.apply(((1.0, 0.0, 0.0), LEVEL))
    first = zeroer.reset_counter
    zeroer.zero(((5.0, 5.0, 0.0), LEVEL))
    assert zeroer.reset_counter == (first + 1) % 256
    assert zeroer.apply(((5.0, 5.0, 0.0), LEVEL))[0] == pytest.approx((0, 0, 0))


def test_restarted_bridges_start_from_different_counters():
    counters = {Zeroer(random.Random(seed)).reset_counter for seed in range(20)}
    assert len(counters) > 10


def test_variances_turn_with_the_start_heading():
    zeroer = Zeroer(random.Random(1))
    zeroer.apply(((0.0, 0.0, 0.0), from_yaw(math.pi / 2)))  # started facing +y
    # Uncertain along the room's y = along the start heading = the start frame's x
    assert zeroer.turn_variances((0.0, 4.0, 1.0)) == pytest.approx((4.0, 0.0, 1.0))
    assert isnan3(zeroer.turn_variances((math.nan,) * 3))


# ---- ROS to PX4 axes ----

def test_flip_to_px4_axes():
    # Facing 90° left of x in ROS (yaw +90°, counterclockwise from above)...
    fields = odometry_fields((1.0, 2.0, 3.0), from_yaw(math.pi / 2))
    assert fields['position'] == (1.0, -2.0, -3.0)
    w, x, y, z = fields['q']
    # ...is yaw -90° in PX4, whose z points down (clockwise positive)
    assert math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) == pytest.approx(-math.pi / 2)


def test_flipped_quaternion_turns_frd_vectors_like_ros_turns_flu_ones():
    q = normalise((0.1, -0.2, 0.3, 0.9))
    body_left = (0.0, 1.0, 0.0)  # FLU; it is (0, -1, 0) in FRD
    world_flu = rotate(q, body_left)
    w, x, y, z = flip_quaternion(q)
    world_frd = rotate((x, y, z, w), (0.0, -1.0, 0.0))
    assert world_frd == pytest.approx((world_flu[0], -world_flu[1], -world_flu[2]))


def test_quaternion_is_normalised():
    w, x, y, z = flip_quaternion((0.0, 0.0, 0.0, 1.00002))  # a little long, as noise can make it
    assert math.sqrt(w * w + x * x + y * y + z * z) == pytest.approx(1.0, abs=1e-12)


def test_mocap_has_no_velocity_or_variances():
    fields = odometry_fields((0.0, 0.0, 0.0), LEVEL)
    for name in ('velocity', 'angular_velocity', 'position_variance', 'orientation_variance',
                 'velocity_variance'):
        assert isnan3(fields[name]), name


def test_vio_velocity_flips_in_the_body_frame():
    fields = odometry_fields((0.0, 0.0, 0.0), LEVEL, velocity=(0.5, 0.2, 0.1),
                             angular_velocity=(0.0, 0.0, 0.3))
    assert fields['velocity'] == (0.5, -0.2, -0.1)  # forward, 0.2 left = -0.2 right, 0.1 up
    assert fields['angular_velocity'] == (0.0, 0.0, -0.3)


def test_covariance_diagonal():
    matrix = [0.0] * 36
    for i, value in enumerate((1, 2, 3, 4, 5, 6)):
        matrix[i * 7] = float(value)
    assert covariance_diagonal(matrix) == ((1, 2, 3), (4, 5, 6))
    unknown = covariance_diagonal([0.0] * 36)
    assert isnan3(unknown[0]) and isnan3(unknown[1])
    assert isnan3(covariance_diagonal([-1.0] + [0.0] * 35)[0])


# ---- The node ----

class Rig:
    def __init__(self, input_type='pose'):
        self.context = rclpy.Context()
        rclpy.init(context=self.context, domain_id=random.randint(100, 200))
        self.bridge = ExternalBridge(
            context=self.context,
            parameter_overrides=[Parameter('input_type', value=input_type)])
        self.fake = Node('fake', context=self.context)
        self.out = []
        self.fake.create_subscription(
            VehicleOdometry, '/fmu/in/vehicle_visual_odometry', self.out.append, PX4_QOS)
        self.timesync = self.fake.create_publisher(
            TimesyncStatus, '/fmu/out/timesync_status', PX4_QOS)
        self.input = self.fake.create_publisher(
            PoseStamped if input_type == 'pose' else Odometry, 'input', 10)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.bridge)
        self.executor.add_node(self.fake)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        wait_until(lambda: (self.input.get_subscription_count() > 0
                            and self.bridge.pub.get_subscription_count() > 0))

    def send(self, msg):
        n = len(self.out)
        self.input.publish(msg)
        wait_until(lambda: len(self.out) > n)
        return self.out[-1]

    def close(self):
        self.executor.shutdown()
        rclpy.shutdown(context=self.context)
        self.thread.join(timeout=5)


def wait_until(condition, timeout=5.0):
    end = time.monotonic() + timeout
    while not condition():
        assert time.monotonic() < end, 'timed out waiting'
        time.sleep(0.01)


def stamp(ns):
    return Time(sec=ns // S, nanosec=ns % S)


def pose_msg(ns, position, q=LEVEL):
    msg = PoseStamped()
    msg.header.stamp = stamp(ns)
    msg.header.frame_id = 'mocap'
    p, o = msg.pose.position, msg.pose.orientation
    p.x, p.y, p.z = position
    o.x, o.y, o.z, o.w = q
    return msg


@pytest.fixture
def rig():
    rig = Rig()
    yield rig
    rig.close()


def test_mocap_from_the_start_in_px4_axes(rig):
    facing_north = from_yaw(math.pi / 2)
    first = rig.send(pose_msg(10 * S, (3.0, 2.0, 0.24), facing_north))
    assert list(first.position) == pytest.approx([0, 0, 0])
    assert first.pose_frame == VehicleOdometry.POSE_FRAME_FRD
    assert first.timestamp_sample == 10_000_000  # Gazebo time, already PX4's: unchanged
    assert first.quality == 100
    assert isnan3(first.velocity) and isnan3(first.position_variance)
    # 1 m north (the start heading) and 0.5 m up: forward 1, down -0.5
    msg = rig.send(pose_msg(11 * S, (3.0, 3.0, 0.74), facing_north))
    assert list(msg.position) == pytest.approx([1.0, 0.0, -0.5])
    assert msg.reset_counter == first.reset_counter


def test_out_of_order_sample_is_dropped(rig):
    rig.send(pose_msg(10 * S, (0.0, 0.0, 0.0)))
    rig.send(pose_msg(10 * S + 20_000_000, (0.1, 0.0, 0.0)))
    rig.input.publish(pose_msg(10 * S + 10_000_000, (0.05, 0.0, 0.0)))  # 10 ms late
    time.sleep(0.2)
    assert len(rig.out) == 2


def test_sim_reset_zeroes_again(rig):
    rig.send(pose_msg(100 * S, (0.0, 0.0, 0.0)))
    before = rig.send(pose_msg(101 * S, (2.0, 0.0, 0.0)))
    after = rig.send(pose_msg(1 * S, (5.0, 5.0, 0.0)))  # sim time back near 0
    assert list(after.position) == pytest.approx([0, 0, 0])
    assert after.reset_counter == (before.reset_counter + 1) % 256


def test_system_clock_waits_for_px4s_offset(rig):
    rig.input.publish(pose_msg(SYSTEM_2026, (0.0, 0.0, 0.0)))
    time.sleep(0.2)
    assert rig.out == []  # no offset yet: nothing sent
    sync = TimesyncStatus()
    sync.source_protocol = TimesyncStatus.SOURCE_PROTOCOL_DDS
    sync.estimated_offset = -(SYSTEM_2026 // 1000 - 400_000_000)  # PX4 booted 400 s ago
    rig.timesync.publish(sync)
    wait_until(lambda: rig.bridge.offset_us is not None)
    msg = rig.send(pose_msg(SYSTEM_2026 + S, (0.0, 0.0, 0.0)))
    assert msg.timestamp_sample == 401_000_000


def test_mavlink_timesync_is_ignored(rig):
    sync = TimesyncStatus()
    sync.source_protocol = TimesyncStatus.SOURCE_PROTOCOL_MAVLINK
    sync.estimated_offset = 123
    rig.timesync.publish(sync)
    time.sleep(0.2)
    assert rig.bridge.offset_us is None


def test_vio_velocity_and_covariance():
    rig = Rig('odom')
    try:
        msg = Odometry()
        msg.header.stamp = stamp(5 * S)
        msg.child_frame_id = 'base_link'
        msg.pose.pose.orientation.w = 1.0
        msg.twist.twist.linear.x, msg.twist.twist.linear.y = 0.5, 0.2
        msg.pose.covariance[0] = msg.pose.covariance[7] = msg.pose.covariance[14] = 1e-4
        msg.pose.covariance[21] = msg.pose.covariance[28] = msg.pose.covariance[35] = 1e-3
        msg.twist.covariance[0] = msg.twist.covariance[7] = msg.twist.covariance[14] = 1e-2
        out = rig.send(msg)
        assert out.velocity_frame == VehicleOdometry.VELOCITY_FRAME_BODY_FRD
        assert list(out.velocity) == pytest.approx([0.5, -0.2, 0.0])
        assert list(out.position_variance) == pytest.approx([1e-4] * 3)
        assert list(out.orientation_variance) == pytest.approx([1e-3] * 3)
        assert list(out.velocity_variance) == pytest.approx([1e-2] * 3)
    finally:
        rig.close()
