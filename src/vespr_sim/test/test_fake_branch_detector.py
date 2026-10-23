"""fake_branch_detector: the view check, the left-pointing rule, and the node fed fake Gazebo poses."""
import math
import os
import random
import threading
import time
import uuid

os.environ.setdefault('GZ_PARTITION', f'detector_test_{uuid.uuid4().hex[:8]}')

from geometry_msgs.msg import PoseStamped  # noqa: E402
from gz.msgs10.pose_v_pb2 import Pose_V  # noqa: E402
import pytest  # noqa: E402
import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

from vespr_sim.fake_branch_detector import FakeBranchDetector, in_view, point_left  # noqa: E402
from vespr_sim.poses import compose, inverse, rotate  # noqa: E402

FOV_H, FOV_V = math.radians(69.0), math.radians(42.0)
CAMERA = (0.12, 0.03, 0.242)


def yaw(angle):
    return (0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2))


# ---- pure functions ----

@pytest.mark.parametrize('point, seen', [
    ((1.0, 0.0, 0.0), True),
    ((0.2, 0.0, 0.0), False),     # too near
    ((3.5, 0.0, 0.0), False),     # too far
    ((-1.0, 0.0, 0.0), False),    # behind
    ((1.0, 0.65, 0.0), True),     # 33 deg left, inside 34.5
    ((1.0, 0.72, 0.0), False),    # 36 deg left
    ((1.0, -0.72, 0.0), False),   # 36 deg right
    ((1.0, 0.0, 0.36), True),     # 20 deg up, inside 21
    ((1.0, 0.0, 0.42), False),    # 23 deg up
    ((1.0, 0.0, -0.42), False),   # 23 deg down
])
def test_in_view(point, seen):
    assert in_view(point, FOV_H, FOV_V, 0.3, 3.0) == seen


def test_point_left_keeps_a_left_pointing_branch():
    q = yaw(math.pi / 2)  # x points left
    assert point_left(q) == q


def test_point_left_turns_a_right_pointing_branch():
    x_axis = rotate(point_left(yaw(-math.pi / 2)), (1, 0, 0))
    assert x_axis == pytest.approx((0, 1, 0), abs=1e-9)
    z_axis = rotate(point_left(yaw(-math.pi / 2)), (0, 0, 1))
    assert z_axis == pytest.approx((0, 0, 1), abs=1e-9)  # still up


def test_point_left_fore_and_aft_points_forward():
    assert rotate(point_left(yaw(math.pi)), (1, 0, 0)) == pytest.approx((1, 0, 0), abs=1e-9)


def test_inverse():
    pose = ((1.0, 2.0, 3.0), yaw(0.7))
    position, orientation = compose(inverse(pose), pose)
    assert position == pytest.approx((0, 0, 0), abs=1e-12)
    assert orientation == pytest.approx((0, 0, 0, 1), abs=1e-12)


# ---- the node ----

def pose_v(*poses, stamp_s=1.0):
    """gz Pose_V from (name, (x, y, z), quaternion) triples."""
    msg = Pose_V()
    msg.header.stamp.sec = int(stamp_s)
    msg.header.stamp.nsec = round((stamp_s - int(stamp_s)) * 1e9)
    for name, (x, y, z), (qx, qy, qz, qw) in poses:
        p = msg.pose.add()
        p.name = name
        p.position.x, p.position.y, p.position.z = x, y, z
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
    return msg


# Branch like perch_rod: model at (3, 0, 0), perch_point 1.5 m up with x along world +y
SCENE = pose_v(('test_world', (3, 0, 0), yaw(0)), ('perch_point', (0, 0, 1.5), yaw(math.pi / 2)))


def drone(position, heading, stamp_s=1.0):
    """Drone model at position, facing heading (rad), with base_link at the model origin."""
    return pose_v(('x500_hook_0', position, yaw(heading)), ('base_link', (0, 0, 0), yaw(0)),
                  ('camera_link', CAMERA, yaw(0)), stamp_s=stamp_s)


class Rig:
    def __init__(self):
        self.context = rclpy.Context()
        rclpy.init(context=self.context, domain_id=random.randint(100, 200))
        self.detector = FakeBranchDetector(
            context=self.context, parameter_overrides=[Parameter('world', value='test_world')])
        self.listener = Node('listener', context=self.context)
        self.seen = []
        self.listener.create_subscription(
            PoseStamped, '/vespr/perception/branch_pose', self.seen.append, 100)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.detector)
        self.executor.add_node(self.listener)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        end = time.monotonic() + 5
        while self.detector.pub.get_subscription_count() == 0:
            assert time.monotonic() < end
            time.sleep(0.01)
        self.detector.on_scene(SCENE)

    def see(self, msg):
        """Feed one sample; the published message, or None if the branch wasn't reported."""
        before = len(self.seen)
        self.detector.on_dynamic(msg)
        end = time.monotonic() + 0.3
        while len(self.seen) == before and time.monotonic() < end:
            time.sleep(0.01)
        return self.seen[-1] if len(self.seen) > before else None

    def close(self):
        self.executor.shutdown()
        rclpy.shutdown(context=self.context)
        self.thread.join(timeout=5)


@pytest.fixture
def rig():
    rig = Rig()
    yield rig
    rig.close()


def position(msg):
    p = msg.pose.position
    return (p.x, p.y, p.z)


def x_axis(msg):
    o = msg.pose.orientation
    return rotate((o.x, o.y, o.z, o.w), (1, 0, 0))


def test_reports_the_branch_from_the_drone(rig):
    msg = rig.see(drone((1, 0, 1.5), 0.0))
    assert msg.header.frame_id == 'base_link'
    assert (msg.header.stamp.sec, msg.header.stamp.nanosec) == (1, 0)
    assert position(msg) == pytest.approx((2, 0, 0))
    assert x_axis(msg) == pytest.approx((0, 1, 0), abs=1e-6)  # along the rod, to the left


def test_same_direction_from_the_other_side(rig):
    # Facing the rod from beyond it: the world's x would point right, so it is turned to the left
    msg = rig.see(drone((5, 0, 1.5), math.pi))
    assert position(msg) == pytest.approx((2, 0, 0))
    assert x_axis(msg) == pytest.approx((0, 1, 0), abs=1e-6)


@pytest.mark.parametrize('where, heading', [
    ((1, 0, 1.5), math.pi / 2),   # facing away to the side
    ((-1, 0, 1.5), 0.0),          # 4 m away, beyond max_range
    ((2.75, 0, 1.5), 0.0),        # 0.13 m in front of the camera, nearer than min_range
    ((2.6, 0, 0.9), 0.0),         # under the branch: 0.24 m ahead of the camera, 0.36 m up
])
def test_silent_when_out_of_view(rig, where, heading):
    assert rig.see(drone(where, heading)) is None


def test_silent_until_the_branch_is_known(rig):
    rig.detector.branch = None
    assert rig.see(drone((1, 0, 1.5), 0.0)) is None


def test_rate_limited_to_30_hz(rig):
    for i in range(100):  # 1 s of 100 Hz samples
        rig.detector.on_dynamic(drone((1, 0, 1.5), 0.0, stamp_s=1.0 + i * 0.01))
    time.sleep(0.2)
    assert 25 <= len(rig.seen) <= 31
