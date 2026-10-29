"""mocap_sim fed fake Gazebo pose messages. No sim needed."""
import os
import random
import threading
import time
import uuid

# Keep gz-transport traffic to this test. Must be set before any gz node exists
os.environ['GZ_PARTITION'] = f'mocap_sim_test_{uuid.uuid4().hex[:8]}'

from geometry_msgs.msg import PoseStamped  # noqa: E402
from gz.msgs10.pose_v_pb2 import Pose_V  # noqa: E402
from gz.transport13 import Node as GzNode  # noqa: E402
import pytest  # noqa: E402
import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

from perch_sim.mocap_sim import MocapSim  # noqa: E402

WORLD = 'test_world'


def pose_v(stamp_s, *poses):
    """gz Pose_V from (name, (x, y, z)) pairs, identity orientation."""
    msg = Pose_V()
    msg.header.stamp.sec = int(stamp_s)
    msg.header.stamp.nsec = round((stamp_s - int(stamp_s)) * 1e9)
    for name, (x, y, z) in poses:
        p = msg.pose.add()
        p.name = name
        p.position.x, p.position.y, p.position.z = x, y, z
        p.orientation.w = 1.0
    return msg


def scene(stamp_s=0.0):
    return pose_v(stamp_s, ('test_world', (3, 0, 0)), ('perch_point', (0, 0, 1.5)),
                  ('x500_hook_0', (1, 2, 0)), ('base_link', (0, 0, 0.24)))


def dynamic(stamp_s, drone_x=1.0):
    return pose_v(stamp_s, ('x500_hook_0', (drone_x, 2, 0)), ('base_link', (0, 0, 0.24)))


class Rig:
    def __init__(self, **params):
        self.context = rclpy.Context()
        rclpy.init(context=self.context, domain_id=random.randint(100, 200))
        self.mocap = MocapSim(
            context=self.context,
            parameter_overrides=[Parameter('world', value=WORLD)]
            + [Parameter(k, value=v) for k, v in params.items()])
        self.listener = Node('listener', context=self.context)
        self.drone, self.branch = [], []
        self.listener.create_subscription(PoseStamped, '/mocap/vespr/pose', self.drone.append, 100)
        self.listener.create_subscription(PoseStamped, '/mocap/branch/pose', self.branch.append, 100)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.mocap)
        self.executor.add_node(self.listener)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        wait_until(lambda: self.mocap.drone_pub.get_subscription_count() > 0)

    def close(self):
        self.executor.shutdown()
        rclpy.shutdown(context=self.context)
        self.thread.join(timeout=5)


def wait_until(condition, timeout=5.0):
    end = time.monotonic() + timeout
    while not condition():
        assert time.monotonic() < end, 'timed out waiting'
        time.sleep(0.01)


def position(msg):
    p = msg.pose.position
    return (p.x, p.y, p.z)


@pytest.fixture
def rig():
    rig = Rig()
    yield rig
    rig.close()


def test_publishes_drone_and_branch(rig):
    rig.mocap.on_scene(scene())
    rig.mocap.on_dynamic(dynamic(10.5))
    wait_until(lambda: rig.drone and rig.branch)

    drone, branch = rig.drone[0], rig.branch[0]
    assert position(drone) == pytest.approx((1, 2, 0.24))
    assert position(branch) == pytest.approx((3, 0, 1.5))
    for msg in (drone, branch):
        assert msg.header.frame_id == 'mocap'
        assert (msg.header.stamp.sec, msg.header.stamp.nanosec) == (10, 500_000_000)


def test_drone_only_until_the_branch_is_known(rig):
    rig.mocap.on_dynamic(dynamic(1.0))
    wait_until(lambda: rig.drone)
    time.sleep(0.1)
    assert not rig.branch


def test_never_faster_than_rate(rig):
    # 250 Hz Gazebo samples for 1 s, rate 120 Hz. Samples come every 4 ms, so the gaps it can
    # pick are 8 or 12 ms: it lands a little under 120 a second, never over
    rig.mocap.on_scene(scene())
    for i in range(250):
        rig.mocap.on_dynamic(dynamic(i * 0.004, drone_x=i))
    wait_until(lambda: len(rig.drone) >= 95)
    time.sleep(0.1)
    assert 95 <= len(rig.drone) <= 121
    xs = [msg.pose.position.x for msg in rig.drone]
    assert xs == sorted(set(xs))  # no sample sent twice


def test_slower_source_passes_every_sample(rig):
    # Gazebo at about 50 Hz, below the 120 Hz limit: every sample goes out, none repeated
    for i in range(50):
        rig.mocap.on_dynamic(dynamic(i * 0.02, drone_x=i))
    wait_until(lambda: len(rig.drone) >= 50)
    time.sleep(0.1)
    assert [msg.pose.position.x for msg in rig.drone] == list(range(50))


def test_sim_reset_restarts_the_rate_limit(rig):
    rig.mocap.on_dynamic(dynamic(100.0))
    rig.mocap.on_dynamic(dynamic(0.0))  # sim time back to zero
    wait_until(lambda: len(rig.drone) == 2)


def test_ambiguous_names_publish_nothing(rig):
    rig.mocap.on_dynamic(pose_v(1.0, ('x500_hook_0', (0, 0, 0)), ('base_link', (0, 0, 0.24)),
                                ('base_link', (5, 5, 5))))
    time.sleep(0.2)
    assert not rig.drone


def test_drop_rate_one_publishes_nothing():
    rig = Rig(drop_rate=1.0)
    try:
        rig.mocap.on_scene(scene())
        for i in range(20):
            rig.mocap.on_dynamic(dynamic(i * 0.01))
        time.sleep(0.2)
        assert not rig.drone and not rig.branch
    finally:
        rig.close()


def test_latency_delays_publishing():
    rig = Rig(latency=0.3)
    try:
        rig.mocap.on_dynamic(dynamic(1.0))
        time.sleep(0.15)
        assert not rig.drone
        wait_until(lambda: rig.drone, timeout=1.0)
        assert rig.drone[0].header.stamp.sec == 1  # the stamp is still the sample time
    finally:
        rig.close()


def test_reads_gazebo_over_gz_transport(rig):
    gz = GzNode()
    scene_pub = gz.advertise(f'/world/{WORLD}/pose/info', Pose_V)
    dynamic_pub = gz.advertise(f'/world/{WORLD}/dynamic_pose/info', Pose_V)

    def published():
        scene_pub.publish(scene())
        dynamic_pub.publish(dynamic(time.monotonic()))
        return rig.drone and rig.branch

    wait_until(published)
    assert position(rig.branch[-1]) == pytest.approx((3, 0, 1.5))
