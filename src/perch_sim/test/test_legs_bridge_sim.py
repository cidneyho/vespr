"""legs_bridge_sim against a fake gate that stands in for Gazebo. No sim needed."""
import math
import random
import threading
import time

from action_msgs.msg import GoalStatus
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import SetBool

from perch_interfaces.action import Grasp
from perch_sim.legs_bridge_sim import GATE_JOINT, LegsBridgeSim

STEP = 0.02  # s


class FakeGate(Node):
    """Moves the gate angle towards the commanded target at a fixed speed, like Gazebo's controller."""

    def __init__(self, context, speed=math.pi):
        super().__init__('fake_gate', context=context)
        self.speed = speed
        self.angle = 0.0
        self.target = 0.0
        self.pub = self.create_publisher(JointState, '/hook/joint_state', 10)
        self.create_subscription(Float64, '/hook/gate', self.on_target, 10)
        self.create_timer(STEP, self.step)

    def on_target(self, msg):
        self.target = msg.data

    def step(self):
        limit = self.speed * STEP
        self.angle += max(-limit, min(limit, self.target - self.angle))
        self.pub.publish(JointState(name=[GATE_JOINT], position=[self.angle]))


class Rig:
    def __init__(self, gate_speed=math.pi, timeout=3.0):
        # A random domain keeps the test off any running ROS system
        self.context = rclpy.Context()
        rclpy.init(context=self.context, domain_id=random.randint(100, 200))
        self.gate = FakeGate(self.context, gate_speed)
        self.bridge = LegsBridgeSim(
            context=self.context, parameter_overrides=[Parameter('timeout', value=timeout)])
        self.client = Node('test_client', context=self.context)
        self.grasp = ActionClient(self.client, Grasp, '/vespr/legs/grasp')
        self.release = ActionClient(self.client, Grasp, '/vespr/legs/release')
        self.set_enabled = self.client.create_client(SetBool, '/vespr/legs/set_enabled')

        self.executor = MultiThreadedExecutor(context=self.context)
        for node in (self.gate, self.bridge, self.client):
            self.executor.add_node(node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

        assert self.grasp.wait_for_server(timeout_sec=5)
        assert self.release.wait_for_server(timeout_sec=5)
        assert self.set_enabled.wait_for_service(timeout_sec=5)
        wait_until(lambda: self.bridge.angle is not None)

    def enable(self, on=True):
        return wait(self.set_enabled.call_async(SetBool.Request(data=on)))

    def send(self, client):
        goal_handle = wait(client.send_goal_async(Grasp.Goal()))
        return goal_handle

    def close(self):
        self.executor.shutdown()
        rclpy.shutdown(context=self.context)
        self.thread.join(timeout=5)


def wait(future, timeout=5.0):
    wait_until(future.done, timeout)
    return future.result()


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


def test_rejects_goals_while_disabled(rig):
    assert not rig.send(rig.grasp).accepted


def test_grasp_then_release(rig):
    rig.enable()

    response = wait(rig.send(rig.grasp).get_result_async())
    assert response.status == GoalStatus.STATUS_SUCCEEDED
    assert response.result.success and response.result.gripped
    assert rig.gate.angle == pytest.approx(math.pi, abs=0.05)

    response = wait(rig.send(rig.release).get_result_async())
    assert response.status == GoalStatus.STATUS_SUCCEEDED
    assert response.result.success and not response.result.gripped
    assert rig.gate.angle == pytest.approx(0.0, abs=0.05)


def test_publishes_state(rig):
    states = []
    rig.client.create_subscription(JointState, '/vespr/legs/state', states.append, 10)
    wait_until(lambda: states)
    assert states[-1].name == ['latch']
    assert states[-1].position[0] == pytest.approx(0.0, abs=0.01)


def test_cancel_stops_the_gate_mid_swing(rig):
    rig.enable()
    goal_handle = rig.send(rig.grasp)
    time.sleep(0.4)
    wait(goal_handle.cancel_goal_async())

    response = wait(goal_handle.get_result_async())
    assert response.status == GoalStatus.STATUS_CANCELED
    stopped_at = rig.gate.angle
    time.sleep(0.5)
    assert 0.3 < rig.gate.angle < 2.5
    assert rig.gate.angle == pytest.approx(stopped_at, abs=0.1)


def test_disable_aborts_and_stops(rig):
    rig.enable()
    goal_handle = rig.send(rig.grasp)
    time.sleep(0.4)
    rig.enable(False)

    response = wait(goal_handle.get_result_async())
    assert response.status == GoalStatus.STATUS_ABORTED
    assert response.result.message == 'legs disabled'
    time.sleep(0.5)
    assert rig.gate.angle < 2.5


def test_new_goal_replaces_the_running_one(rig):
    rig.enable()
    first = rig.send(rig.grasp)
    time.sleep(0.3)
    second = rig.send(rig.release)

    response = wait(first.get_result_async())
    assert response.status == GoalStatus.STATUS_ABORTED
    assert response.result.message == 'replaced by a new goal'
    assert wait(second.get_result_async()).status == GoalStatus.STATUS_SUCCEEDED
    assert rig.gate.angle == pytest.approx(0.0, abs=0.05)


def test_times_out_when_the_gate_is_stuck():
    rig = Rig(gate_speed=0.0, timeout=0.5)
    try:
        rig.enable()
        response = wait(rig.send(rig.grasp).get_result_async())
        assert response.status == GoalStatus.STATUS_ABORTED
        assert response.result.message.startswith('timed out')
    finally:
        rig.close()
