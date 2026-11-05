"""legs_bridge for Gazebo: the legs interface, driving the hook gate on the x500_hook model.

Serves the same interface as the real legs_bridge will (docs/design/legs.md), under the same
node name, so the mission can't tell sim from real. Start the sim with the hook model, then:
    ros2 launch perch_sim legs_bridge_sim.launch.py

The gate is commanded on /hook/gate (target angle) and measured on /hook/joint_state, both
bridged from Gazebo by the launch file.
"""
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import SetBool

from perch_interfaces.action import Grasp
from perch_sim.gate import CLOSED, OPEN, latch_position, reached

GATE_JOINT = 'hook_gate_joint'
RATE = 10.0  # Hz, for the state topic and goal feedback


class Job:
    """The goal currently moving the gate."""

    def __init__(self, goal_handle, target):
        self.goal_handle = goal_handle
        self.target = target
        self.reason = None  # set when something else ends it: a new goal or a disable


class LegsBridgeSim(Node):

    def __init__(self, **kwargs):
        super().__init__('legs_bridge', **kwargs)
        self.timeout = self.declare_parameter('timeout', 3.0).value
        self.tolerance = self.declare_parameter('tolerance', 0.05).value

        self.lock = threading.Lock()  # guards everything below
        self.enabled = False
        self.angle = None  # measured gate angle, None until Gazebo reports it
        self.job = None

        group = ReentrantCallbackGroup()
        self.gate_pub = self.create_publisher(Float64, '/hook/gate', 10)
        self.state_pub = self.create_publisher(JointState, '/vespr/legs/state', 10)
        self.create_subscription(
            JointState, '/hook/joint_state', self.on_gate_state, 10, callback_group=group)
        self.create_service(
            SetBool, '/vespr/legs/set_enabled', self.on_set_enabled, callback_group=group)
        self.action_servers = [
            ActionServer(
                self, Grasp, f'/vespr/legs/{name}',
                execute_callback=lambda goal_handle, target=target: self.execute(goal_handle, target),
                goal_callback=self.on_goal,
                cancel_callback=lambda _: CancelResponse.ACCEPT,
                callback_group=group)
            for name, target in (('grasp', CLOSED), ('release', OPEN))
        ]
        self.create_timer(1.0 / RATE, self.publish_state, callback_group=group)
        self.get_logger().info('Legs disabled. Waiting for the gate state from Gazebo')

    def on_gate_state(self, msg):
        if GATE_JOINT not in msg.name:
            return
        with self.lock:
            first = self.angle is None
            self.angle = msg.position[msg.name.index(GATE_JOINT)]
        if first:
            self.get_logger().info('Gate state received')

    def on_set_enabled(self, request, response):
        with self.lock:
            self.enabled = request.data
            if not self.enabled and self.job is not None:
                self.job.reason = 'legs disabled'
                self.job = None
                self.stop_gate()
        response.success = True
        response.message = 'enabled' if request.data else 'disabled'
        self.get_logger().info(f'Legs {response.message}')
        return response

    def on_goal(self, _goal_request):
        with self.lock:
            if not self.enabled:
                self.get_logger().warn('Goal rejected: legs disabled')
                return GoalResponse.REJECT
            if self.angle is None:
                self.get_logger().warn('Goal rejected: no gate state from Gazebo yet')
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute(self, goal_handle, target):
        start = self.get_clock().now()
        with self.lock:
            # Disabled between accepting the goal and starting it
            if not self.enabled:
                goal_handle.abort()
                return result(False, False, 'legs disabled')
            if self.job is not None:
                self.job.reason = 'replaced by a new goal'
            job = self.job = Job(goal_handle, target)
            self.gate_pub.publish(Float64(data=target))

        feedback = Grasp.Feedback(moving=True)
        while True:
            with self.lock:
                if job.reason is not None:
                    goal_handle.abort()
                    return result(False, False, job.reason)
                if goal_handle.is_cancel_requested:
                    self.end(job)
                    goal_handle.canceled()
                    return result(False, False, 'cancelled')
                if reached(self.angle, target, self.tolerance):
                    self.job = None
                    goal_handle.succeed()
                    gripped = target == CLOSED
                    return result(True, gripped, 'gripped' if gripped else 'open')
                if (self.get_clock().now() - start).nanoseconds / 1e9 > self.timeout:
                    self.end(job)
                    goal_handle.abort()
                    return result(False, False, f'timed out after {self.timeout:.1f} s')
                feedback.latch_position = latch_position(self.angle)
            goal_handle.publish_feedback(feedback)
            time.sleep(1.0 / RATE)

    # ---- called with the lock held ----

    def end(self, job):
        """Stop the gate for a job that ends early, unless a newer job already owns it."""
        if self.job is job:
            self.job = None
            self.stop_gate()

    def stop_gate(self):
        """Hold the gate where it is now."""
        self.gate_pub.publish(Float64(data=self.angle))

    # ----

    def publish_state(self):
        with self.lock:
            if self.angle is None:
                return
            position = latch_position(self.angle)
        msg = JointState(name=['latch'], position=[position])
        msg.header.stamp = self.get_clock().now().to_msg()
        self.state_pub.publish(msg)


def result(success, gripped, message):
    return Grasp.Result(success=success, gripped=gripped, message=message)


def main():
    rclpy.init()
    node = LegsBridgeSim()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
