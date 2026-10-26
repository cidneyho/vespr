"""The mission node: runs the perch cycle when an operator sends /vespr/perch. See docs/design/mission.md.

One process, two nodes: `mission` (this) and the flight Commander, which it drives directly.
    ros2 launch vespr_flight mission.launch.py
    ros2 action send_goal /vespr/perch vespr_interfaces/action/Perch \\
      "{north: 0.3, east: 0.8, altitude: 1.3, heading: 60.0, perch_time: 10.0}" --feedback
The state is also published on /vespr/mission/state.
"""
import threading
import time

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import SetBool

from perch_flight.commander import Commander, PX4_QOS
from px4_msgs.msg import VehicleAttitude
from vespr_flight.perch_cycle import Failed, PerchCycle, Settings
from vespr_flight.planning.planner import make
from vespr_flight.planning.steps import level
from vespr_interfaces.action import Grasp, Perch

PHASES = ('fly_to', 'search', 'approach')
DEFAULT_PLANNERS = {'fly_to': 'straight', 'search': 'turn', 'approach': 'standoff'}
LEGS_TIMEOUT = 10.0  # s to wait for the legs bridge to answer
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Legs:
    """The legs bridge's interface, called from the mission's thread and waited on."""

    def __init__(self, node, group):
        self.node = node
        self.enable_client = node.create_client(
            SetBool, '/vespr/legs/set_enabled', callback_group=group)
        self.grasp_client = ActionClient(node, Grasp, '/vespr/legs/grasp', callback_group=group)
        self.release_client = ActionClient(node, Grasp, '/vespr/legs/release', callback_group=group)

    def enable(self, on):
        if not self.enable_client.wait_for_service(timeout_sec=LEGS_TIMEOUT):
            raise LegsError('legs bridge not running')
        response = wait(self.enable_client.call_async(SetBool.Request(data=on)))
        if response is None or not response.success:
            raise LegsError(f'could not {"enable" if on else "disable"} the legs')

    def grasp(self):
        result = self.run(self.grasp_client)
        return result.success, result.gripped, result.message

    def release(self):
        result = self.run(self.release_client)
        return result.success, result.message

    def run(self, client):
        if not client.wait_for_server(timeout_sec=LEGS_TIMEOUT):
            raise LegsError('legs bridge not running')
        handle = wait(client.send_goal_async(Grasp.Goal()))
        if handle is None or not handle.accepted:
            raise LegsError('legs bridge refused the goal')
        response = wait(handle.get_result_async(), timeout=LEGS_TIMEOUT)
        if response is None:
            raise LegsError('legs bridge did not answer')
        return response.result


class LegsError(Failed):
    """The legs bridge didn't do what was asked. Ends the state like any other failure."""


def wait(future, timeout=LEGS_TIMEOUT):
    """Result of a future completed by the executor's other threads, or None after timeout."""
    end = time.monotonic() + timeout
    while not future.done():
        if time.monotonic() > end:
            return None
        time.sleep(0.01)
    return future.result()


class Mission(Node):

    def __init__(self, commander, **kwargs):
        # Planner settings are given as e.g. approach.step_length, so accept any parameter
        super().__init__('mission', allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True, **kwargs)
        self.commander = commander
        self.settings = Settings(**{
            name: float(self.param(name, default)) for name, default in vars(Settings()).items()})
        self.group = ReentrantCallbackGroup()
        self.legs = Legs(self, self.group)
        self.state_pub = self.create_publisher(String, '/vespr/mission/state', LATCHED)
        self.create_subscription(PoseStamped, '/vespr/perception/branch_pose', self.on_branch,
                                 qos_profile_sensor_data, callback_group=self.group)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.on_attitude,
                                 PX4_QOS, callback_group=self.group)
        self.attitude = None
        self.branch = (None, None)
        self.running = threading.Lock()
        self.stopping = False  # Ctrl+C: end the run as if cancelled
        ActionServer(self, Perch, '/vespr/perch', self.execute, goal_callback=self.on_goal,
                     cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=self.group)
        self.publish_state('ground')
        self.get_logger().info('Ready for /vespr/perch goals')

    def param(self, name, default):
        return self.get_parameter_or(name, rclpy.Parameter(name, value=default)).value

    def planners(self, goal):
        """Fresh planners for one run, chosen and set up from the parameters."""
        out = {}
        for phase in PHASES:
            settings = {name: p.value for name, p in self.get_parameters_by_prefix(phase).items()}
            if phase == 'fly_to':
                settings.update(target_north=goal.north, target_east=goal.east,
                                target_altitude=goal.altitude, target_heading=goal.heading)
            out[phase] = make(phase, self.param(f'{phase}_planner', DEFAULT_PLANNERS[phase]),
                              **settings)
        return out

    def on_attitude(self, msg):
        self.attitude = tuple(msg.q)

    def on_branch(self, msg):
        # Timed by arrival on Commander's clock: the detector stamps sim time, Commander uses wall time
        if self.attitude is None:
            return  # can't level it yet
        p, o = msg.pose.position, msg.pose.orientation
        pose = ((p.x, p.y, p.z), (o.x, o.y, o.z, o.w))
        self.branch = (level(pose, self.attitude), self.commander.now())

    def on_goal(self, goal):
        if self.running.locked():
            self.get_logger().warn('Refused a goal: a perch is already running')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def publish_state(self, state):
        self.state_pub.publish(String(data=state))

    def execute(self, handle):
        with self.running:
            goal = handle.request
            self.get_logger().info(
                f'Perch: to north={goal.north:.2f} east={goal.east:.2f} altitude={goal.altitude:.2f}, '
                f'facing {goal.heading:.0f}, perch {goal.perch_time:.0f} s')

            def report(state):
                self.get_logger().info(f'State: {state}')
                self.publish_state(state)
                handle.publish_feedback(Perch.Feedback(state=state))

            try:
                cycle = PerchCycle(self.commander, self.legs, self.planners(goal),
                                   sighting=lambda: self.branch, report=report,
                                   cancelled=lambda: handle.is_cancel_requested or self.stopping,
                                   settings=self.settings, log=self.get_logger().info)
                success, state, message = cycle.run(goal)
            except (ValueError, TypeError) as e:  # bad planner name or setting
                success, state, message = False, 'ground', f'could not start: {e}'
            (self.get_logger().info if success else self.get_logger().error)(
                f'Perch {"done" if success else "ended"} in {state}: {message}')
            self.publish_state('ground')
            if handle.is_cancel_requested:
                handle.canceled()
            elif success:
                handle.succeed()
            else:
                handle.abort()
            return Perch.Result(success=success, state=state, message=message)


def main():
    # Handle Ctrl+C ourselves, as flight_cli does: end a running perch like a cancel (its abort
    # move lands the drone), wait for that, then quit. A second Ctrl+C quits at once
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    commander = Commander()
    mission = Mission(commander)
    executor = MultiThreadedExecutor()
    executor.add_node(commander)
    executor.add_node(mission)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        while True:
            try:
                time.sleep(0.2)
                if mission.stopping and not mission.running.locked() and commander.safe_to_exit():
                    break
            except KeyboardInterrupt:
                if mission.stopping:
                    break
                mission.get_logger().warn('Ctrl+C: ending the perch and landing, then quitting. '
                                          'Ctrl+C again to quit right away')
                mission.stopping = True
    finally:
        executor.shutdown()
        mission.destroy_node()
        commander.destroy_node()
        rclpy.try_shutdown()
