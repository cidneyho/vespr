"""Stand-in for the mocap system in Gazebo: drone and branch poses from Gazebo's ground truth.

Publishes on the same topics, type and frame as the real mocap node (docs/design/mocap_sim.md):
    /mocap/vespr/pose    the drone's base_link
    /mocap/branch/pose   the branch model's perch_point link
Both are geometry_msgs/PoseStamped in frame "mocap" (Gazebo's world frame), stamped with the sim
time of the Gazebo sample. Perfect by default; noise, latency and dropped frames are parameters.

Reads Gazebo directly through gz-transport rather than ros_gz_bridge, because the bridge drops
the entity names from pose messages. Start the sim, then:
    ros2 launch vespr_sim mocap_sim.launch.py world:=perch_rod
"""
from collections import deque
import random
import threading
import time

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from vespr_sim.poses import add_noise, compose

FRAME = 'mocap'


def find_pose(msg, name):
    """(pose, error) for the one entity called name in a gz Pose_V. Links are relative to their model."""
    found = [p for p in msg.pose if p.name == name]
    if len(found) != 1:
        return None, f'{len(found)} entities called {name!r}' if found else f'no entity called {name!r}'
    p = found[0]
    position = (p.position.x, p.position.y, p.position.z)
    orientation = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
    return (position, orientation), None


def link_in_world(msg, model, link):
    """(pose, error) of a model's link in the world: the model's world pose combined with the link's."""
    model_pose, error = find_pose(msg, model)
    if error:
        return None, error
    link_pose, error = find_pose(msg, link)
    if error:
        return None, error
    return compose(model_pose, link_pose), None


class MocapSim(Node):

    def __init__(self, **kwargs):
        super().__init__('mocap_sim', **kwargs)
        world = self.declare_parameter('world', 'perch_rod').value
        self.drone_model = self.declare_parameter('drone_model', 'x500_hook_0').value
        self.drone_link = self.declare_parameter('drone_link', 'base_link').value
        # The perch worlds name the branch model after the world
        self.branch_model = self.declare_parameter('branch_model', '').value or world
        self.rate = self.declare_parameter('rate', 120.0).value
        self.position_noise = self.declare_parameter('position_noise', 0.0).value
        self.orientation_noise = self.declare_parameter('orientation_noise', 0.0).value
        self.latency = self.declare_parameter('latency', 0.0).value
        self.drop_rate = self.declare_parameter('drop_rate', 0.0).value

        self.rng = random.Random()
        self.lock = threading.Lock()  # gz-transport calls back on its own threads
        self.branch = None  # perch_point in the world, read once from the static scene
        self.last_stamp = None  # ns, newest Gazebo sample seen
        self.next_due = None  # ns, earliest sample time the rate limit lets through
        self.pending = deque()  # (due, messages) waiting out the latency
        self.warned = set()

        self.drone_pub = self.create_publisher(PoseStamped, '/mocap/vespr/pose', 10)
        self.branch_pub = self.create_publisher(PoseStamped, '/mocap/branch/pose', 10)
        if self.latency > 0:
            self.create_timer(0.002, self.flush)

        self.get_logger().info(
            f'World {world}: drone {self.drone_model}/{self.drone_link}, '
            f'branch {self.branch_model}/perch_point. Waiting for Gazebo')
        self.scene_topic = f'/world/{world}/pose/info'
        self.unsubscribe_timer = self.create_timer(0.5, self.unsubscribe_scene)
        self.gz = GzNode()
        self.gz.subscribe(Pose_V, self.scene_topic, self.on_scene)
        self.gz.subscribe(Pose_V, f'/world/{world}/dynamic_pose/info', self.on_dynamic)

    def on_scene(self, msg):
        with self.lock:
            if self.branch is not None:
                return
            pose, error = link_in_world(msg, self.branch_model, 'perch_point')
            if error:
                self.warn_once(f'Branch: {error} in {self.scene_topic}')
                return
            self.branch = pose
        (x, y, z), _ = pose
        self.get_logger().info(f'Branch perch_point at ({x:.3f}, {y:.3f}, {z:.3f})')

    def unsubscribe_scene(self):
        """The branch is static, so stop receiving the whole scene once we have it."""
        with self.lock:
            done = self.branch is not None
        if done:
            self.gz.unsubscribe(self.scene_topic)
            self.unsubscribe_timer.cancel()

    def on_dynamic(self, msg):
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nsec
        with self.lock:
            if not self.due(stamp):
                return
            if self.rng.random() < self.drop_rate:
                return  # a lost mocap frame: no body is reported
            drone, error = link_in_world(msg, self.drone_model, self.drone_link)
            if error:
                self.warn_once(f'Drone: {error}')
                return
            messages = [(self.drone_pub, self.pose_msg(drone, msg.header.stamp))]
            if self.branch is not None:
                messages.append((self.branch_pub, self.pose_msg(self.branch, msg.header.stamp)))
            if self.latency > 0:
                self.pending.append((time.monotonic() + self.latency, messages))
                return
        publish(messages)

    def flush(self):
        now = time.monotonic()
        ready = []
        with self.lock:
            while self.pending and self.pending[0][0] <= now:
                ready.append(self.pending.popleft()[1])
        for messages in ready:
            publish(messages)

    # ---- called with the lock held ----

    def due(self, stamp):
        """Rate limit: on average at most `rate` samples a second, and never a sample twice."""
        period = 1e9 / self.rate
        slack = 0.05 * period  # so a source at exactly `rate` isn't thinned by timing jitter
        if self.last_stamp is not None and stamp < self.last_stamp:
            self.next_due = None  # sim time went backwards: the sim was reset
        self.last_stamp = stamp
        if self.next_due is not None and stamp < self.next_due - slack:
            return False
        # Keep to the schedule, but after a gap restart it rather than catch up in a burst
        if self.next_due is None or stamp - self.next_due > period:
            self.next_due = stamp
        self.next_due += period
        return True

    def pose_msg(self, pose, stamp):
        (x, y, z), (qx, qy, qz, qw) = add_noise(
            pose, self.position_noise, self.orientation_noise, self.rng)
        msg = PoseStamped()
        msg.header.frame_id = FRAME
        msg.header.stamp = Time(sec=stamp.sec, nanosec=stamp.nsec)
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
        o = msg.pose.orientation
        o.x, o.y, o.z, o.w = qx, qy, qz, qw
        return msg

    def warn_once(self, text):
        if text not in self.warned:
            self.warned.add(text)
            self.get_logger().warn(text)


def publish(messages):
    for publisher, msg in messages:
        publisher.publish(msg)


def main():
    rclpy.init()
    node = MocapSim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
