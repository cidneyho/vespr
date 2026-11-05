"""Base for the perch_sim nodes that turn Gazebo's ground truth into a sensor's output.

Handles what they share: reading Gazebo over gz-transport, the branch's perch_point, the rate
limit, dropped frames, noise and latency. A subclass declares its own parameters, then calls
start(), and implements sample(msg) to turn one Gazebo sample into the messages to publish.
"""
from collections import deque
import random
import threading
import time

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from rclpy.node import Node

from perch_sim.gazebo import RateLimit, link_in_world, stamp_ns
from perch_sim.poses import add_noise


class SimSensor(Node):

    def __init__(self, name, rate, **kwargs):
        super().__init__(name, **kwargs)
        self.world = self.declare_parameter('world', 'perch_rod').value
        self.drone_model = self.declare_parameter('drone_model', 'x500_hook_0').value
        self.drone_link = self.declare_parameter('drone_link', 'base_link').value
        # The perch worlds name the branch model after the world
        self.branch_model = self.declare_parameter('branch_model', '').value or self.world
        self.rate = self.declare_parameter('rate', rate).value
        self.position_noise = self.declare_parameter('position_noise', 0.0).value
        self.orientation_noise = self.declare_parameter('orientation_noise', 0.0).value
        self.latency = self.declare_parameter('latency', 0.0).value
        self.drop_rate = self.declare_parameter('drop_rate', 0.0).value
        # 0 means a new seed each run; set one to repeat a run's noise exactly
        seed = self.declare_parameter('seed', 0).value

        self.rng = random.Random(seed or None)
        self.lock = threading.Lock()  # gz-transport calls back on its own threads
        self.branch = None  # perch_point in the world, read once from the static scene
        self.limit = RateLimit(self.rate)
        self.pending = deque()  # (due, messages) waiting out the latency
        self.warned = set()
        if self.latency > 0:
            self.create_timer(0.002, self.flush)

    def start(self):
        """Subscribe to Gazebo. Call once the subclass is set up: callbacks can come at once."""
        self.scene_topic = f'/world/{self.world}/pose/info'
        self.unsubscribe_timer = self.create_timer(0.5, self.unsubscribe_scene)
        self.gz = GzNode()
        self.gz.subscribe(Pose_V, self.scene_topic, self.on_scene)
        self.gz.subscribe(Pose_V, f'/world/{self.world}/dynamic_pose/info', self.on_dynamic)

    def sample(self, msg):
        """[(publisher, message)] for one Gazebo sample. Called with the lock held."""
        raise NotImplementedError

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
        with self.lock:
            if not self.limit.due(stamp_ns(msg)):
                return
            if self.rng.random() < self.drop_rate:
                return  # a lost frame: nothing is reported
            messages = self.sample(msg)
            if not messages:
                return
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

    # ---- for sample(), called with the lock held ----

    def drone_pose(self, msg, link=None):
        """World pose of a link on the drone (default its body), or None with a warning."""
        pose, error = link_in_world(msg, self.drone_model, link or self.drone_link)
        if error:
            self.warn_once(f'Drone: {error}')
        return pose

    def pose_msg(self, pose, msg, frame):
        """PoseStamped with noise, stamped with the Gazebo sample's sim time."""
        (x, y, z), (qx, qy, qz, qw) = add_noise(
            pose, self.position_noise, self.orientation_noise, self.rng)
        out = PoseStamped()
        out.header.frame_id = frame
        out.header.stamp = Time(sec=msg.header.stamp.sec, nanosec=msg.header.stamp.nsec)
        out.pose.position.x, out.pose.position.y, out.pose.position.z = x, y, z
        o = out.pose.orientation
        o.x, o.y, o.z, o.w = qx, qy, qz, qw
        return out

    def warn_once(self, text):
        if text not in self.warned:
            self.warned.add(text)
            self.get_logger().warn(text)


def publish(messages):
    for publisher, msg in messages:
        publisher.publish(msg)
