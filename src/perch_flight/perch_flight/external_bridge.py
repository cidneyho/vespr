"""external_bridge: an outside pose (mocap or VIO) into PX4's estimator, so PX4 flies on it.

Design: docs/design/external_bridge.md.
    input (remapped)                   PoseStamped (input_type pose: mocap)
                                       or Odometry (input_type odom: VIO, velocity in base_link)
    /fmu/in/vehicle_visual_odometry    px4_msgs/VehicleOdometry, PX4's "external vision" input
    /fmu/out/timesync_status           read for its clock offset, for system-clock stamps

Positions go out from where the drone was at the first sample, level, x along its heading then
(the same frame as VIO's), turned to PX4's axes (x forward, y right, z down).

Stamps: on Gazebo's clock in the sim, which is already PX4's clock there, so they pass through. On
the system clock on the real drone, so PX4's measured offset is added. Detected from the stamp's
size, nothing to set: see SYSTEM_CLOCK_MIN_NS in external_convert.py.

PX4 must be told to use it: EV=mocap or EV=vio with run_sim.sh (perch_sim/config/x500_hook.params).
    ros2 launch perch_flight external_bridge.launch.py source:=mocap
"""
import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import TimesyncStatus, VehicleOdometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from perch_flight.commander import PX4_QOS
from perch_flight.external_convert import (NAN3, Zeroer, covariance_diagonal, odometry_fields,
                                           on_system_clock, px4_time_us)

OUTPUT = '/fmu/in/vehicle_visual_odometry'
TIMESYNC = '/fmu/out/timesync_status'
QUALITY = 100          # 0-100. PX4 drops samples below EKF2_EV_QMIN (0 by default)
SILENCE_WARNING = 0.2  # s without input before warning (wall time)
# Stamps going back further than this mean a sim reset: zero again. A sample only slightly older
# than the last one arrived out of order and is dropped instead
RESET_JUMP_NS = 1_000_000_000


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class ExternalBridge(Node):

    def __init__(self, **kwargs):
        super().__init__('external_bridge', **kwargs)
        self.input_type = self.declare_parameter('input_type', 'pose').value
        if self.input_type not in ('pose', 'odom'):
            raise ValueError(f"input_type must be 'pose' or 'odom', not {self.input_type!r}")
        self.zeroer = Zeroer()
        self.offset_us = None       # PX4's clock minus the system clock, from timesync_status
        self.system_clock = None    # which clock the input stamps are on, once seen
        self.last_stamp = None
        self.last_input = None      # wall time of the last input, for the silence warning
        self.silent = False
        self.sent = 0

        self.pub = self.create_publisher(VehicleOdometry, OUTPUT, PX4_QOS)
        self.create_subscription(TimesyncStatus, TIMESYNC, self.on_timesync, PX4_QOS)
        if self.input_type == 'pose':
            self.create_subscription(PoseStamped, 'input', self.on_pose, qos_profile_sensor_data)
        else:
            self.create_subscription(Odometry, 'input', self.on_odom, qos_profile_sensor_data)
        self.create_timer(0.1, self.check_silence)
        self.get_logger().info(
            f'Waiting for {self.resolve_topic_name("input")} '
            f'({"PoseStamped" if self.input_type == "pose" else "Odometry"})')

    # ---- inputs ----

    def on_timesync(self, msg):
        if msg.source_protocol == TimesyncStatus.SOURCE_PROTOCOL_DDS:  # not a MAVLink link's
            self.offset_us = msg.estimated_offset

    def on_pose(self, msg):
        pose = position_orientation(msg.pose)
        self.send_sample(msg.header.stamp, pose)

    def on_odom(self, msg):
        pose = position_orientation(msg.pose.pose)
        position_variance, orientation_variance = covariance_diagonal(msg.pose.covariance)
        velocity_variance, _ = covariance_diagonal(msg.twist.covariance)
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        self.send_sample(msg.header.stamp, pose,
                         velocity=(v.x, v.y, v.z), angular_velocity=(w.x, w.y, w.z),
                         position_variance=position_variance,
                         orientation_variance=orientation_variance,
                         velocity_variance=velocity_variance)

    # ---- conversion ----

    def send_sample(self, stamp, pose, position_variance=NAN3, orientation_variance=NAN3, **body):
        """Publish one sample. body: velocity, angular_velocity, velocity_variance (VIO only)."""
        self.last_input = self.wall_now()
        ns = stamp_ns(stamp)
        if not self.check_clock(ns):
            return
        if self.last_stamp is not None and ns <= self.last_stamp:
            if self.last_stamp - ns < RESET_JUMP_NS:
                # Arrived out of order (Wi-Fi): PX4 already has a newer sample, so drop this one
                return
            # Stamps went back a long way: the sim was reset (as fake_vio does). Start a new frame
            self.zeroer.zero(pose)
            self.get_logger().warn(
                f'Input stamps went back {(self.last_stamp - ns) * 1e-9:.1f} s: sim reset? '
                f'Zeroed again here (reset_counter {self.zeroer.reset_counter})')
        self.last_stamp = ns
        first = not self.zeroer.zeroed
        position, orientation = self.zeroer.apply(pose)
        if first:
            (x, y, z), heading = self.zeroer.start
            self.get_logger().info(
                f'Zeroed at ({x:.3f}, {y:.3f}, {z:.3f}), heading {math.degrees(heading):.1f} deg: '
                f'PX4 positions are from here, x along this heading')

        fields = odometry_fields(
            position, orientation,
            position_variance=self.zeroer.turn_variances(position_variance),
            orientation_variance=self.zeroer.turn_variances(orientation_variance),
            **body)
        sample_us = px4_time_us(ns, self.offset_us)

        out = VehicleOdometry()
        # PX4 only logs `timestamp`; the estimator uses timestamp_sample
        out.timestamp = out.timestamp_sample = sample_us
        out.pose_frame = VehicleOdometry.POSE_FRAME_FRD
        out.velocity_frame = VehicleOdometry.VELOCITY_FRAME_BODY_FRD
        for name, value in fields.items():
            setattr(out, name, [float(c) for c in value])
        out.reset_counter = self.zeroer.reset_counter
        out.quality = QUALITY
        self.pub.publish(out)
        self.sent += 1

    def check_clock(self, ns):
        """Log which clock the stamps are on; False if this sample can't be timed yet."""
        system = on_system_clock(ns)
        if self.system_clock is None:
            self.system_clock = system
            self.get_logger().info(
                'Input stamps are on the system clock (real drone): adding PX4\'s time offset '
                f'from {TIMESYNC}' if system else
                'Input stamps are on PX4\'s clock already (sim, Gazebo time): passing them through')
        elif system != self.system_clock:
            self.system_clock = system
            self.get_logger().warn(
                f'Input stamps switched to the {"system" if system else "Gazebo/PX4"} clock')
        if system and self.offset_us is None:
            self.get_logger().warn(
                f'No {TIMESYNC} from PX4 yet: not sending, a guessed time is worse than none',
                throttle_duration_sec=2.0)
            return False
        return True

    def check_silence(self):
        if self.last_input is None:
            return
        quiet = self.wall_now() - self.last_input
        if quiet > SILENCE_WARNING and not self.silent:
            self.silent = True
            self.get_logger().warn(f'No input for {SILENCE_WARNING} s: PX4 is getting nothing')
        elif quiet <= SILENCE_WARNING and self.silent:
            self.silent = False
            self.get_logger().info('Input is back')

    def wall_now(self):
        return self.get_clock().now().nanoseconds * 1e-9


def position_orientation(pose):
    p, o = pose.position, pose.orientation
    return (p.x, p.y, p.z), (o.x, o.y, o.z, o.w)


def main():
    rclpy.init()
    node = ExternalBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
