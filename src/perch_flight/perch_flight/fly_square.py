"""Test flight: fly the simulated X500 around a square using PX4 offboard control over ROS 2.

Start the sim and the Micro XRCE-DDS Agent first (see README.md), then:
    ros2 run perch_flight fly_square

Positions are relative to where the drone started:
north / east in metres, altitude in metres above the start point,
yaw in degrees (0 = facing north, 90 = facing east).
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

ALTITUDE = 5.0

# (north, east, altitude, yaw) -- edit these to fly a different path
WAYPOINTS = [
    (0.0, 0.0, ALTITUDE, 0.0),     # straight up
    (5.0, 0.0, ALTITUDE, 0.0),     # 5 m north
    (5.0, 5.0, ALTITUDE, 90.0),    # 5 m east
    (0.0, 5.0, ALTITUDE, 180.0),   # back south
    (0.0, 0.0, ALTITUDE, 270.0),   # back over the start point
]

TOLERANCE = 0.5      # metres from a waypoint that counts as arrived
HOVER_TIME = 2.0     # seconds to hover at each waypoint
WAYPOINT_TIMEOUT = 30.0
RATE = 10.0          # Hz. PX4 needs setpoints faster than 2 Hz to stay in offboard

# PX4's QoS for /fmu topics; subscribers with a different profile receive nothing
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class FlySquare(Node):

    def __init__(self):
        super().__init__('fly_square')
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_QOS)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', PX4_QOS)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', PX4_QOS)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.on_position, PX4_QOS)
        # VehicleStatus is version 1 of the message, so PX4 adds a _v1 suffix to the topic
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1', self.on_status, PX4_QOS)

        self.position = None
        self.status = None
        self.state = 'wait_for_px4'
        self.state_since = self.now()
        self.waypoint = 0
        self.arrived_at = None
        self.done = False
        self.get_logger().info('Waiting for PX4 (is the Micro XRCE-DDS Agent running?)')
        self.create_timer(1.0 / RATE, self.step)

    def on_position(self, msg):
        self.position = msg

    def on_status(self, msg):
        self.status = msg

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def set_state(self, state):
        self.state = state
        self.state_since = self.now()

    def step(self):
        """Runs at RATE. Keeps setpoints flowing and moves through the flight."""
        if self.state in ('wait_for_px4', 'start', 'arming', 'fly'):
            self.publish_setpoint()
        elapsed = self.now() - self.state_since

        if self.state == 'wait_for_px4':
            if self.ready():
                self.get_logger().info('PX4 ready')
                self.set_state('start')
            elif elapsed > 20.0:
                self.get_logger().warn(
                    f'Still waiting: position={self.position is not None}, '
                    f'status={self.status is not None}', throttle_duration_sec=10.0)

        elif self.state == 'start':
            # Offboard mode refuses to start until it has received setpoints for a moment
            if elapsed > 1.0:
                self.get_logger().info('Switching to offboard and arming')
                self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self.set_state('arming')

        elif self.state == 'arming':
            if self.armed() and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.set_state('fly')
                self.log_waypoint()
            elif elapsed > 5.0:
                self.get_logger().error('Could not arm or enter offboard mode. Check the pxh> terminal.')
                self.done = True

        elif self.state == 'fly':
            self.fly()

        elif self.state == 'landing':
            if not self.armed():
                self.get_logger().info('Landed and disarmed')
                self.done = True

    def fly(self):
        north, east, altitude, _ = WAYPOINTS[self.waypoint]
        p = self.position
        if math.dist((p.x, p.y, p.z), (north, east, -altitude)) < TOLERANCE:
            if self.arrived_at is None:
                self.arrived_at = self.now()
            hovered = self.now() - self.arrived_at >= HOVER_TIME
        else:
            hovered = False
            if self.now() - self.state_since > WAYPOINT_TIMEOUT:
                self.get_logger().warn('   not reached within the timeout, moving on')
                hovered = True

        if hovered:
            self.waypoint += 1
            self.arrived_at = None
            self.state_since = self.now()
            if self.waypoint == len(WAYPOINTS):
                self.get_logger().info('Landing')
                self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.set_state('landing')
            else:
                self.log_waypoint()

    def ready(self):
        return (self.position is not None and self.status is not None
                and self.position.xy_valid and self.position.z_valid
                and self.status.pre_flight_checks_pass)

    def armed(self):
        return self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def log_waypoint(self):
        north, east, altitude, yaw = WAYPOINTS[self.waypoint]
        self.get_logger().info(
            f'-> north={north:.1f}  east={east:.1f}  alt={altitude:.1f}  yaw={yaw:.0f}')

    def publish_setpoint(self):
        """Send the current target. Before takeoff this is the ground under the drone."""
        timestamp = self.get_clock().now().nanoseconds // 1000

        mode = OffboardControlMode()
        mode.timestamp = timestamp
        mode.position = True
        self.offboard_pub.publish(mode)

        north, east, altitude, yaw = WAYPOINTS[self.waypoint] if self.state != 'wait_for_px4' \
            else (0.0, 0.0, 0.0, 0.0)
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = timestamp
        # PX4 uses north-east-DOWN, so "up" is negative
        setpoint.position = [north, east, -altitude]
        setpoint.yaw = math.remainder(math.radians(yaw), 2 * math.pi)  # PX4 wants -pi..pi
        self.setpoint_pub.publish(setpoint)

    def send_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)


def main():
    rclpy.init()
    node = FlySquare()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
