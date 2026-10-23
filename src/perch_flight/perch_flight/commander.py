"""Flight commander: everything needed to fly PX4 in offboard mode, behind a small API.

The flight CLI (flight_cli.py) drives it today; the perch mission can drive it later.
Call takeoff(), goto(), move_body(), yaw_by(), hold() and land() from any thread. Only
the RATE Hz timer talks to PX4, so a caller waiting on something never stops the
setpoint stream.

Positions are relative to where PX4's position estimate started (the takeoff spot in sim):
north / east in meters, altitude in meters above that point, yaw in degrees
(0 = facing north, 90 = facing east).
"""
from enum import Enum
import math
import threading

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

RATE = 10.0              # Hz. PX4 needs setpoints faster than 2 Hz to stay in offboard
CRUISE_SPEED = 0.5       # m/s, default speed for moves
MAX_SPEED = 1.0          # m/s, the speed command can't go above this
YAW_RATE = 30.0          # deg/s
TAKEOFF_ALTITUDE = 1.5   # m
ARRIVE_TOLERANCE = 0.15  # m from the target that counts as arrived
COMMAND_RETRY = 1.0      # s between resends of arm / mode / land, which can get lost
START_TIMEOUT = 5.0      # s to arm and enter offboard before giving up

# PX4's QoS for /fmu topics; subscribers with a different profile receive nothing
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

NAV_STATE_NAMES = {
    VehicleStatus.NAVIGATION_STATE_MANUAL: 'manual',
    VehicleStatus.NAVIGATION_STATE_ALTCTL: 'altitude',
    VehicleStatus.NAVIGATION_STATE_POSCTL: 'position',
    VehicleStatus.NAVIGATION_STATE_AUTO_LOITER: 'hold',
    VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF: 'takeoff',
    VehicleStatus.NAVIGATION_STATE_AUTO_LAND: 'land',
    VehicleStatus.NAVIGATION_STATE_OFFBOARD: 'offboard',
}


class Refused(Exception):
    """A command that can't be run right now. The message says why."""


class State(Enum):
    WAIT_FOR_PX4 = 'wait_for_px4'  # no position or status from PX4 yet
    IDLE = 'idle'                  # on the ground, disarmed
    STARTING = 'starting'          # streaming setpoints, asking PX4 to arm and enter offboard
    FLYING = 'flying'              # in offboard, following self.trajectory
    LANDING = 'landing'            # PX4's land mode has the drone; it disarms by itself on touchdown
    PILOT = 'pilot'                # PX4 left offboard in the air (RC switch or failsafe); we don't command


class Trajectory:
    """Target that slides from start to end at a constant speed, turning yaw at the same time.

    Positions are NED in meters, yaw in radians.
    """

    def __init__(self, start, end, speed, start_yaw, end_yaw, yaw_rate, start_time):
        self.start = tuple(start)
        self.end = tuple(end)
        self.speed = speed
        self.start_yaw = start_yaw
        self.end_yaw = end_yaw
        self.start_time = start_time
        self.length = math.dist(self.start, self.end)
        self.move_time = self.length / speed
        # Turn the short way around
        self.yaw_change = math.remainder(end_yaw - start_yaw, 2 * math.pi)
        self.yaw_rate = math.copysign(yaw_rate, self.yaw_change)
        self.yaw_time = abs(self.yaw_change) / yaw_rate

    @classmethod
    def hold(cls, position, yaw, start_time):
        """A target that stays put."""
        return cls(position, position, 1.0, yaw, yaw, 1.0, start_time)

    def done(self, now):
        t = now - self.start_time
        return t >= self.move_time and t >= self.yaw_time

    def sample(self, now):
        """Returns (position, velocity, yaw, yaw_speed) of the target at time `now`."""
        t = max(0.0, now - self.start_time)
        if t >= self.move_time:
            position, velocity = self.end, (0.0, 0.0, 0.0)
        else:
            fraction = t / self.move_time
            position = tuple(s + (e - s) * fraction for s, e in zip(self.start, self.end))
            velocity = tuple((e - s) / self.move_time for s, e in zip(self.start, self.end))

        if t >= self.yaw_time:
            yaw, yaw_speed = self.start_yaw + self.yaw_change, 0.0
        else:
            yaw, yaw_speed = self.start_yaw + self.yaw_rate * t, self.yaw_rate
        return position, velocity, math.remainder(yaw, 2 * math.pi), yaw_speed


class Commander(Node):

    def __init__(self):
        super().__init__('flight_commander')
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

        self.lock = threading.Lock()   # guards everything below against the CLI thread
        self.position = None
        self.status = None
        self.state = State.WAIT_FOR_PX4
        self.state_since = self.now()
        self.last_retry = 0.0
        self.trajectory = None
        self.takeoff_altitude = None   # set while starting from the ground
        self.land_confirmed = False
        self.arrived = True
        self.cruise_speed = CRUISE_SPEED
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
        self.last_retry = 0.0

    # ---- API: safe to call from any thread. Each raises Refused or returns a message ----

    def takeoff(self, altitude=TAKEOFF_ALTITUDE):
        with self.lock:
            if self.state != State.IDLE:
                raise Refused(f'can only take off from the ground (state: {self.state.value})')
            if not self.ready():
                raise Refused('PX4 not ready: no position estimate yet')
            self.trajectory = Trajectory.hold(self.measured_ned(), self.measured_yaw(), self.now())
            self.takeoff_altitude = altitude
            self.set_state(State.STARTING)
            return f'arming and taking off to {altitude:.2f} m'

    def goto(self, north, east, altitude, speed=None):
        with self.lock:
            return self._goto(north, east, altitude, speed)

    def move_body(self, forward, right, up, speed=None):
        """Move relative to where the nose points, starting from the current target."""
        with self.lock:
            self.check_can_move()
            north, east, down = self.trajectory.end
            yaw = self.trajectory.end_yaw
            return self._goto(
                north + forward * math.cos(yaw) - right * math.sin(yaw),
                east + forward * math.sin(yaw) + right * math.cos(yaw),
                -down + up,
                speed)

    def yaw_by(self, degrees):
        """Turn in place. Positive is clockwise seen from above."""
        with self.lock:
            self.check_can_move()
            target = self.trajectory.end
            start_yaw = self.trajectory.end_yaw
            self.trajectory = Trajectory(
                target, target, 1.0, start_yaw, start_yaw + math.radians(degrees),
                math.radians(YAW_RATE), self.now())
            self.arrived = False
            return f'yawing {degrees:+.0f} deg to {math.degrees(self.trajectory.end_yaw):.0f}'

    def hold(self):
        """Stop where the target is now. After a pilot takeover, take control back."""
        with self.lock:
            now = self.now()
            if self.state == State.FLYING:
                position, _, yaw, _ = self.trajectory.sample(now)
                self.trajectory = Trajectory.hold(position, yaw, now)
                self.arrived = False
                return 'holding'
            if self.state == State.PILOT:
                self.trajectory = Trajectory.hold(self.measured_ned(), self.measured_yaw(), now)
                self.takeoff_altitude = None
                self.set_state(State.STARTING)
                return 'taking control back: switching to offboard and holding here'
            raise Refused(f'nothing to hold (state: {self.state.value})')

    def land(self):
        with self.lock:
            if self.state in (State.STARTING, State.FLYING, State.PILOT):
                self.land_confirmed = False
                self.set_state(State.LANDING)
                return 'landing'
            raise Refused(f'not flying (state: {self.state.value})')

    def set_speed(self, speed):
        with self.lock:
            if not 0.0 < speed <= MAX_SPEED:
                raise Refused(f'speed must be above 0 and at most {MAX_SPEED} m/s')
            self.cruise_speed = speed
            return f'cruise speed {speed:.2f} m/s'

    def status_text(self):
        with self.lock:
            lines = [f'state      {self.state.value}']
            if self.status is not None:
                mode = NAV_STATE_NAMES.get(self.status.nav_state, str(self.status.nav_state))
                lines.append(f'px4        {"armed" if self.armed() else "disarmed"}, {mode} mode')
            if self.position is not None:
                north, east, down = self.measured_ned()
                lines.append(f'position   north={north:.2f}  east={east:.2f}  '
                             f'altitude={-down:.2f}  yaw={math.degrees(self.measured_yaw()):.0f}')
            if self.trajectory is not None and self.state == State.FLYING:
                north, east, down = self.trajectory.end
                lines.append(f'target     north={north:.2f}  east={east:.2f}  altitude={-down:.2f}  '
                             f'yaw={math.degrees(self.trajectory.end_yaw):.0f}')
            lines.append(f'speed      {self.cruise_speed:.2f} m/s')
            return '\n'.join(lines)

    def safe_to_exit(self):
        """True once PX4 no longer needs us: on the ground, in land mode, or with the pilot."""
        with self.lock:
            return (self.state in (State.WAIT_FOR_PX4, State.IDLE, State.PILOT)
                    or (self.state == State.LANDING and self.land_confirmed))

    # ---- checks, called with the lock held ----

    def _goto(self, north, east, altitude, speed):
        self.check_can_move()
        start = self.trajectory.end
        end = (north, east, -altitude)
        speed = self.cruise_speed if speed is None else min(speed, MAX_SPEED)
        yaw = self.trajectory.end_yaw
        self.trajectory = Trajectory(
            start, end, speed, yaw, yaw, math.radians(YAW_RATE), self.now())
        self.arrived = False
        return f'-> north={north:.2f}  east={east:.2f}  altitude={altitude:.2f} at {speed:.2f} m/s'

    def check_can_move(self):
        if self.state != State.FLYING:
            raise Refused(f'not flying (state: {self.state.value})')
        if not self.trajectory.done(self.now()):
            raise Refused('still moving; wait for it to arrive, or hold to stop')

    # ---- the RATE Hz loop: the only place that talks to PX4 ----

    def step(self):
        with self.lock:
            if self.position is None or self.status is None:
                return
            now = self.now()
            elapsed = now - self.state_since

            # Stream setpoints in every state except landing, so offboard can start any time.
            # Outside starting / flying the target follows the drone, which PX4 ignores
            if self.state in (State.STARTING, State.FLYING):
                self.publish_setpoint(*self.trajectory.sample(now))
            elif self.state != State.LANDING:
                self.publish_setpoint(self.measured_ned(), (0.0, 0.0, 0.0), self.measured_yaw(), 0.0)

            if self.state == State.WAIT_FOR_PX4:
                if self.ready():
                    self.get_logger().info('PX4 ready')
                    self.set_state(State.IDLE)

            elif self.state == State.STARTING:
                self.step_starting(now, elapsed)

            elif self.state == State.FLYING:
                self.step_flying(now)

            elif self.state == State.LANDING:
                if not self.armed():
                    self.get_logger().info('Landed and disarmed')
                    self.set_state(State.IDLE)
                elif self.status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
                    self.land_confirmed = True
                elif self.land_confirmed:
                    # Was landing, now something else: the pilot switched modes
                    self.get_logger().warn('PX4 left land mode. Pilot has control')
                    self.set_state(State.PILOT)
                elif now - self.last_retry >= COMMAND_RETRY:
                    self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                    self.last_retry = now

            elif self.state == State.PILOT:
                if not self.armed():
                    self.get_logger().info('Disarmed')
                    self.set_state(State.IDLE)

    def step_starting(self, now, elapsed):
        if self.armed() and self.in_offboard():
            if self.takeoff_altitude is not None:
                north, east, down = self.trajectory.end
                yaw = self.trajectory.end_yaw
                self.trajectory = Trajectory(
                    (north, east, down), (north, east, -self.takeoff_altitude), self.cruise_speed,
                    yaw, yaw, math.radians(YAW_RATE), now)
                self.takeoff_altitude = None
                self.get_logger().info('Armed, in offboard, climbing')
            else:
                self.get_logger().info('In offboard, holding')
            self.arrived = False
            self.set_state(State.FLYING)
        elif elapsed > START_TIMEOUT:
            if self.takeoff_altitude is not None:
                if self.in_offboard() and not self.status.pre_flight_checks_pass:
                    self.get_logger().error(
                        'PX4 preflight checks fail in offboard mode, so not arming. Check the pxh> terminal.')
                else:
                    self.get_logger().error('Could not arm or enter offboard mode. Check the pxh> terminal.')
                if self.armed():
                    self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
                self.set_state(State.IDLE)
            else:
                self.get_logger().error('Could not switch back to offboard. Pilot still has control')
                self.set_state(State.PILOT)
        # Offboard mode refuses to start until it has received setpoints for a moment.
        # Switch to offboard first, then arm only once PX4's checks pass for offboard
        elif elapsed > 1.0 and now - self.last_retry >= COMMAND_RETRY:
            if not self.in_offboard():
                self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            elif not self.armed() and self.status.pre_flight_checks_pass:
                self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.last_retry = now

    def step_flying(self, now):
        if not self.armed():
            self.get_logger().warn('Disarmed in flight (kill switch?)')
            self.set_state(State.IDLE)
        elif not self.in_offboard():
            self.get_logger().warn(
                'PX4 left offboard (pilot switch or failsafe). Not commanding. Type hold to take back control')
            self.set_state(State.PILOT)
        elif not self.arrived and self.trajectory.done(now):
            off = math.dist(self.measured_ned(), self.trajectory.end)
            if off < ARRIVE_TOLERANCE:
                self.arrived = True
                self.get_logger().info('Arrived')

    # ---- PX4 helpers ----

    def ready(self):
        """PX4 has a local position estimate, which offboard flight needs.

        PX4's arming checks (pre_flight_checks_pass) are left to step_starting: PX4 judges them
        for the mode it is in, and until we switch it to offboard that is usually Hold, which
        also needs GPS. Indoors, with no GPS, Hold's checks fail though offboard's pass.
        """
        return (self.position is not None and self.status is not None
                and self.position.xy_valid and self.position.z_valid)

    def armed(self):
        return self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def in_offboard(self):
        return self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    def measured_ned(self):
        p = self.position
        return (p.x, p.y, p.z)

    def measured_yaw(self):
        return self.position.heading

    def publish_setpoint(self, position, velocity, yaw, yaw_speed):
        timestamp = self.get_clock().now().nanoseconds // 1000

        mode = OffboardControlMode()
        mode.timestamp = timestamp
        mode.position = True
        self.offboard_pub.publish(mode)

        setpoint = TrajectorySetpoint()
        setpoint.timestamp = timestamp
        setpoint.position = [float(v) for v in position]
        # Velocity is feedforward: PX4 still holds position, but knows the target is moving
        setpoint.velocity = [float(v) for v in velocity]
        setpoint.yaw = float(yaw)
        setpoint.yawspeed = float(yaw_speed)
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
