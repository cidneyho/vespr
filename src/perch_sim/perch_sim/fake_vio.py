"""Stand-in for the VIO node in Gazebo: the drone's pose and velocity from where it started.

Publishes on the same topic and type as the real VIO (docs/design/fake_vio.md):
    /vespr/perception/odom   nav_msgs/Odometry, frame "odom", child "base_link"
The odom frame's origin is where base_link was in the first Gazebo sample, level, x along the
drone's heading then. Velocity is in base_link. Stamped with the sim time of the Gazebo sample.
Perfect by default; noise, drift, latency and dropped samples are parameters.
Start the sim, then:
    ros2 launch perch_sim fake_vio.launch.py world:=perch_rod
"""
import math

from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException

from perch_sim.gazebo import stamp_ns
from perch_sim.poses import (add_noise, compose, from_yaw, inverse, quat_multiply, rotate,
                             to_rotation_vector, yaw)
from perch_sim.sim_sensor import SimSensor

FRAME = 'odom'
CHILD_FRAME = 'base_link'
COVARIANCE_FLOOR = 1e-6  # so perfect output isn't read as "covariance unknown" (all zeros)


class Odometer:
    """Ground truth to VIO-like output: zeroed at the first pose, with optional drift. No ROS.

    position_drift is m of error per m flown, along a random direction picked when zeroed.
    yaw_drift is rad per s, turning the whole frame about its origin, in a random direction.
    """

    def __init__(self, position_drift, yaw_drift, rng):
        self.position_drift = position_drift
        self.yaw_drift = yaw_drift
        self.rng = rng
        self.origin = None  # inverse of the start pose, level

    def update(self, pose, stamp):
        """(pose in odom, linear velocity, angular velocity in the body) for a world pose at stamp ns."""
        if self.origin is None or stamp < self.last_stamp:  # first sample, or the sim was reset
            self.zero(pose, stamp)
        dt = (stamp - self.last_stamp) * 1e-9
        (position, orientation), (last_position, _) = pose, self.last_pose
        step = [a - b for a, b in zip(position, last_position)]
        self.distance += math.sqrt(sum(c * c for c in step))
        if dt > 0:
            # World velocity turned into the body frame; turning rate from the body's own rotation
            linear = rotate(inverse(pose)[1], [c / dt for c in step])
            turn = quat_multiply(inverse(self.last_pose)[1], orientation)
            angular = tuple(c / dt for c in to_rotation_vector(turn))
        else:
            linear = angular = (0.0, 0.0, 0.0)
        self.last_pose, self.last_stamp = pose, stamp

        (x, y, z), q = compose(self.origin, pose)
        drift_yaw = self.yaw_sign * self.yaw_drift * (stamp - self.start) * 1e-9
        (x, y, z), q = compose(((0.0, 0.0, 0.0), from_yaw(drift_yaw)), ((x, y, z), q))
        error = [c * self.position_drift * self.distance for c in self.drift_direction]
        return ((x + error[0], y + error[1], z + error[2]), q), linear, angular

    def zero(self, pose, stamp):
        position, orientation = pose
        self.origin = inverse((position, from_yaw(yaw(orientation))))
        self.start = self.last_stamp = stamp
        self.last_pose = pose
        self.distance = 0.0
        direction = [self.rng.gauss(0.0, 1.0) for _ in range(3)]
        length = math.sqrt(sum(c * c for c in direction)) or 1.0
        self.drift_direction = [c / length for c in direction]
        self.yaw_sign = self.rng.choice((-1.0, 1.0))


class FakeVio(SimSensor):

    def __init__(self, **kwargs):
        super().__init__('vio', rate=30.0, **kwargs)
        self.linear_velocity_noise = self.declare_parameter('linear_velocity_noise', 0.0).value
        self.angular_velocity_noise = self.declare_parameter('angular_velocity_noise', 0.0).value
        position_drift = self.declare_parameter('position_drift', 0.0).value
        yaw_drift = self.declare_parameter('yaw_drift', 0.0).value  # degrees per minute
        self.odometer = Odometer(position_drift, math.radians(yaw_drift) / 60, self.rng)
        self.pub = self.create_publisher(Odometry, '/vespr/perception/odom', 10)
        self.get_logger().info(
            f'World {self.world}: drone {self.drone_model}/{self.drone_link}. Waiting for Gazebo')
        self.start()

    def sample(self, msg):
        drone = self.drone_pose(msg)
        if drone is None:
            return []
        pose, linear, angular = self.odometer.update(drone, stamp_ns(msg))
        (x, y, z), (qx, qy, qz, qw) = add_noise(
            pose, self.position_noise, self.orientation_noise, self.rng)
        linear = self.jitter(linear, self.linear_velocity_noise)
        angular = self.jitter(angular, self.angular_velocity_noise)

        out = Odometry()
        out.header.frame_id = FRAME
        out.header.stamp = Time(sec=msg.header.stamp.sec, nanosec=msg.header.stamp.nsec)
        out.child_frame_id = CHILD_FRAME
        p, o = out.pose.pose.position, out.pose.pose.orientation
        p.x, p.y, p.z = x, y, z
        o.x, o.y, o.z, o.w = qx, qy, qz, qw
        v, w = out.twist.twist.linear, out.twist.twist.angular
        v.x, v.y, v.z = linear
        w.x, w.y, w.z = angular
        out.pose.covariance = covariance(self.position_noise, self.orientation_noise)
        out.twist.covariance = covariance(self.linear_velocity_noise, self.angular_velocity_noise)
        return [(self.pub, out)]

    def jitter(self, vector, sigma):
        if sigma <= 0:
            return vector
        return tuple(c + self.rng.gauss(0.0, sigma) for c in vector)


def covariance(linear_sigma, angular_sigma):
    """6 x 6 row-major diagonal covariance: three linear axes, then three angular."""
    diagonal = [max(linear_sigma ** 2, COVARIANCE_FLOOR)] * 3 \
        + [max(angular_sigma ** 2, COVARIANCE_FLOOR)] * 3
    matrix = [0.0] * 36
    for i, value in enumerate(diagonal):
        matrix[i * 7] = value
    return matrix


def main():
    rclpy.init()
    node = FakeVio()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
