"""Stand-in for the branch detector in Gazebo: where the branch is from the drone, when it's in view.

Publishes /vespr/perception/branch_pose (geometry_msgs/PoseStamped, frame base_link): the branch
model's perch_point from the centre of the drone's body, x along the branch pointing to the
drone's left. Only while the perch point is in the camera's view and range
(docs/design/fake_branch_detector.md). The camera is the drone model's camera_link, read from
Gazebo. Start the sim, then:
    ros2 launch vespr_sim fake_branch_detector.launch.py world:=perch_rod
"""
import math

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException

from vespr_sim.poses import compose, inverse, quat_multiply, rotate
from vespr_sim.sim_sensor import SimSensor

FRAME = 'base_link'
TURN_ABOUT_Z = (0.0, 0.0, 1.0, 0.0)  # 180 deg about z
FORE_AND_AFT = 1e-6  # sideways part of the branch direction below which it counts as fore and aft


def in_view(point, fov_horizontal, fov_vertical, min_range, max_range):
    """Whether a point in the camera frame (x forward, y left, z up) is in view. FOVs in rad."""
    x, y, z = point
    return (min_range <= x <= max_range
            and abs(math.atan2(y, x)) <= fov_horizontal / 2
            and abs(math.atan2(z, x)) <= fov_vertical / 2)


def point_left(orientation):
    """Branch orientation with x along the branch pointing to the drone's left (or forward)."""
    x, y, _ = rotate(orientation, (1.0, 0.0, 0.0))
    if y < -FORE_AND_AFT or (abs(y) <= FORE_AND_AFT and x < 0):
        return quat_multiply(orientation, TURN_ABOUT_Z)
    return orientation


class FakeBranchDetector(SimSensor):

    def __init__(self, **kwargs):
        super().__init__('branch_detector', rate=30.0, **kwargs)
        self.camera_link = self.declare_parameter('camera_link', 'camera_link').value
        self.fov_horizontal = math.radians(self.declare_parameter('fov_horizontal', 69.0).value)
        self.fov_vertical = math.radians(self.declare_parameter('fov_vertical', 42.0).value)
        self.min_range = self.declare_parameter('min_range', 0.3).value
        self.max_range = self.declare_parameter('max_range', 3.0).value
        self.pub = self.create_publisher(PoseStamped, '/vespr/perception/branch_pose', 10)
        self.get_logger().info(
            f'World {self.world}: drone {self.drone_model}, camera {self.camera_link}, '
            f'branch {self.branch_model}/perch_point. Waiting for Gazebo')
        self.start()

    def sample(self, msg):
        if self.branch is None:
            return []
        drone = self.drone_pose(msg)
        camera = self.drone_pose(msg, self.camera_link)
        if drone is None or camera is None:
            return []
        branch_from_camera, _ = compose(inverse(camera), self.branch)
        if not in_view(branch_from_camera, self.fov_horizontal, self.fov_vertical,
                       self.min_range, self.max_range):
            return []
        position, orientation = compose(inverse(drone), self.branch)
        return [(self.pub, self.pose_msg((position, point_left(orientation)), msg, FRAME))]


def main():
    rclpy.init()
    node = FakeBranchDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
