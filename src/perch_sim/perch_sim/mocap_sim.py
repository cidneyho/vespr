"""Stand-in for the mocap system in Gazebo: drone and branch poses from Gazebo's ground truth.

Publishes on the same topics, type and frame as the real mocap node (docs/design/mocap_sim.md):
    /mocap/vespr/pose    the drone's base_link
    /mocap/branch/pose   the branch model's perch_point link
Both are geometry_msgs/PoseStamped in frame "mocap" (Gazebo's world frame), stamped with the sim
time of the Gazebo sample. Perfect by default; noise, latency and dropped frames are parameters.
Start the sim, then:
    ros2 launch perch_sim mocap_sim.launch.py world:=perch_rod
"""
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException

from perch_sim.sim_sensor import SimSensor

FRAME = 'mocap'


class MocapSim(SimSensor):

    def __init__(self, **kwargs):
        super().__init__('mocap_sim', rate=120.0, **kwargs)
        self.drone_pub = self.create_publisher(PoseStamped, '/mocap/vespr/pose', 10)
        self.branch_pub = self.create_publisher(PoseStamped, '/mocap/branch/pose', 10)
        self.get_logger().info(
            f'World {self.world}: drone {self.drone_model}/{self.drone_link}, '
            f'branch {self.branch_model}/perch_point. Waiting for Gazebo')
        self.start()

    def sample(self, msg):
        drone = self.drone_pose(msg)
        if drone is None:
            return []
        out = [(self.drone_pub, self.pose_msg(drone, msg, FRAME))]
        if self.branch is not None:
            out.append((self.branch_pub, self.pose_msg(self.branch, msg, FRAME)))
        return out


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
