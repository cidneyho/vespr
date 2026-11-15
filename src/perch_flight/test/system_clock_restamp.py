"""Test tool, not a unit test: mocap_sim's poses re-stamped with the system clock, as the real
mocap PC stamps them. Lets the sim exercise external_bridge's real-drone clock path (docs/design/
external_bridge.md, Testing): the bridge must detect the system clock and add PX4's time offset.

    python3 src/perch_flight/test/system_clock_restamp.py   # /mocap/vespr/pose -> OUT below
    ros2 run perch_flight external_bridge --ros-args -p input_type:=pose \
        -r input:=/test/system_clock/pose

Each pose is stamped when it arrives here, a little after Gazebo made it, as a real stamp carries
the network's delay.
"""
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

IN, OUT = '/mocap/vespr/pose', '/test/system_clock/pose'


class Restamp(Node):

    def __init__(self):
        super().__init__('system_clock_restamp')
        self.pub = self.create_publisher(PoseStamped, OUT, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, IN, self.on_pose, qos_profile_sensor_data)
        self.get_logger().info(f'{IN} -> {OUT}, stamped with the system clock')

    def on_pose(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()  # system clock: no use_sim_time
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = Restamp()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
