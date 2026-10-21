"""ROS 2 entry point for the perception subsystem.

This file holds ROS wiring only. The image processing lives in the other
modules of this package so it can be run and tested without ROS.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class PerceptionNode(Node):
    """Find the perch branch in camera images and publish its location."""

    def __init__(self):
        super().__init__('perception_node')

        # Publishers and subscribers get added here once the topics, message
        # types and frames marked TBD in docs/system-map/ are agreed:
        #   in:  camera images    (camera driver / simulation -> perception)
        #   in:  drone pose       (PX4 onboard estimate -> perception)
        #   out: branch location  (perception -> flight control)

        self.get_logger().info('perception_node started')


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
