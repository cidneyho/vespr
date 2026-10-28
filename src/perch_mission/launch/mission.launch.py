"""The mission node (with the flight Commander in the same process).

Start the sim, the Micro XRCE-DDS Agent, the branch detector and the legs bridge first, then:
    ros2 launch perch_mission mission.launch.py
    ros2 launch perch_mission mission.launch.py params_file:=/path/to/my_settings.yaml
Settings: config/mission.yaml.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default = os.path.join(get_package_share_directory('perch_mission'), 'config', 'mission.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default,
                              description='Mission and planner settings'),
        Node(
            package='perch_mission',
            executable='mission',
            parameters=[LaunchConfiguration('params_file')],
            output='screen',
        ),
    ])
