"""Sim mocap: /mocap/vespr/pose and /mocap/branch/pose from Gazebo's ground truth.

Start the sim first, in the same world:
    src/perch_sim/scripts/run_sim.sh perch_rod gz_x500_hook
    ros2 launch perch_sim mocap_sim.launch.py world:=perch_rod position_noise:=0.002
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# name: (default, description). The node's parameters of the same names
ARGS = {
    'world': ('perch_rod', 'Gazebo world name. The branch model is named after it'),
    'rate': ('120.0', 'Maximum publish rate, Hz'),
    'position_noise': ('0.0', 'Gaussian position noise, standard deviation in m'),
    'orientation_noise': ('0.0', 'Gaussian noise about each axis, standard deviation in rad'),
    'seed': ('0', 'Random seed; 0 picks a new one each run'),
    'latency': ('0.0', 'Delay before publishing, s'),
    'drop_rate': ('0.0', 'Chance (0 to 1) that a sample is dropped'),
}


def generate_launch_description():
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=default, description=text)
          for name, (default, text) in ARGS.items()),
        Node(
            package='perch_sim',
            executable='mocap_sim',
            parameters=[{name: LaunchConfiguration(name) for name in ARGS}],
        ),
    ])
