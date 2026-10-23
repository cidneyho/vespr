"""Sim branch detector: /vespr/perception/branch_pose from Gazebo's ground truth, while in view.

Start the sim first, in the same world, with the camera-carrying hook drone:
    src/perch_sim/scripts/run_sim.sh perch_rod gz_x500_hook
    ros2 launch vespr_sim fake_branch_detector.launch.py world:=perch_rod
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# name: (default, description). The node's parameters of the same names
ARGS = {
    'world': ('perch_rod', 'Gazebo world name. The branch model is named after it'),
    'rate': ('30.0', 'Maximum publish rate, Hz'),
    'fov_horizontal': ('69.0', 'Camera view width, deg'),
    'fov_vertical': ('42.0', 'Camera view height, deg'),
    'min_range': ('0.3', 'Closest detection ahead of the camera, m'),
    'max_range': ('3.0', 'Furthest detection ahead of the camera, m'),
    'position_noise': ('0.0', 'Gaussian position noise, standard deviation in m'),
    'orientation_noise': ('0.0', 'Gaussian noise about each axis, standard deviation in rad'),
    'latency': ('0.0', 'Delay before publishing, s'),
    'drop_rate': ('0.0', 'Chance (0 to 1) that a sample is dropped'),
}


def generate_launch_description():
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=default, description=text)
          for name, (default, text) in ARGS.items()),
        Node(
            package='vespr_sim',
            executable='fake_branch_detector',
            parameters=[{name: LaunchConfiguration(name) for name in ARGS}],
        ),
    ])
