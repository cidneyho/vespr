"""Sim VIO: /vespr/perception/odom from Gazebo's ground truth, zeroed where the drone starts.

Start the sim first, in the same world:
    src/perch_sim/scripts/run_sim.sh perch_rod gz_x500_hook
    ros2 launch vespr_sim fake_vio.launch.py world:=perch_rod position_drift:=0.01
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# name: (default, description). The node's parameters of the same names
ARGS = {
    'world': ('perch_rod', 'Gazebo world name'),
    'drone_model': ('x500_hook_0', 'Gazebo model of the drone'),
    'drone_link': ('base_link', "The drone's body link"),
    'rate': ('30.0', 'Maximum publish rate, Hz'),
    'position_noise': ('0.0', 'Gaussian position noise, standard deviation in m'),
    'orientation_noise': ('0.0', 'Gaussian noise about each axis, standard deviation in rad'),
    'linear_velocity_noise': ('0.0', 'Gaussian velocity noise, standard deviation in m/s'),
    'angular_velocity_noise': ('0.0', 'Gaussian turning-rate noise, standard deviation in rad/s'),
    'position_drift': ('0.0', 'Position drift, m of error per m flown'),
    'yaw_drift': ('0.0', 'Heading drift, degrees per minute'),
    'seed': ('0', 'Random seed; 0 picks a new one each run'),
    'latency': ('0.0', 'Delay before publishing, s'),
    'drop_rate': ('0.0', 'Chance (0 to 1) that a sample is dropped'),
}


def generate_launch_description():
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=default, description=text)
          for name, (default, text) in ARGS.items()),
        Node(
            package='vespr_sim',
            executable='fake_vio',
            parameters=[{name: LaunchConfiguration(name) for name in ARGS}],
        ),
    ])
