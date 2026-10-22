"""Sim legs: bridge the hook gate topics from Gazebo and start legs_bridge_sim.

Start the sim first: src/perch_sim/scripts/run_sim.sh perch_rod gz_x500_hook
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='hook_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/hook/gate@std_msgs/msg/Float64]gz.msgs.Double',
                '/hook/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            ],
        ),
        Node(
            package='vespr_legs',
            executable='legs_bridge_sim',
            parameters=[{'use_sim_time': True}],
        ),
    ])
