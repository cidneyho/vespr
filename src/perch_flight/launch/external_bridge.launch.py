"""external_bridge: mocap or VIO into PX4's estimator (docs/design/external_bridge.md).

source picks the input, and is the stage switch: mocap (stages 1 and 2) or vio (stage 3).
PX4 must be set up for the same source: EV=mocap or EV=vio with run_sim.sh.
    ros2 launch perch_flight external_bridge.launch.py               # mocap
    ros2 launch perch_flight external_bridge.launch.py source:=vio
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# source: (input topic, input_type)
SOURCES = {
    'mocap': ('/mocap/vespr/pose', 'pose'),
    'vio': ('/vespr/perception/odom', 'odom'),
}


def bridge(context):
    source = LaunchConfiguration('source').perform(context)
    if source not in SOURCES:
        raise ValueError(f"source must be one of {', '.join(SOURCES)}, not {source!r}")
    topic, input_type = SOURCES[source]
    return [Node(
        package='perch_flight',
        executable='external_bridge',
        parameters=[{'input_type': input_type}],
        remappings=[('input', topic)],
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('source', default_value='mocap',
                              description='mocap (stages 1 and 2) or vio (stage 3)'),
        OpaqueFunction(function=bridge),
    ])
