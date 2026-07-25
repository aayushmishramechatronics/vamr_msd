#!/usr/bin/env python3
"""
launches vision_assist_node, requires the camera simulation (or real
camera driver) to already be publishing on the configured camera topic,
and benefits from Nav2's controller_server already running (so the
published /speed_limit actually has an effect) - but does not hard-fail
if either isn't up yet, it just won't do anything useful until they are.

example:

    ros2 launch vamr_msd camera.launch.py
    ros2 launch vamr_msd navigation_launch.py
    ros2 launch vamr_msd vision_navigation.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true.',
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('vamr_msd'), 'config', 'vision_nav.yaml']
        ),
        description='Full path to the vision_assist_node parameters YAML file.',
    )

    vision_assist_node = Node(
        package='vamr_msd',
        executable='vision_assist_node.py',
        name='vision_assist_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        vision_assist_node,
    ])