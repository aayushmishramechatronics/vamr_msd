#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *args, **kwargs):
    
    mission_file_value = LaunchConfiguration('mission_file').perform(context)

    overrides = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    if mission_file_value:
        overrides['mission_file'] = mission_file_value

    goal_queue_node = Node(
        package='vamr_msd',
        executable='goal_queue_node.py',
        name='goal_queue_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('params_file'),
            overrides,
        ],
    )
    return [goal_queue_node]


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true.',
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('vamr_msd'), 'config', 'goal_queue.yaml']
        ),
        description='Full path to the goal_queue_node parameters YAML file.',
    )

    mission_file_arg = DeclareLaunchArgument(
        'mission_file',
        default_value='',
        description=(
            'Optional path to a YAML mission file to auto-load at startup. '
            'Leave empty to use whatever mission_file (if any) is set in '
            'params_file instead.'
        ),
    )

    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        mission_file_arg,
        OpaqueFunction(function=_launch_setup),
    ])