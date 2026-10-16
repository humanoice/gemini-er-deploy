# SPDX-License-Identifier: GPL-3.0-only
"""Run er_goto with config/params.yaml.

ros2 launch er_goto er_goto.launch.py target:=chair [dry_run:=true] [save_debug_images:=true]
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution([FindPackageShare('er_goto'), 'config', 'params.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument(
            'target', description='Object or command, e.g. "chair" or "walk to the red chair"'),
        DeclareLaunchArgument(
            'dry_run', default_value='false',
            description='Publish to /cmd_vel_test instead of /cmd_vel'),
        DeclareLaunchArgument(
            'save_debug_images', default_value='false',
            description='Save each LOOK frame with the Gemini point drawn on it'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Parameter file'),
        Node(
            package='er_goto',
            executable='er_goto_node',
            name='er_goto',
            output='screen',
            emulate_tty=True,
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'target': ParameterValue(LaunchConfiguration('target'), value_type=str),
                    'dry_run': ParameterValue(LaunchConfiguration('dry_run'), value_type=bool),
                    'save_debug_images': ParameterValue(
                        LaunchConfiguration('save_debug_images'), value_type=bool),
                },
            ],
            # The node exits when it arrives or gives up; end the launch with it.
            on_exit=Shutdown(),
        ),
    ])
