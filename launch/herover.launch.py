"""Everything herover on this machine: cmd_vel bridge, mocap relay, gamepad teleop.

    ros2 launch herover herover.launch.py
    ros2 launch herover herover.launch.py teleop:=false         # no gamepad
    ros2 launch herover herover.launch.py publish_pose:=false   # stack's pose_filter runs
    ros2 launch herover herover.launch.py namespace:=herover_2 config_file:=/path/robots.yaml

- cmd_vel_bridge: /<ns>/cmd_vel -> UDP to every robot in config_file.
- pose_filter: OptiTrack /poses -> /<name>/pose and /<name>/odom. It does
  not start the mocap driver: motion_capture_tracking must run separately.
- joy_teleop.launch.py (teleop:=true): gamepad drives /<namespace>/cmd_vel.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('herover')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=os.path.join(share, 'config', 'robots.yaml'),
            description='robot list for the cmd_vel bridge'),
        DeclareLaunchArgument(
            'publish_pose', default_value='true',
            description='pose_filter also publishes /<name>/pose. Set false when '
                        "hero_mrs_control_stack's own pose_filter runs (multi_agent "
                        'launch, support:=true), so poses are not published twice'),
        DeclareLaunchArgument(
            'teleop', default_value='true',
            description='start joy_node + joy_teleop'),
        DeclareLaunchArgument(
            'namespace', default_value='herover_1',
            description='robot the gamepad drives'),
        DeclareLaunchArgument(
            'joy_dev', default_value='0',
            description='gamepad index, see: ros2 run joy joy_enumerate_devices'),
        Node(
            package='herover', executable='cmd_vel_bridge', output='screen',
            parameters=[{'config_file': LaunchConfiguration('config_file')}]),
        Node(
            package='herover', executable='pose_filter', output='screen',
            parameters=[{'publish_pose': ParameterValue(
                LaunchConfiguration('publish_pose'), value_type=bool)}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'joy_teleop.launch.py')),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'joy_dev': LaunchConfiguration('joy_dev'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('teleop'))),
    ])
