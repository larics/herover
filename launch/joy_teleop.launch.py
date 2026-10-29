"""Drive one herover with a gamepad: joy_node + joy_teleop in its namespace.

    ros2 launch herover joy_teleop.launch.py                       # herover_1
    ros2 launch herover joy_teleop.launch.py namespace:=herover_2

Hold LB to drive (controls in herover/joy_teleop.py). The cmd_vel_bridge
runs separately.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='herover_1',
            description='robot to drive: publishes /<namespace>/cmd_vel'),
        DeclareLaunchArgument(
            'joy_dev', default_value='0',
            description='gamepad index, see: ros2 run joy joy_enumerate_devices'),
        Node(
            package='joy', executable='joy_node', namespace=namespace,
            parameters=[{
                'device_id': LaunchConfiguration('joy_dev'),
                'deadzone': 0.1,
                # Repeat while the sticks are held still: joy_teleop stops the
                # robot when joy goes quiet for its joy_timeout (0.5 s).
                'autorepeat_rate': 20.0,
            }]),
        Node(
            package='herover', executable='joy_teleop', namespace=namespace,
            output='screen'),
    ])
