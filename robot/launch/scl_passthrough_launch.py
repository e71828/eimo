from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution, LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    serial_port_launch_arg = DeclareLaunchArgument("serial_port", default_value=TextSubstitution(text="/dev/ttyAMA0"))

    return LaunchDescription([
        serial_port_launch_arg,
        Node(
            package='robot',
            executable='scl',
            parameters=[{"serial_port": LaunchConfiguration("serial_port")}]
        ),
        Node(
            package='robot',
            executable='voltage',
        )

    ])