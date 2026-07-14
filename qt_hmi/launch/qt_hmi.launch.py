from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='qt_hmi',
            executable='qt_hmi_node',
            name='qt_hmi',
            output='screen',
        ),
    ])
