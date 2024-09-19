from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='example_package',
            executable='example_python_node',
            output='screen'
        ),
        Node(
            package='example_package',
            executable='example_cpp_node',
            output='screen'
        )
    ])
