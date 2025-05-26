from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='btcpp_model',
            executable='btcpp_model',
            output='screen'
        )
    ])