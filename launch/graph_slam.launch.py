from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='graph_slam',
            executable='graph_slam_node',
            name='graph_slam_node',
            output='screen',
            # arguments=['--ros-args', '--log-level', 'debug'],
        )
    ])