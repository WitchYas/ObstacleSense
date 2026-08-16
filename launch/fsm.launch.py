from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tp5_navigation',
            executable='fsm_node',
            name='bump_go',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])
