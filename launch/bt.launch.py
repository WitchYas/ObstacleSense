from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('tp5_navigation')
    tree_file = os.path.join(pkg, 'trees', 'navigation.xml')

    return LaunchDescription([
        Node(
            package='tp5_navigation',
            executable='bt_node',
            name='bt_navigator',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'tree_file': tree_file
            }]
        )
    ])
