from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Path to params file (you can keep this even if not used yet)
    params_file = os.path.join(
        get_package_share_directory('gap_follower'),
        'config',
        'gap_follower_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='gap_follower',
            executable='path_follower_node', 
            name='path_follower_node',
            output='screen',
            parameters=[params_file]
        )
    ])