from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    share = get_package_share_directory('pillar_camera_sim')
    params = os.path.join(share, 'config', 'simulation.yaml')
    rviz = os.path.join(share, 'config', 'simulation.rviz')
    return LaunchDescription([
        Node(package='pillar_camera_sim', executable='simulation_node',
             name='pillar_camera_sim', output='screen', parameters=[params]),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz], output='screen'),
    ])
