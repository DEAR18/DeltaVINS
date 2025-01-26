from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    delta_vins_share_dir = get_package_share_directory('delta_vins')
    rviz_config_file = os.path.join(delta_vins_share_dir, 'rviz', 'delta_vins.rviz')
    config_file_path = os.path.join(delta_vins_share_dir, 'Config', 'Config.yaml')

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='delta_vins_rviz2',
            arguments=['-d', rviz_config_file]
        ),
        Node(
            package='delta_vins',
            executable='DeltaVINSTest',
            name='delta_vins_test',
            arguments=[config_file_path]
        ),
    ])