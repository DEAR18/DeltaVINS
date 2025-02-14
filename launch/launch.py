from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
import os

def generate_launch_description():
    delta_vins_share_dir = get_package_share_directory('delta_vins')
    rviz_config_file = os.path.join(delta_vins_share_dir, 'rviz', 'delta_vins.rviz')
    config_file_path = os.path.join(delta_vins_share_dir, 'Config', 'Config.yaml')

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='delta_vins_rviz2',
        arguments=['-d', rviz_config_file],
    )
    delta_vins_node = Node(
            package='delta_vins',
            executable='RunDeltaVINS',
            name='RunDeltaVINS',
            arguments=[config_file_path]
        )
    return LaunchDescription([
        rviz2_node,
        delta_vins_node,
    ])