"""Nav2 on a floor map: the mapless stack, plus a map server and AMCL.

    ros2 launch env/nav2_map.launch.py params:=/tmp/nav2_map.yaml

Started by env/start_nav2_map.sh, which makes that parameter file first
(env/nav2_map_params.py). The vendor's own map launch (dr_nav2) is not used:
it has no AMCL, and its lidar layer is for another lidar.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TF = [('/tf', 'tf'), ('/tf_static', 'tf_static')]      # as nav2_bringup's own launch files do


def generate_launch_description():
    params = LaunchConfiguration('params')
    mapless = os.path.join(get_package_share_directory('dr_nav2_mapless'), 'launch', 'dr_nav2_mapless.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('params'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mapless),
            launch_arguments=[
                ('dr_nav2_config_file', params), ('launch_realsense', 'false'), ('use_rviz', 'false'),
                # For nav2_bringup's navigation launch two includes down, which writes this
                # into the parameters, false by default. With false the costmap subscribes
                # after the map server has sent the map, never gets it, and stays an empty
                # 30 m square with its corner at the origin (seen 2026-10-09).
                ('map_subscribe_transient_local', 'true')]),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', parameters=[params], remappings=TF),
        Node(package='nav2_amcl', executable='amcl', name='amcl',
             output='screen', parameters=[params], remappings=TF),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen',
             parameters=[{'use_sim_time': False, 'autostart': True, 'node_names': ['map_server', 'amcl']}]),
    ])
