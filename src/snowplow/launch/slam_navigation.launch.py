#/bin/python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from launch_ros.actions import LoadComposableNodes 
from launch_ros.descriptions import ComposableNode

from launch.actions import IncludeLaunchDescription,  DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    pkg_share = get_package_share_directory('snowplow')

    params_file = os.path.join(pkg_share, 'config/nav2/nav2_params.yaml')
    
    param_substitutions = {
        'yaml_filename': os.path.join(pkg_share, 'worlds/slam/competition_map_start.yaml'),
        'use_sim_time': 'True'
        }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True)


    pkg_nav2_bringup = get_package_share_directory('snowplow')
    # Start navigation
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch/navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'True', 
            'params_file': params_file,
            'namespace': '',
            'autostart': 'True',
            'use_composition': 'False',
            'container_name': 'nav2_container',
            'use_respawn': 'False',
            'log_level': 'info'
            }.items(),
    )


    return LaunchDescription(
        [
            nav2_bringup_launch
        ]
    )
