 #/bin/python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory('snowplow')
    motor_pkg_share = get_package_share_directory('snowplow_motor_controller')
    # xacro_file = os.path.join(pkg_share, 'description', 'robot.urdf_ign.xacro')
    # doc = xacro.process_file(xacro_file, mappings={'use_sim_time': 'false'})
    # xacro.process_doc(doc)

    # Borrowing the motor-controllers diff bot (Using our snowplow urdf doesn't work for some reason but lacking time to diagnose and fix this issue)
    diffbot = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    motor_pkg_share,'launch','diffbot.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'gui': 'false'}.items()
    )

    twist_mux_params = os.path.join(pkg_share,'config','twist_mux.yaml')

    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    )

    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{'use_sim_time': False}],
        remappings=[('/cmd_vel_in', '/diff_cont/cmd_vel_unstamped'),
                    ('cmd_vel_out', '/diff_cont/cmd_vel')]
    )


    return LaunchDescription(
        [
            diffbot,
            joystick,
            twist_mux,
            twist_stamper,
        ]
    )

