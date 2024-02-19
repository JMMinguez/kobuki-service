import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # Nodo servidor
    server_node = Node(
        package='service_forward',
        executable='server',
        output='screen',
        remappings=[
            ('l_vel_', '/cmd_vel')
        ]
    )

    # Nodo cliente
    client_node = Node(
        package='service_forward',
        executable='client',
        arguments=['2'],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(server_node)
    ld.add_action(client_node)

    return ld