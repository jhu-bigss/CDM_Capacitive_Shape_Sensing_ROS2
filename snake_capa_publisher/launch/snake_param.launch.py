from struct import pack
import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='snake_capa_publisher').find('snake_capa_publisher')
    

    capa_mapper_node = launch_ros.actions.Node(
        package='snake_capa_publisher',
        executable='capa_mapper',
        parameters= [os.path.join(pkg_share, 'config', 'snake1.yaml')]
    )

    return launch.LaunchDescription([capa_mapper_node])