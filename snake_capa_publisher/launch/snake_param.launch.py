from struct import pack
import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='snake_capa_publisher').find('snake_capa_publisher')
    

    capa_mapper_node = launch_ros.actions.Node(
        package='snake_capa_publisher',
        executable='capa_mapper',
        parameters= [os.path.join(pkg_share, 'config', 'snake1.yaml')]
    )

    capa_publisher_node = launch_ros.actions.Node(
        package='snake_capa_publisher',
        executable='capa_publisher'
    )

    gui_arg = DeclareLaunchArgument(name='gui', default_value='false', choices=['true', 'false'],
                                    description='Flag to enable snake_bend_slider_gui')

    snake_rviz = launch.actions.IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('snake_description'), 'launch', 'view_snake_tool.launch.py')),
        launch_arguments={

        }
    )

    
    return launch.LaunchDescription([capa_publisher_node,
        capa_mapper_node,
        gui_arg,
        snake_rviz
        ])