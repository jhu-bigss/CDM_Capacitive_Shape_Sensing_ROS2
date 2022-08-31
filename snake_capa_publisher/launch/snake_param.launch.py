import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('snake_param_reader'),
        'config',
        'snake1.yaml'
        )
        
    node=Node(
        package = 'snake_param_reader',
        name = 'param_node',
        executable = 'param_reader_node',
        parameters = [config]
    )
    ld.add_action(node)
    return ld