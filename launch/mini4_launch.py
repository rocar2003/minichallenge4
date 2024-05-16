import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('minichallenge4_bueno'),
            'config',
            'params.yaml'
    )

    generation_node = Node(
        package= 'minichallenge4_bueno',
        executable= 'controller',
        output = 'screen',
        parameters= [{config}]
    )

    traffic_node = Node(
        package= 'minichallenge4_bueno',
        executable= 'traffic_lights',
        output = 'screen',
    )

    ld = LaunchDescription([generation_node, traffic_node])
    return ld