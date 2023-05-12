import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nodo1 = Node(
        package='detection_node',
        executable='detector',
        name='detector',
    )

    nodo2 = Node(
        package='detection_node',
        executable='point_cloud_printer',
        name='viewer',
    )
    
    nodo3 = Node(
        package='detection_node',
        executable='pose_extractor',
        name='pose_extractor',
    )
    
    nodo5 = Node(
        package='detection_node',
        executable='serial',
        name='serial',
    )

    return LaunchDescription([
        nodo1,
        nodo2,
        nodo3,
        nodo4,
        nodo5
    ])

if __name__ == '__main__':
    generate_launch_description()