import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nodo1 = Node(
        package='detection_node',
        executable='detector',
        name='detector',
    )
    
    nodo4 = Node(
        package='detection_node',
        executable='pose_extractor',
        name='pose_extractor',
    )

    return LaunchDescription([
        nodo1,
        nodo4
    ])

if __name__ == '__main__':
    generate_launch_description()