import os
import sys
import yaml
import json
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    args = []
    length = len(sys.argv)
    if (len(sys.argv) >= 5):
        i = 4
        while i < len(sys.argv):
            args.append(sys.argv[i])
            i = i + 1


    # Demo Program
    moma_node = Node(
        package='omron_moma',
        executable='demo',
        output='screen'
    )
    
    # Destination Publisher
    destination_publisher_node = Node(
        package='pickplace',
        executable='destination_publisher',
        output='screen'
    )

    return LaunchDescription([ 
        moma_node, 
        destination_publisher_node
        ])

