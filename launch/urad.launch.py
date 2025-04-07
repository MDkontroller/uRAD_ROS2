#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generate a launch description for the radar system.
    This launch file starts the radar publisher and the zero frequency filter nodes, as well as a simpleb subscriber
    """
        
    # Create the launch description
    ld = LaunchDescription()
    
    # Add the radar publisher node
    radar_publisher = Node(
        package='urad',
        executable='urad_publisher_continuous',
        name='radar_publisher',
        output='screen',
    )
    
    # Add the zero frequency filter node
    zero_ff_node = Node(
        package='urad',
        executable='urad_zero_ff',
        name='zero_ff_node',
        output='screen'
    )
    
    # Add the radar subscriber node for visualization or additional processing
    radar_subscriber = Node(
        package='urad',
        executable='urad_subscriber',
        name='radar_subscriber',
        output='screen'
    )
    
    # Add all nodes to the launch description
    ld.add_action(radar_publisher)
    ld.add_action(zero_ff_node)
    ld.add_action(radar_subscriber)
    
    return ld