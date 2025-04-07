from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='urad',
            executable='urad_publisher',
            name='urad_publisher',
            output='screen'
        ),
        Node(
            package='urad',
            executable='urad_subscriber',
            name='urad_subscriber',
            output='screen'
        )
    ]) 