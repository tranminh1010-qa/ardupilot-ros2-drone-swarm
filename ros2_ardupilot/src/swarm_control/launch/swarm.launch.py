import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='swarm_control',
            executable='leader_swarm',
            name='leader_drone',
            output='screen'
        ),
        Node(
            package='swarm_control',
            executable='follower_drone',
            name='follower_drone',
            output='screen'
        )
    ])
