import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='swarm_control',
            executable='leader_swarm.py',
            name='leader_drone',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='swarm_control',
            executable='follower_drone.py',
            name='follower_drone_1',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[('/mavros', '/mavros1')]
        ),
        Node(
            package='swarm_control',
            executable='follower_drone.py',
            name='follower_drone_2',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[('/mavros', '/mavros2')]
        )
    ])