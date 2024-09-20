from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch Gazebo server
        ExecuteProcess(
            cmd=['gzserver', 'empty.world'],
            output='screen'),

        # Wait for Gazebo to initialize
        TimerAction(
            period=5.0,
            actions=[
                # Launch MAVROS nodes
                Node(
                    package='mavros',
                    executable='mavros_node',
                    name='mavros1',
                    parameters=[{'fcu_url': 'udp://:14550@localhost:14560'}],
                    remappings=[('/mavros', '/mavros1')]
                ),
                Node(
                    package='mavros',
                    executable='mavros_node',
                    name='mavros2',
                    parameters=[{'fcu_url': 'udp://:14551@localhost:14561'}],
                    remappings=[('/mavros', '/mavros2')]
                ),

                # Launch your drone nodes
                Node(
                    package='swarm_control',
                    executable='leader_swarm.py',
                    name='leader_swarm'
                ),
                Node(
                    package='swarm_control',
                    executable='follower_drone.py',
                    name='follower_drone_1',
                    remappings=[('/mavros', '/mavros1')]
                ),
                Node(
                    package='swarm_control',
                    executable='follower_drone.py',
                    name='follower_drone_2',
                    remappings=[('/mavros', '/mavros2')]
                )
            ]
        )
    ])