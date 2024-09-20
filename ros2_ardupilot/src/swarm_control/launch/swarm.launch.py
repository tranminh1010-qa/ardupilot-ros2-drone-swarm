import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    gazebo_pkg = get_package_share_directory('gazebo_ros')
    world_file = os.path.join(gazebo_pkg, 'worlds', 'empty.world')
    param_file = os.path.join(
        get_package_share_directory('swarm_control'),
        'parameters',
        'ardu_gps_noise.param'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_pkg, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world_file}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_pkg, 'launch', 'gzclient.launch.py')
            )
        ),
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
        ),
        Node(
            package='swarm_control',
            executable='flight_data_logger.py',
            name='flight_data_logger',
            output='screen'
        ),
        Node(
            package='ardupilot',
            executable='ardupilot_sim',
            name='ardupilot_sim',
            output='screen',
            parameters=[param_file]
        )

    ])